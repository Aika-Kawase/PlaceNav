#!/usr/bin/env python3
"""Safety gate between PlaceNav's nominal command and MotionDecision.

This node does not perform obstacle avoidance.  It only gates and limits the
nominal Twist so that stale commands, missing camera data, goal completion, or
an emergency-stop request produce a zero command.
"""

import argparse
import time
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class SafetyFilterNode:
    def __init__(self, robot: str, robot_config_path: Path):
        with robot_config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)[robot]

        safety = config.get("safety", {})
        self.nominal_topic = safety.get("nominal_cmd_topic", config["vel_navi_topic"])
        self.output_topic = safety.get("output_cmd_topic", "/local_path/cmd_vel")
        self.camera_topic = config["camera_topic"]
        self.emergency_topic = safety.get("emergency_stop_topic", "/emergency_stop")
        self.stop_topic = safety.get("safety_stop_topic", "/placenav/safety_stop")
        self.reason_topic = safety.get("safety_reason_topic", "/placenav/safety_reason")

        self.command_timeout = float(safety.get("command_timeout", 0.75))
        self.camera_timeout = float(safety.get("camera_timeout", 0.75))
        self.publish_rate = float(safety.get("publish_rate", 20.0))
        self.max_v = float(config["max_v"])
        self.max_w = float(config["max_w"])
        self.max_accel = float(safety.get("max_accel", 0.10))
        self.max_angular_accel = float(safety.get("max_angular_accel", 0.20))
        self.require_emergency_stop = bool(safety.get("require_emergency_stop", True))

        for name, value in (
            ("command_timeout", self.command_timeout),
            ("camera_timeout", self.camera_timeout),
            ("publish_rate", self.publish_rate),
            ("max_accel", self.max_accel),
            ("max_angular_accel", self.max_angular_accel),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")

        self.latest_nominal = Twist()
        self.last_command_time = None
        self.last_camera_time = None
        self.last_loop_time = time.monotonic()
        self.emergency_state = None
        self.manual_stop = False
        self.goal_reached = False
        self.last_reason = "startup"
        self.output = Twist()

        rospy.init_node("placenav_safety_filter", anonymous=False)
        self.output_pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.reason_pub = rospy.Publisher(self.reason_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.nominal_topic, Twist, self.nominal_callback, queue_size=1)
        rospy.Subscriber(self.camera_topic, Image, self.camera_callback, queue_size=1)
        rospy.Subscriber(self.emergency_topic, Bool, self.emergency_callback, queue_size=1)
        rospy.Subscriber(self.stop_topic, Bool, self.stop_callback, queue_size=1)
        rospy.Subscriber("/toponav/reached_goal", Bool, self.goal_callback, queue_size=1)
        rospy.on_shutdown(self.publish_stop)

        self.rate = rospy.Rate(self.publish_rate)
        rospy.loginfo(
            "Safety filter: %s -> %s (camera=%s, emergency=%s)",
            self.nominal_topic,
            self.output_topic,
            self.camera_topic,
            self.emergency_topic,
        )

    @staticmethod
    def _finite(value: float) -> bool:
        return value == value and abs(value) != float("inf")

    def nominal_callback(self, msg: Twist):
        if not all(self._finite(value) for value in (msg.linear.x, msg.angular.z)):
            self.latest_nominal = Twist()
            self.last_command_time = time.monotonic()
            self.last_reason = "invalid_nominal_command"
            return
        self.latest_nominal = msg
        self.last_command_time = time.monotonic()

    def camera_callback(self, _msg: Image):
        self.last_camera_time = time.monotonic()

    def emergency_callback(self, msg: Bool):
        self.emergency_state = bool(msg.data)

    def stop_callback(self, msg: Bool):
        self.manual_stop = bool(msg.data)

    def goal_callback(self, msg: Bool):
        self.goal_reached = bool(msg.data)

    def stop_required(self, now: float):
        if self.require_emergency_stop and self.emergency_state is None:
            return True, "waiting_for_emergency_stop"
        if self.emergency_state:
            return True, "emergency_stop"
        if self.manual_stop:
            return True, "safety_stop"
        if self.goal_reached:
            return True, "goal_reached"
        if self.last_command_time is None or now - self.last_command_time > self.command_timeout:
            return True, "nominal_command_timeout"
        if self.last_camera_time is None or now - self.last_camera_time > self.camera_timeout:
            return True, "camera_timeout"
        return False, "ok"

    def limited_target(self, dt: float):
        target_v = max(-self.max_v, min(self.max_v, self.latest_nominal.linear.x))
        target_w = max(-self.max_w, min(self.max_w, self.latest_nominal.angular.z))
        max_dv = self.max_accel * dt
        max_dw = self.max_angular_accel * dt
        next_v = max(self.output.linear.x - max_dv, min(self.output.linear.x + max_dv, target_v))
        next_w = max(self.output.angular.z - max_dw, min(self.output.angular.z + max_dw, target_w))
        return next_v, next_w

    def publish_stop(self):
        self.output = Twist()
        for _ in range(3):
            self.output_pub.publish(self.output)
            time.sleep(0.02)

    def run(self):
        while not rospy.is_shutdown():
            now = time.monotonic()
            dt = max(1e-3, now - self.last_loop_time)
            self.last_loop_time = now
            should_stop, reason = self.stop_required(now)
            if should_stop:
                self.output = Twist()
            else:
                self.output.linear.x, self.output.angular.z = self.limited_target(dt)

            self.output_pub.publish(self.output)
            if reason != self.last_reason:
                self.reason_pub.publish(String(data=reason))
                rospy.logwarn("Safety filter: %s", reason) if reason != "ok" else rospy.loginfo("Safety filter: clear")
                self.last_reason = reason
            self.rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser(description="Gate PlaceNav nominal Twist before MotionDecision")
    parser.add_argument("--robot", required=True)
    parser.add_argument("--robot-config-path", type=Path, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    SafetyFilterNode(args.robot, args.robot_config_path).run()
