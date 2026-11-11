import argparse
from collections import deque
import json
import math
import shutil
import pickle
import time
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import yaml

import rospy
from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry
from utils import bridge, odometry_msg_to_numpy, to_local_coords
from topomap_odometry import OdomSample, interpolate_odometry, relative_pose


def ros_stamp(stamp_ns):
    return {"secs": stamp_ns // 1_000_000_000,
            "nsecs": stamp_ns % 1_000_000_000}


class TopomapGeneratorNode:
    def __init__(self, args):
        self.args = args
        self.lock = Lock()
        self.pending_image = None
        self.image_odometry = None
        self.latest_odometry = None
        self.odom_buffer = deque(maxlen=1000)
        self.previous_odometry = None
        self.last_saved_odometry = None
        self.last_image_time = None
        self.odom_error = None
        self.traj_data = {'position': [], 'yaw': [],
                          'relative_position': [], 'relative_yaw': []}
        self.node_data = {
            "schema_version": 1,
            "units": {"position": "m", "angle": "rad"},
            "relative_frame": "from_node",
            "sampling": {
                "mode": args.sampling_mode,
                "straight_distance": args.straight_distance,
                "turn_distance": args.turn_distance,
                "yaw_threshold": args.yaw_threshold,
                "min_distance": args.min_distance,
                "odom_max_gap": args.odom_max_gap,
                "max_node_distance": args.max_node_distance,
            },
            "nodes": [], "edges": [],
        }

        rospy.init_node("CREATE_TOPOMAP", anonymous=False)
        with args.robot_config_path.open(encoding="utf-8") as stream:
            robot_config = yaml.safe_load(stream)[args.robot]
        self.image_topic = robot_config["camera_topic"]
        self.odometry_topic = args.odom_topic or robot_config["odom_topic"]
        self.node_data["odom_topic"] = self.odometry_topic
        self.topomap_name_dir = args.topomap_directory / args.route_name
        if self.topomap_name_dir.exists():
            if args.map_mode == "odometry":
                raise FileExistsError("Map already exists; choose a new route name: " +
                                      str(self.topomap_name_dir))
            # Preserve the legacy generator's explicit same-route replacement.
            rospy.loginfo("%s already exists. Removing previous images...",
                          self.topomap_name_dir)
            for path in self.topomap_name_dir.iterdir():
                if path.is_dir() and not path.is_symlink():
                    shutil.rmtree(path)
                else:
                    path.unlink()
        self.topomap_name_dir.mkdir(parents=True, exist_ok=True)
        self.idle_timeout = args.idle_timeout
        if self.idle_timeout is None:
            self.idle_timeout = 2 * args.dt if args.map_mode == "legacy" else 0
        self.image_sub = rospy.Subscriber(self.image_topic, Image,
                                         self.callback_obs, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.callback_joy)
        self.odom_sub = rospy.Subscriber(self.odometry_topic, Odometry,
                                        self.callback_odometry, queue_size=100)
        rospy.loginfo("Map mode=%s, odometry=%s. Waiting for images...",
                      args.map_mode, self.odometry_topic)

    def callback_obs(self, msg):
        with self.lock:
            self.last_image_time = time.monotonic()
            self.pending_image = msg
            # Legacy mode uses the latest odometry available at image receipt.
            self.image_odometry = self.latest_odometry

    def callback_joy(self, msg):
        if msg.buttons and msg.buttons[0]:
            rospy.signal_shutdown("shutdown")

    def callback_odometry(self, msg):
        if self.args.map_mode == "legacy":
            with self.lock:
                self.latest_odometry = msg
            return
        pose = tuple(float(value) for value in odometry_msg_to_numpy(msg))
        q = msg.pose.pose.orientation
        if (msg.header.stamp.is_zero() or not msg.header.frame_id or
                not msg.child_frame_id or not all(math.isfinite(v) for v in pose) or
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or
                q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12):
            rospy.logwarn_throttle(5, "Ignoring invalid odometry pose/stamp/frames")
            return
        sample = OdomSample(msg.header.stamp.to_nsec(), pose,
                            msg.header.frame_id, msg.child_frame_id)
        error = None
        with self.lock:
            if self.odom_buffer:
                last = self.odom_buffer[-1]
                if sample.stamp_ns < last.stamp_ns:
                    error = "Odometry time moved backwards; use a new map after reset"
                elif (sample.frame_id, sample.child_frame_id) != (
                        last.frame_id, last.child_frame_id):
                    error = "Odometry frames changed; refusing to mix coordinate frames"
                elif sample.stamp_ns == last.stamp_ns:
                    self.odom_buffer.pop()
            if error is None:
                self.odom_buffer.append(sample)
        if error:
            self.odom_error = error
            rospy.logerr(error)
            rospy.signal_shutdown(error)

    def take_observation(self):
        with self.lock:
            msg = self.pending_image
            latest = self.image_odometry
            samples = list(self.odom_buffer)
        if msg is None:
            return None
        match = None
        if self.args.map_mode == "odometry":
            match = interpolate_odometry(
                samples, msg.header.stamp.to_nsec(),
                int(self.args.odom_max_gap * 1_000_000_000))
            if match is None:
                rospy.logwarn_throttle(
                    5, "Waiting for odometry bracketing the image timestamp "
                       "(max interval %.3fs)", self.args.odom_max_gap)
                return None
            pose = np.asarray(match[0].pose)
        elif latest is not None:
            pose = odometry_msg_to_numpy(latest).astype(float)
        else:
            return None
        with self.lock:
            if self.pending_image is msg:
                self.pending_image = None
        return msg, pose, match

    def record_node(self, msg, pose, match, filename):
        sample, before_ns, after_ns = match
        nodes = self.node_data["nodes"]
        self.node_data["frame_id"] = sample.frame_id
        self.node_data["child_frame_id"] = sample.child_frame_id
        self.node_data["image_topic"] = self.image_topic
        if nodes:
            previous = nodes[-1]["pose"]
            delta = relative_pose(
                (previous["x"], previous["y"], previous["yaw"]), pose)
            self.node_data["edges"].append({
                "from": len(nodes) - 1, "to": len(nodes),
                "dx": delta[0], "dy": delta[1], "dyaw": delta[2],
                "distance": math.hypot(delta[0], delta[1]),
            })
        nodes.append({
            "id": len(nodes), "image": filename,
            "stamp": ros_stamp(msg.header.stamp.to_nsec()),
            "pose": dict(zip(("x", "y", "yaw"), map(float, pose))),
            "odom_before_stamp": ros_stamp(before_ns),
            "odom_after_stamp": ros_stamp(after_ns),
        })
        destination = self.topomap_name_dir / "node_odometry.json"
        temporary = destination.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(self.node_data, stream, indent=2, allow_nan=False)
            stream.write("\n")
        temporary.replace(destination)

    def main_loop(self):
        period = 0.1 if self.args.sampling_mode == "adaptive" else self.args.dt
        next_sample_time = 0.0
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now().to_sec()
                if now >= next_sample_time:
                    observation = self.take_observation()
                    if observation is not None:
                        next_sample_time = now + period
                        msg, pose, match = observation
                        reference = (self.previous_odometry if self.args.map_mode == "legacy"
                                     else self.last_saved_odometry)
                        pose_delta = (np.zeros(3) if reference is None else
                                      np.asarray(relative_pose(reference, pose)))
                        # Preserve legacy pkl deltas, including skipped observations.
                        if self.args.map_mode == "legacy" and reference is not None:
                            pose_delta = to_local_coords(
                                pose, reference, reference[2]).astype(float)
                        self.previous_odometry = pose.copy()
                        save_image = self.last_saved_odometry is None
                        if not save_image:
                            delta = (to_local_coords(pose, self.last_saved_odometry,
                                                     self.last_saved_odometry[2]).astype(float)
                                     if self.args.map_mode == "legacy" else
                                     np.asarray(relative_pose(self.last_saved_odometry, pose)))
                            distance = float(np.linalg.norm(delta[:2]))
                            yaw_delta = abs(float(delta[2]))
                            threshold = (self.args.turn_distance if yaw_delta >= self.args.yaw_threshold
                                         else self.args.straight_distance)
                            save_image = (self.args.sampling_mode == "time" or
                                          (distance >= self.args.min_distance and
                                           (distance >= threshold or yaw_delta >= self.args.yaw_threshold)))
                        if save_image:
                            if (self.args.map_mode == "odometry" and
                                    self.last_saved_odometry is not None and
                                    self.args.max_node_distance is not None and
                                    distance > self.args.max_node_distance):
                                raise ValueError(
                                    "Node gap %.3fm exceeds --max-node-distance %.3fm; "
                                    "refusing to connect across missing input. "
                                    "Only the saved map prefix remains (%d nodes)." %
                                    (distance, self.args.max_node_distance,
                                     len(self.traj_data["position"])))
                            filename = str(len(self.traj_data["position"])) + ".jpg"
                            rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                            if not cv2.imwrite(str(self.topomap_name_dir / filename),
                                               cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                                raise OSError("Could not save image: " + filename)
                            self.traj_data["position"].append(pose[:2].copy())
                            self.traj_data["yaw"].append(float(pose[2]))
                            self.traj_data["relative_position"].append(pose_delta[:2].copy())
                            self.traj_data["relative_yaw"].append(float(pose_delta[2]))
                            if match is not None:
                                self.record_node(msg, pose, match, filename)
                            self.last_saved_odometry = pose.copy()
                            rospy.loginfo("Saved node %s at x=%.3f y=%.3f yaw=%.3f",
                                          filename, *pose)
                if (self.idle_timeout > 0 and self.last_image_time is not None and
                        time.monotonic() - self.last_image_time > self.idle_timeout):
                    rospy.loginfo("Image input stopped. Finishing map.")
                    break
                # Wall sleep also allows shutdown/timeout while bag /clock is paused.
                time.sleep(0.005)
        finally:
            with (self.topomap_name_dir / "traj_data.pkl").open("wb") as stream:
                pickle.dump(self.traj_data, stream)
        if self.odom_error:
            raise ValueError(self.odom_error)


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Generate image topomaps from ROS topics")
    parser.add_argument("--robot", required=True)
    parser.add_argument("--robot_config_path", type=Path, required=True)
    parser.add_argument("--route_name", "-r", default="topomap")
    parser.add_argument("--dt", "-t", type=positive_float, default=3.0,
                        help="Time sampling period; not a distance")
    parser.add_argument("--sampling-mode", choices=["time", "adaptive"], default="time")
    parser.add_argument("--straight-distance", type=positive_float, default=3.0)
    parser.add_argument("--turn-distance", type=positive_float, default=0.15)
    parser.add_argument("--yaw-threshold", type=positive_float, default=0.12)
    parser.add_argument("--min-distance", type=positive_float, default=0.10)
    parser.add_argument("--map-mode", choices=["legacy", "odometry"], default="legacy")
    parser.add_argument("--odom-topic", help="nav_msgs/Odometry input; WHILL default: /odom")
    parser.add_argument("--odom-max-gap", type=positive_float, default=0.2,
                        help="Maximum bracketing odometry interval in seconds")
    parser.add_argument("--max-node-distance", type=positive_float,
                        help="Maximum node separation in odometry mode; abort before "
                             "saving a longer edge. Default: 2*straight-distance "
                             "with adaptive sampling, no limit with time sampling")
    parser.add_argument("--idle-timeout", type=float,
                        help="Wall seconds without images; 0 waits for Ctrl+C. "
                             "Default: 2*dt in legacy, 0 in odometry mode")
    parser.add_argument("--topomap_directory", type=Path, default=Path("placenav/topomaps/images"))
    args = parser.parse_args(argv)
    if (args.map_mode == "odometry" and args.sampling_mode == "adaptive" and
            args.max_node_distance is None):
        args.max_node_distance = 2 * args.straight_distance
    if args.idle_timeout is not None and (not math.isfinite(args.idle_timeout) or args.idle_timeout < 0):
        parser.error("--idle-timeout must be finite and nonnegative")
    return args


if __name__ == "__main__":
    try:
        TopomapGeneratorNode(parse_args()).main_loop()
    except (OSError, ValueError) as error:
        rospy.logfatal("%s", error)
        raise SystemExit(1)
