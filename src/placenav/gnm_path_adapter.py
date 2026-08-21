#!/usr/bin/env python3

import argparse
import math
from typing import Iterable, List, Sequence, Tuple

import rospy
import tf2_geometry_msgs  # noqa: F401; registers geometry message conversions
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from placenav_viz_msgs.msg import Viz


XY = Tuple[float, float]


def waypoint_yaws(points: Sequence[XY]) -> List[float]:
    """Return a forward-facing yaw for every point in a local path."""
    if not points:
        return []
    if len(points) == 1:
        return [0.0]

    yaws = []
    previous_yaw = 0.0
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            dx = next_point[0] - point[0]
            dy = next_point[1] - point[1]
        else:
            previous_point = points[index - 1]
            dx = point[0] - previous_point[0]
            dy = point[1] - previous_point[1]

        if math.hypot(dx, dy) > 1e-9:
            previous_yaw = math.atan2(dy, dx)
        yaws.append(previous_yaw)
    return yaws


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def local_poses(
    waypoints: Iterable[Point],
    stamp: rospy.Time,
    frame_id: str,
    include_origin: bool,
) -> List[PoseStamped]:
    points = [(point.x, point.y) for point in waypoints]
    if include_origin:
        points.insert(0, (0.0, 0.0))

    yaws = waypoint_yaws(points)
    poses = []
    for (x, y), yaw in zip(points, yaws):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = quaternion_from_yaw(yaw)
        poses.append(pose)
    return poses


class GnmPathAdapter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/toponav/viz_info")
        self.output_topic = rospy.get_param("~output_topic", "/placenav/gnm_path")
        self.source_frame = rospy.get_param("~source_frame", "base_link")
        self.target_frame = rospy.get_param("~target_frame", "odom")
        self.include_origin = rospy.get_param("~include_origin", True)
        self.expected_waypoints = rospy.get_param("~expected_waypoints", 5)
        self.transform_timeout = rospy.Duration(
            rospy.get_param("~transform_timeout", 0.2)
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.path_publisher = rospy.Publisher(self.output_topic, Path, queue_size=1)
        self.viz_subscriber = rospy.Subscriber(
            self.input_topic, Viz, self.viz_callback, queue_size=1
        )

        rospy.loginfo(
            "GNM path adapter: %s (%s) -> %s (%s)",
            self.input_topic,
            self.source_frame,
            self.output_topic,
            self.target_frame,
        )

    def viz_callback(self, message: Viz):
        if not message.waypoints:
            rospy.logwarn_throttle(2.0, "GNM path contains no waypoints")
            return
        if len(message.waypoints) != self.expected_waypoints:
            rospy.logwarn_throttle(
                2.0,
                "Expected %d GNM waypoints but received %d",
                self.expected_waypoints,
                len(message.waypoints),
            )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                message.query_timestamp,
                self.transform_timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(
                1.0,
                "Cannot transform GNM path from %s to %s at %.9f: %s",
                self.source_frame,
                self.target_frame,
                message.query_timestamp.to_sec(),
                error,
            )
            return

        path = Path()
        path.header.stamp = message.query_timestamp
        path.header.frame_id = self.target_frame
        path.poses = [
            tf2_geometry_msgs.do_transform_pose(pose, transform)
            for pose in local_poses(
                message.waypoints,
                message.query_timestamp,
                self.source_frame,
                self.include_origin,
            )
        ]
        for pose in path.poses:
            pose.header = path.header
        self.path_publisher.publish(path)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert PlaceNav GNM waypoints into an odom-frame nav_msgs/Path"
    )
    parser.add_argument("--input-topic", default="/toponav/viz_info")
    parser.add_argument("--output-topic", default="/placenav/gnm_path")
    parser.add_argument("--source-frame", default="base_link")
    parser.add_argument("--target-frame", default="odom")
    parser.add_argument("--transform-timeout", type=float, default=0.2)
    parser.add_argument("--expected-waypoints", type=int, default=5)
    parser.add_argument("--no-origin", action="store_true")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("gnm_path_adapter", anonymous=False)
    args = parse_args()
    rospy.set_param("~input_topic", args.input_topic)
    rospy.set_param("~output_topic", args.output_topic)
    rospy.set_param("~source_frame", args.source_frame)
    rospy.set_param("~target_frame", args.target_frame)
    rospy.set_param("~transform_timeout", args.transform_timeout)
    rospy.set_param("~expected_waypoints", args.expected_waypoints)
    rospy.set_param("~include_origin", not args.no_origin)
    GnmPathAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
