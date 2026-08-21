#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        quaternion.w * quaternion.w
        + quaternion.x * quaternion.x
        - quaternion.y * quaternion.y
        - quaternion.z * quaternion.z,
    )


def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class TestGnmPathPublisher:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/placenav/gnm_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.publish_rate = rospy.get_param("~publish_rate", 5.0)
        self.step = rospy.get_param("~waypoint_step", 0.6)
        self.curvature = rospy.get_param("~curvature", 0.0)
        self.configured_points = rospy.get_param("~local_points", [])
        self.use_rviz_goal = rospy.get_param("~use_rviz_goal", False)
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal_pose_count = max(2, rospy.get_param("~goal_pose_count", 6))
        self.goal_min_x = rospy.get_param("~goal_min_x", -5.5)
        self.goal_max_x = rospy.get_param("~goal_max_x", 5.5)
        self.goal_min_y = rospy.get_param("~goal_min_y", -5.5)
        self.goal_max_y = rospy.get_param("~goal_max_y", 5.5)
        self.start_delay = rospy.get_param("~start_delay", 0.0)
        self.first_odom_time = None
        self.latest_odometry = None
        self.path = None

        self.publisher = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.bounds_publisher = rospy.Publisher(
            "/placenav_teb_test/goal_bounds", Marker, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        self.goal_subscriber = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)
        self.publish_goal_bounds()

    def publish_goal_bounds(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "placenav_teb_test"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            Point(x=self.goal_min_x, y=self.goal_min_y),
            Point(x=self.goal_max_x, y=self.goal_min_y),
            Point(x=self.goal_max_x, y=self.goal_max_y),
            Point(x=self.goal_min_x, y=self.goal_max_y),
            Point(x=self.goal_min_x, y=self.goal_min_y),
        ]
        self.bounds_publisher.publish(marker)

    def odom_callback(self, odometry):
        self.latest_odometry = odometry
        if self.use_rviz_goal:
            return
        if self.path is not None:
            return
        if self.first_odom_time is None:
            self.first_odom_time = rospy.Time.now()
        if (rospy.Time.now() - self.first_odom_time).to_sec() < self.start_delay:
            return

        origin_x = odometry.pose.pose.position.x
        origin_y = odometry.pose.pose.position.y
        origin_yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        if self.configured_points:
            if len(self.configured_points) < 2:
                rospy.logfatal("~local_points must contain at least two [forward, left] points")
                rospy.signal_shutdown("invalid local_points")
                return
            try:
                local_points = [
                    (float(point[0]), float(point[1]))
                    for point in self.configured_points
                ]
            except (TypeError, ValueError, IndexError):
                rospy.logfatal("~local_points must be a list of [forward, left] pairs")
                rospy.signal_shutdown("invalid local_points")
                return
        else:
            local_points = [(0.0, 0.0)]
            for index in range(1, 6):
                local_x = self.step * index
                local_y = self.curvature * local_x * local_x
                local_points.append((local_x, local_y))

        path = Path()
        path.header.frame_id = self.frame_id
        for index, (local_x, local_y) in enumerate(local_points):
            if index + 1 < len(local_points):
                next_x, next_y = local_points[index + 1]
                local_heading = math.atan2(next_y - local_y, next_x - local_x)
            else:
                previous_x, previous_y = local_points[index - 1]
                local_heading = math.atan2(local_y - previous_y, local_x - previous_x)

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = origin_x + cos_yaw * local_x - sin_yaw * local_y
            pose.pose.position.y = origin_y + sin_yaw * local_x + cos_yaw * local_y
            pose.pose.orientation = quaternion_from_yaw(origin_yaw + local_heading)
            path.poses.append(pose)

        self.path = path
        rospy.loginfo(
            "Fixed test GNM Path created at (%.3f, %.3f, %.3f), "
            "%d poses, final local point (%.3f, %.3f)",
            origin_x,
            origin_y,
            origin_yaw,
            len(local_points),
            local_points[-1][0],
            local_points[-1][1],
        )

    def goal_callback(self, goal):
        if not self.use_rviz_goal:
            return
        if self.latest_odometry is None:
            rospy.logwarn("Ignoring RViz goal until odometry is available")
            return
        if goal.header.frame_id != self.frame_id:
            rospy.logwarn(
                "Ignoring RViz goal in frame '%s'; RViz Fixed Frame must be '%s'",
                goal.header.frame_id,
                self.frame_id,
            )
            return
        if not (
            self.goal_min_x <= goal.pose.position.x <= self.goal_max_x
            and self.goal_min_y <= goal.pose.position.y <= self.goal_max_y
        ):
            rospy.logwarn(
                "Ignoring RViz goal (%.3f, %.3f): outside test bounds "
                "x=[%.3f, %.3f], y=[%.3f, %.3f]",
                goal.pose.position.x,
                goal.pose.position.y,
                self.goal_min_x,
                self.goal_max_x,
                self.goal_min_y,
                self.goal_max_y,
            )
            return

        start = self.latest_odometry.pose.pose.position
        goal_position = goal.pose.position
        dx = goal_position.x - start.x
        dy = goal_position.y - start.y
        heading = math.atan2(dy, dx)

        path = Path()
        path.header.frame_id = self.frame_id
        for index in range(self.goal_pose_count):
            ratio = float(index) / float(self.goal_pose_count - 1)
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = start.x + ratio * dx
            pose.pose.position.y = start.y + ratio * dy
            pose.pose.orientation = (
                goal.pose.orientation
                if index == self.goal_pose_count - 1
                else quaternion_from_yaw(heading)
            )
            path.poses.append(pose)
        self.path = path
        rospy.loginfo(
            "RViz goal converted to test Path: start (%.3f, %.3f), goal (%.3f, %.3f)",
            start.x,
            start.y,
            goal_position.x,
            goal_position.y,
        )

    def publish(self, _event):
        if self.path is None:
            return
        stamp = rospy.Time.now()
        self.path.header.stamp = stamp
        for pose in self.path.poses:
            pose.header.stamp = stamp
        self.publisher.publish(self.path)


def main():
    rospy.init_node("test_gnm_path_publisher")
    TestGnmPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
