#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class TestObstacleCloudPublisher:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/stvl_test_points")
        self.frame_id = rospy.get_param("~frame_id", "robot_0/odom")
        self.center_x = rospy.get_param("~center_x", -2.0)
        self.center_y = rospy.get_param("~center_y", -0.9)
        self.half_width_x = rospy.get_param("~half_width_x", 0.15)
        self.half_width_y = rospy.get_param("~half_width_y", 0.15)
        self.min_z = rospy.get_param("~min_z", 0.2)
        self.max_z = rospy.get_param("~max_z", 1.0)
        self.interactive = rospy.get_param("~interactive", False)
        self.publisher = rospy.Publisher(self.topic, PointCloud2, queue_size=1)
        self.points = self.make_points()
        self.server = None
        if self.interactive:
            self.make_interactive_marker()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)

    def make_interactive_marker(self):
        self.server = InteractiveMarkerServer("test_obstacle")
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = self.frame_id
        interactive_marker.name = "obstacle"
        interactive_marker.description = "Drag obstacle in XY"
        interactive_marker.pose.position.x = self.center_x
        interactive_marker.pose.position.y = self.center_y
        interactive_marker.pose.position.z = 0.5 * (self.min_z + self.max_z)
        interactive_marker.scale = max(
            0.5, 2.0 * self.half_width_x, 2.0 * self.half_width_y
        )

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = 2.0 * self.half_width_x
        cube.scale.y = 2.0 * self.half_width_y
        cube.scale.z = self.max_z - self.min_z
        cube.color.r = 1.0
        cube.color.g = 0.15
        cube.color.b = 0.05
        cube.color.a = 0.8

        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.orientation.w = 0.70710678
        move_control.orientation.y = 0.70710678
        move_control.always_visible = True
        move_control.markers.append(cube)
        interactive_marker.controls.append(move_control)

        self.server.insert(interactive_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        self.center_x = feedback.pose.position.x
        self.center_y = feedback.pose.position.y
        self.points = self.make_points()
        rospy.loginfo(
            "Test obstacle moved to (%.3f, %.3f)", self.center_x, self.center_y
        )

    def make_points(self):
        points = []
        x = self.center_x - self.half_width_x
        while x <= self.center_x + self.half_width_x + 1e-6:
            y = self.center_y - self.half_width_y
            while y <= self.center_y + self.half_width_y + 1e-6:
                z = self.min_z
                while z <= self.max_z + 1e-6:
                    points.append((x, y, z))
                    z += 0.1
                y += 0.05
            x += 0.05
        return points

    def publish(self, _event):
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, self.points))


def main():
    rospy.init_node("test_obstacle_cloud_publisher")
    TestObstacleCloudPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
