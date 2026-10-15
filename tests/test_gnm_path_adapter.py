#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from placenav.gnm_path_adapter import waypoint_yaws, local_poses  # noqa: E402
from placenav.utils import waypoints_to_ros_msg
import numpy as np
import rospy


class WaypointYawsTest(unittest.TestCase):
    def test_extended_guidance_poses(self):
        messages = waypoints_to_ros_msg(np.array([[.1, 0], [.55, 0]]))
        poses = local_poses(messages, rospy.Time(12), "base_link", True, 2.0)
        self.assertAlmostEqual(poses[2].pose.position.x, .55)
        self.assertAlmostEqual(poses[-1].pose.position.x, 2.0)
        self.assertEqual(poses[-1].header.frame_id, "base_link")
        self.assertEqual(poses[-1].header.stamp, rospy.Time(12))
        self.assertAlmostEqual(poses[-1].pose.orientation.w, 1.0)

    def test_nomad_xy_and_vint_xy_heading_to_path(self):
        for count, width in ((8, 2), (5, 4)):
            points = np.zeros((count, width))
            points[:, 0] = np.arange(1, count + 1) * 0.1
            messages = waypoints_to_ros_msg(points)
            poses = local_poses(messages, rospy.Time(12), "base_link", True)
            self.assertEqual(len(poses), count + 1)
            self.assertAlmostEqual(poses[-1].pose.position.x, count * 0.1)

    def test_empty_path(self):
        self.assertEqual(waypoint_yaws([]), [])

    def test_single_point_faces_forward(self):
        self.assertEqual(waypoint_yaws([(1.0, 0.0)]), [0.0])

    def test_straight_path_faces_forward(self):
        self.assertEqual(waypoint_yaws([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]), [0.0, 0.0, 0.0])

    def test_curve_uses_each_segment_and_repeats_last_heading(self):
        result = waypoint_yaws([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
        self.assertAlmostEqual(result[0], 0.0)
        self.assertAlmostEqual(result[1], math.pi / 2.0)
        self.assertAlmostEqual(result[2], math.pi / 2.0)


if __name__ == "__main__":
    unittest.main()
