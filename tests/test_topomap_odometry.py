"""Planar geometry and image-time odometry checks (no ROS master needed)."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/placenav"))
from topomap_odometry import OdomSample, interpolate_odometry, relative_pose


class RelativePoseTest(unittest.TestCase):
    def assert_pose(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=9)

    def test_forward_and_left_are_relative_to_departure_heading(self):
        # Robot faces north: moving north is forward, west is left.
        self.assert_pose(relative_pose((10, 20, math.pi/2),
                                       (9, 22, math.pi/2)), (2, 1, 0))

    def test_angle_wrap_uses_short_rotation(self):
        delta = relative_pose((0, 0, math.radians(179)),
                              (0, 0, math.radians(-179)))
        self.assertAlmostEqual(delta[2], math.radians(2))

    def test_backward_motion_keeps_signed_delta(self):
        self.assert_pose(relative_pose((2, 3, math.pi/2),
                                       (2, 1, math.pi/2)), (-2, 0, 0))

    def test_edges_compose_to_the_last_node(self):
        poses = [(0, 0, 0), (2, 0, math.pi/2), (2, 3, math.pi)]
        x, y, yaw = poses[0]
        for start, end in zip(poses, poses[1:]):
            dx, dy, dyaw = relative_pose(start, end)
            x, y = x + math.cos(yaw)*dx - math.sin(yaw)*dy, y + math.sin(yaw)*dx + math.cos(yaw)*dy
            yaw += dyaw
        self.assert_pose((x, y, yaw), poses[-1])

    def test_sampling_distance_is_chord_not_travelled_length(self):
        dx, dy, _ = relative_pose((5, 7, 1.2), (8, 11, 2.0))
        self.assertAlmostEqual(math.hypot(dx, dy), 5)


class InterpolationTest(unittest.TestCase):
    def setUp(self):
        self.before = OdomSample(1_000_000_000, (0, 0, math.radians(179)),
                                 "odom", "base_link")
        self.after = OdomSample(1_100_000_000, (4, 2, math.radians(-179)),
                                "odom", "base_link")

    def test_interpolates_position_and_wraps_heading(self):
        sample, before, after = interpolate_odometry(
            [self.before, self.after], 1_050_000_000, 200_000_000)
        for actual, expected in zip(sample.pose, (2, 1, math.pi)):
            self.assertAlmostEqual(actual, expected, places=9)
        self.assertEqual((before, after), (1_000_000_000, 1_100_000_000))
        self.assertEqual(sample.child_frame_id, "base_link")

    def test_exact_timestamp_does_not_require_extrapolation(self):
        sample, before, after = interpolate_odometry([self.before],
                                                     self.before.stamp_ns, 1)
        self.assertEqual(sample, self.before)
        self.assertEqual(before, after)

    def test_missing_bracket_and_large_gap_are_rejected(self):
        for stamp in (999_999_999, 1_100_000_001):
            self.assertIsNone(interpolate_odometry([self.before, self.after],
                                                   stamp, 200_000_000))
        self.assertIsNone(interpolate_odometry([self.before, self.after],
                                               1_050_000_000, 50_000_000))
        self.assertIsNone(interpolate_odometry([], 1, 200_000_000))

    def test_frame_change_cannot_be_interpolated(self):
        changed = OdomSample(self.after.stamp_ns, self.after.pose,
                             "vo_odom", "camera")
        self.assertIsNone(interpolate_odometry([self.before, changed],
                                               1_050_000_000, 200_000_000))


if __name__ == "__main__":
    unittest.main()
