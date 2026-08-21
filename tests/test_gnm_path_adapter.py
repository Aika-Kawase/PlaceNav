#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from placenav.gnm_path_adapter import waypoint_yaws  # noqa: E402


class WaypointYawsTest(unittest.TestCase):
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
