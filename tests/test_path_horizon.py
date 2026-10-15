import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.path_horizon import extend_horizon


class PathHorizonTest(unittest.TestCase):
    def test_short_straight(self):
        original = [(0, 0), (.1, 0), (.55, 0)]
        result = extend_horizon(original, 2)
        self.assertEqual(result[:3], original)
        self.assertAlmostEqual(result[-1][0], 2)
        self.assertLessEqual(max(math.dist(a, b) for a, b in zip(result[2:], result[3:])), .100001)

    def test_curve_preserved(self):
        original = [(0, 0), (.2, 0), (.5, .2)]
        result = extend_horizon(original, 2)
        self.assertEqual(result[:3], original)
        self.assertAlmostEqual((result[-1][1]-.2)/(result[-1][0]-.5), 2/3)

    def test_disabled_and_long(self):
        for points, target in [([(0, 0), (.55, 0)], 0), ([(0, 0), (3, 0)], 2)]:
            self.assertEqual(extend_horizon(points, target), points)

    def test_stationary_reverse_lateral(self):
        for points in [[(0, 0), (.05, 0)], [(0, 0), (-.5, 0)],
                       [(0, 0), (.5, 0), (.5, .4)], [(0, 0), (0, 0)]]:
            self.assertEqual(extend_horizon(points, 2), points)

    def test_extension_bounded(self):
        self.assertAlmostEqual(extend_horizon([(0, 0), (.2, 0)], 3)[-1][0], 1.7)

    def test_invalid(self):
        for points, target in [([(float('nan'), 0)], 2), ([(0, 0)], -1), ([], float('inf'))]:
            with self.assertRaises(ValueError):
                extend_horizon(points, target)


if __name__ == '__main__':
    unittest.main()
