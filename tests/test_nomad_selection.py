import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.nomad_selection import evaluate, transform_xy


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.grid = np.zeros((120, 120), dtype=np.int16)
        self.footprint = [[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1]]
        self.paths = np.array([[[.3, 0], [.6, 0], [.9, 0]],
                               [[.2, .2], [.5, .4], [.8, .5]]])

    def select(self, paths=None, **kwargs):
        return evaluate(self.paths if paths is None else paths, self.grid, .05,
                        [-3, -3, 0], [0, 0, 0], self.footprint,
                        clearance=0., **kwargs)

    def test_free_space_prefers_straight(self):
        index, records = self.select()
        self.assertEqual(index, 0)
        self.assertTrue(all(r["valid"] for r in records))

    def test_collision_selects_other(self):
        self.grid[60, 72] = 100
        index, records = self.select()
        self.assertEqual(index, 1)
        self.assertFalse(records[0]["valid"])

    def test_unknown_and_all_invalid(self):
        self.grid[:] = -1
        self.assertEqual(self.select()[0], -1)

    def test_segment_collision_not_only_waypoints(self):
        self.grid[60, 69] = 100
        self.assertFalse(self.select()[1][0]["valid"])

    def test_footprint_interior_collision(self):
        self.footprint = [[-.2, -.2], [.8, -.2], [.8, .2], [-.2, .2]]
        self.grid[60, 68] = 100
        self.assertEqual(self.select()[0], -1)

    def test_out_of_bounds_and_nonfinite(self):
        bad = self.paths.copy()
        bad[0] += 5
        bad[1, 1, 0] = np.nan
        self.assertEqual(self.select(bad)[0], -1)

    def test_continuity_in_fixed_frame(self):
        self.assertEqual(self.select(previous=self.paths[1], continuity_weight=10)[0], 1)

    def test_rotated_translated_grid_same_result(self):
        args = (self.paths, self.grid, .05, [-3, -3, 0], [0, 0, 0], self.footprint)
        a = evaluate(*args, clearance=0.)
        pose = [10, -7, .6]
        origin = [*transform_xy([[-3, -3]], pose)[0], .6]
        b = evaluate(self.paths, self.grid, .05, origin, pose, self.footprint, clearance=0.)
        self.assertEqual(a[0], b[0])
        self.assertAlmostEqual(a[1][0]["score"], b[1][0]["score"])

    def test_invalid_configuration(self):
        with self.assertRaises(ValueError):
            self.select(continuity_weight=float("nan"))


if __name__ == "__main__":
    unittest.main()
