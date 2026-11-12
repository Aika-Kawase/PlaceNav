"""Goal decisions use the existing posterior without changing localization."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from placenav.place_recognition.odometry_motion import OdometryMotionModel
from placenav.subgoal_selector import ProgressSubgoalSelector


class SelectorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        path = Path(self.temp.name)/'nodes.json'
        names = [str(i)+'.jpg' for i in range(5)]
        path.write_text(json.dumps(dict(
            schema_version=1, units=dict(position='m', angle='rad'), child_frame_id='base_link',
            nodes=[dict(id=i, image=name, pose=dict(x=3.5*i, y=0, yaw=0))
                   for i, name in enumerate(names)])))
        self.motion = OdometryMotionModel(path, names, noise=0)
        self.motion.initialize(np.array([1., 0, 0, 0, 0]))
        self.selector = ProgressSubgoalSelector(self.motion, 4, mode='passage')

    def tearDown(self):
        self.temp.cleanup()

    def mass(self, *positions):
        self.motion.probability.fill(0)
        for position, weight in positions:
            i = np.argmin(np.abs(self.motion.grid_s-position))
            self.motion.probability[i] += weight

    def test_immediate_initial_goal_even_with_ambiguous_global_belief(self):
        self.mass((0, .5), (14, .5))
        self.assertEqual(self.selector.update(0, 1), (1, False))

    def test_midpoint_and_closest_node_changes_do_not_advance_goal(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((1.8, 1))
        for stamp, closest in enumerate([1, 0, 1, 1], 2):
            self.assertEqual(self.selector.update(closest, stamp), (1, False))

    def test_passage_requires_two_fresh_images_and_resets_after_failure(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((3.6, 1))
        self.assertEqual(self.selector.update(1, 2), (1, False))
        self.assertEqual(self.selector.update(1, 2), (1, False))
        self.mass((3.4, 1));self.selector.update(1, 3)
        self.mass((3.6, 1))
        self.assertEqual(self.selector.update(1, 4), (1, False))
        self.assertEqual(self.selector.update(1, 5), (2, False))
        self.assertEqual(self.selector.update(0, 6), (1, False))
        self.assertEqual(self.selector.status, 'RECOVERED')

    def test_goal_left_behind_recovers_even_without_passage_confirmation(self):
        self.mass((0, 1));self.selector.update(0, 1)
        # Node 2 is the mode, although less than half the mass has passed goal 1.
        self.mass((0, .3), (3.4, .3), (7, .4))
        self.assertEqual(self.selector.update(2, 2), (3, False))
        self.assertEqual(self.selector.status, 'RECOVERED')
        self.assertEqual(self.selector.count, 0)

    def test_goal_ahead_recovers_to_current_node_plus_one(self):
        self.mass((7, 1));self.selector.update(2, 1)
        self.mass((0, 1))
        self.assertEqual(self.selector.update(0, 2), (1, False))
        self.assertEqual(self.selector.status, 'RECOVERED')

    def test_advanced_candidate_is_checked_before_output(self):
        # The mode stays at node 0 while the global tail probability exceeds .5.
        self.mass((0, .4), (3.6, .2), (7, .2), (10.5, .2))
        self.selector.update(0, 1)
        for stamp in range(2, 10):
            self.assertEqual(self.selector.update(0, stamp), (1, False))
        self.assertEqual(self.selector.status, 'RECOVERED')

    def test_intermediate_passage_does_not_require_old_local_mass_gate(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((3.6, .55), (14, .45))
        self.assertEqual(self.selector.update(1, 2), (1, False))
        self.assertLess(self.selector.probabilities['local'], .8)
        self.assertEqual(self.selector.update(1, 3), (2, False))
        self.assertEqual(self.selector.status, 'ADVANCED')

    def test_recovery_clears_confirmation_and_cannot_declare_final_arrival(self):
        self.mass((0, 1));self.selector.update(0, 1)
        self.mass((3.6, 1));self.selector.update(1, 2)
        self.assertEqual(self.selector.count, 1)
        self.mass((14, 1))
        self.assertEqual(self.selector.update(4, 3), (4, False))
        self.assertEqual(self.selector.status, 'RECOVERED')
        self.assertEqual(self.selector.count, 0)
        self.assertEqual(self.selector.update(4, 3), (4, False))
        self.assertEqual(self.selector.update(4, 4), (4, False))
        self.assertEqual(self.selector.update(4, 5), (4, True))

    def test_final_goal_can_recover_back_before_arrival(self):
        self.mass((10.5, 1));self.selector.update(3, 1)
        self.mass((3.5, 1))
        self.assertEqual(self.selector.update(1, 2), (2, False))
        self.assertEqual(self.selector.status, 'RECOVERED')

    def test_recovery_respects_configured_final_goal(self):
        selector = ProgressSubgoalSelector(self.motion, 2, mode='passage')
        self.mass((0, 1));selector.update(0, 1)
        self.mass((14, 1))
        for stamp in range(2, 6):
            self.assertEqual(selector.update(4, stamp), (2, False))

    def test_confirmation_reset_and_clock_rewind_keep_goal(self):
        self.mass((0, 1));self.selector.update(0, 10)
        self.mass((3.6, 1));self.selector.update(1, 11)
        self.selector.reset_confirmation()
        self.assertEqual(self.selector.update(1, 12), (1, False))
        self.assertEqual(self.selector.update(1, 5), (1, False))
        self.assertEqual(self.selector.update(1, 6), (2, False))

    def test_nearest_mode_keeps_original_image_selection(self):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='nearest')
        self.mass((0, 1))
        for stamp, closest in enumerate([0, 1, 0, 2], 1):
            self.assertEqual(selector.update(closest, stamp), (closest+1, False))

    def test_passage_final_neighborhood_and_terminal_latch(self):
        selector = ProgressSubgoalSelector(self.motion, 4, mode='passage')
        self.mass((12.3, 1))  # End node may be nearest well before arrival.
        for stamp in range(1, 4):
            self.assertEqual(selector.update(4, stamp), (4, False))
        self.mass((13.8, 1))
        self.assertEqual(selector.update(4, 4), (4, False))
        self.assertEqual(selector.update(4, 5), (4, True))
        self.mass((0, 1))
        self.assertEqual(selector.update(0, 6), (4, True))

    def test_nearest_final_decision_is_unchanged(self):
        selector = ProgressSubgoalSelector(self.motion, 4)
        self.mass((12.3, 1))
        self.assertEqual(selector.update(3, 1), (4, False))
        self.assertEqual(selector.update(4, 2), (4, True))

    def test_final_local_mass_gate_is_preserved(self):
        self.mass((14, .6), (0, .4))
        for stamp in range(1, 5):
            self.assertEqual(self.selector.update(4, stamp), (4, False))
            self.assertEqual(self.selector.status, 'REMOTE_BELIEF')

    def test_intermediate_final_goal_does_not_accept_farther_positions(self):
        selector = ProgressSubgoalSelector(self.motion, 2, mode='passage')
        self.mass((9, 1))
        for stamp in range(1, 4):
            self.assertEqual(selector.update(2, stamp), (2, False))
        self.mass((6.8, 1));selector.update(2, 4)
        self.assertEqual(selector.update(2, 5), (2, True))

    def test_read_only_probabilities_use_global_mass(self):
        self.mass((3.6, .2), (14, .8))
        original = self.motion.probability.copy();owners = self.motion.owner.copy()
        info = self.motion.progress_probabilities(1)
        self.assertAlmostEqual(info['passed'], 1)
        self.assertAlmostEqual(info['local'], .2)
        self.selector.update(0, 1);self.selector.update(1, 2)
        np.testing.assert_array_equal(self.motion.probability, original)
        np.testing.assert_array_equal(self.motion.owner, owners)

    def test_invalid_configuration_is_rejected(self):
        for options in [dict(mode='typo'), dict(mode='passage', lookahead=2),
                        dict(threshold=0), dict(threshold=float('nan')),
                        dict(min_local_mass=1.1), dict(confirmations=0),
                        dict(final_tolerance=-1)]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                ProgressSubgoalSelector(self.motion, 4, **options)


if __name__ == '__main__':
    unittest.main()
