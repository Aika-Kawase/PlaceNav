"""Metric prediction, image-time continuity and VPR integration checks."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import h5py
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from placenav.topomap_odometry import OdomSample
from placenav.place_recognition.odometry_motion import OdometryHistory, OdometryMotionModel
from placenav.place_recognition.bayesian_querier import PlaceRecognitionTopologicalFilter


def sample(t, x=0, y=0, yaw=0, frame='odom'):
    return OdomSample(round(t * 1e9), (x, y, yaw), frame, 'base_link')


class MotionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.make_map([0, 2, 4, 6])

    def tearDown(self):
        self.temp.cleanup()

    def make_map(self, positions):
        self.names = [str(i) + '.jpg' for i in range(len(positions))]
        self.path = self.root / 'node_odometry.json'
        self.path.write_text(json.dumps({
            'schema_version': 1, 'units': {'position': 'm', 'angle': 'rad'},
            'child_frame_id': 'base_link',
            'nodes': [{'id': i, 'image': name, 'pose': {'x': x, 'y': 0., 'yaw': 0.}}
                      for i, (name, x) in enumerate(zip(self.names, positions))]}))

    def model(self, prior):
        model = OdometryMotionModel(self.path, self.names, noise=0)
        model.initialize(np.array(prior, dtype=float))
        model.predict(sample(10))
        return model

    def mean(self, model):
        return float(model.grid_s @ model.probability)

    def test_small_moves_accumulate_across_node_changes(self):
        model = self.model([1, 0, 0, 0]);start = self.mean(model)
        for i in range(1, 21):
            model.predict(sample(10 + i * .1, x=i * .1))
            model.observe(np.ones(4))
            # Reading/changing the selected node does not reset motion state.
            np.argmax(model.node_belief())
        self.assertAlmostEqual(self.mean(model) - start, 2., places=8)
        self.assertEqual(int(np.argmax(model.node_belief())), 1)

    def test_stationary_robot_does_not_diffuse(self):
        model = self.model([.2, .5, .2, .1]);original = model.probability.copy()
        for i in range(1, 21):
            model.predict(sample(10 + i * .1))
        np.testing.assert_array_equal(model.probability, original)

    def test_nonuniform_node_spacing_uses_meters(self):
        self.make_map([0, .2, .4, 2.4, 4.4])
        model = self.model([0, 1, 0, 0, 0]);start = self.mean(model)
        for i in range(1, 21):
            model.predict(sample(10 + i * .1, x=i * .1))
        self.assertAlmostEqual(self.mean(model) - start, 2., places=8)
        self.assertEqual(int(np.argmax(model.node_belief())), 3)

    def test_backward_motion_is_retained(self):
        model = self.model([0, 1, 0, 0]);start = self.mean(model)
        model.predict(sample(11, x=-.5))
        self.assertAlmostEqual(self.mean(model) - start, -.5, places=8)

    def test_turn_in_place_then_sideways_does_not_count_as_forward(self):
        model = self.model([0, 1, 0, 0]);start = self.mean(model)
        model.predict(sample(11, yaw=math.pi/2))
        model.predict(sample(12, y=1, yaw=math.pi/2))
        self.assertAlmostEqual(self.mean(model), start, places=8)
        model.predict(sample(13, yaw=math.pi/2))
        model.predict(sample(14))
        model.predict(sample(15, x=.5))
        self.assertAlmostEqual(self.mean(model) - start, .5, places=8)

    def test_live_odom_origin_and_heading_do_not_need_to_match_map(self):
        a = self.model([.2, .5, .2, .1]);b = self.model([.2, .5, .2, .1])
        b.reset_odometry();angle = 1.2
        b.predict(sample(10, 100, -30, angle, 'vo_odom'))
        a.predict(sample(11, x=.4))
        b.predict(sample(11, 100 + .4*math.cos(angle), -30 + .4*math.sin(angle), angle, 'vo_odom'))
        np.testing.assert_allclose(a.node_belief(), b.node_belief(), atol=1e-12)

    def test_vpr_measurement_update_is_the_same_nodewise_bayes_rule(self):
        model = self.model([.1, .5, .3, .1]);prior = model.node_belief()
        likelihood = np.array([.5, .2, 1., .4])
        expected = prior * likelihood;expected /= expected.sum()
        np.testing.assert_allclose(model.observe(likelihood), expected, atol=1e-12)

    def test_missing_or_mismatched_map_fails_clearly(self):
        with self.assertRaises(FileNotFoundError):
            OdometryMotionModel(self.root/'missing.json', self.names)
        with self.assertRaises(ValueError):
            OdometryMotionModel(self.path, list(reversed(self.names)))

    def test_reanchor_preserves_location_and_skips_reset_displacement(self):
        model = self.model([.1, .5, .3, .1]);original = model.node_belief()
        model.reset_odometry();model.predict(sample(11, 100, 200, 2., 'vo_odom'))
        np.testing.assert_allclose(model.node_belief(), original)

    def test_short_gap_during_turn_keeps_heading_and_forward_motion(self):
        model = OdometryMotionModel(self.path, self.names, noise=0)
        model.initialize(np.array([0., 1., 0., 0.]))
        history = OdometryHistory(.2, max_speed=3)
        last_epoch = None
        start = self.mean(model)
        # Turn partly before a missing interval, then finish turning within it.
        for current in [sample(10), sample(10.1, yaw=math.pi/2),
                        sample(10.4, yaw=math.pi), sample(10.5),
                        sample(10.7, x=.5)]:
            history.add(current)
            if history.epoch != last_epoch:
                model.reset_odometry()
                last_epoch = history.epoch
            model.predict(history.at(current.stamp_ns))
        self.assertEqual(history.epoch, 0)
        self.assertIsNone(history.at(10_250_000_000))
        self.assertAlmostEqual(self.mean(model) - start, .5, places=8)
        np.testing.assert_allclose(model.heading_offset[model.probability > 0], 0, atol=1e-12)

    def test_short_gap_retains_translation_between_endpoints(self):
        model = self.model([0, 1, 0, 0]);start = self.mean(model)
        history = OdometryHistory(.2, max_speed=3)
        last_epoch = None
        for current in [sample(10), sample(10.3, x=.3), sample(10.4, x=.4)]:
            history.add(current)
            if history.epoch != last_epoch:
                model.reset_odometry()
                last_epoch = history.epoch
            model.predict(history.at(current.stamp_ns))
        self.assertAlmostEqual(self.mean(model) - start, .4, places=8)

    def test_duplicate_image_is_not_assimilated_twice(self):
        descriptors = np.eye(4, dtype=np.float32)
        path = self.root/'features.h5'
        with h5py.File(path, 'w') as db:
            for name, desc in zip(self.names, descriptors):
                db.create_group(name).create_dataset('global_descriptor', data=desc)
        calls = []
        def extract(index):
            calls.append(index)
            return {'global_descriptor': torch.tensor(descriptors[index:index+1])}
        filt = PlaceRecognitionTopologicalFilter(extract, path, self.root,
                                                 motion_model='odometry')
        filt.initialize_model(0);filt.match(0, sample(10))
        original = filt.belief.copy();before = len(calls)
        filt.match(0, sample(10))
        np.testing.assert_array_equal(filt.belief, original)
        self.assertEqual(len(calls), before)
        filt.db_handler.gallery_db.close()


class HistoryTest(unittest.TestCase):
    def test_future_odometry_is_required_and_interpolated(self):
        history = OdometryHistory(.2);history.add(sample(10))
        self.assertIsNone(history.at(10_050_000_000))
        history.add(sample(10.1, x=2))
        self.assertAlmostEqual(history.at(10_050_000_000).pose[0], 1.)

    def test_gap_frame_change_and_clock_reset_start_new_epoch(self):
        history = OdometryHistory(.2);history.add(sample(10))
        self.assertEqual(history.add(sample(11.1)), 'odometry input gap')
        self.assertIsNone(history.at(10_500_000_000))
        self.assertEqual(history.add(sample(11.2, frame='new_odom')), 'coordinate frames changed')
        self.assertEqual(history.add(sample(9)), 'timestamp moved backwards')
        self.assertEqual(history.epoch, 3)

    def test_short_gap_keeps_endpoints_without_interpolating_missing_interval(self):
        history = OdometryHistory(.2, max_speed=3)
        history.add(sample(10))
        self.assertIsNone(history.add(sample(10.3, x=.3, yaw=.4)))
        self.assertEqual(history.epoch, 0)
        self.assertIsNone(history.at(10_150_000_000))
        history.add(sample(10.4, x=.4, yaw=.5))
        self.assertAlmostEqual(history.at(10_350_000_000).pose[0], .35)

    def test_short_gap_boundary_and_configured_interpolation_limit(self):
        history = OdometryHistory(.2)
        history.add(sample(10))
        self.assertIsNone(history.add(sample(11)))
        self.assertEqual(history.add(sample(12.000000001)), 'odometry input gap')
        configured = OdometryHistory(2.)
        configured.add(sample(10))
        self.assertIsNone(configured.add(sample(11.5)))
        self.assertIsNotNone(configured.at(11_000_000_000))

    def test_short_gap_does_not_hide_pose_or_frame_resets(self):
        history = OdometryHistory(.2, max_speed=3)
        history.add(sample(10))
        self.assertEqual(history.add(sample(10.3, x=100)), 'odometry pose jump')
        self.assertEqual(history.add(sample(10.6, x=100, frame='new_odom')),
                         'coordinate frames changed')
        self.assertEqual(history.add(sample(10.5, x=100, frame='new_odom')),
                         'timestamp moved backwards')
        self.assertEqual(history.epoch, 3)

    def test_same_frame_position_reset_is_not_integrated_as_motion(self):
        history = OdometryHistory(.2, max_speed=3)
        history.add(sample(10, x=100))
        self.assertEqual(history.add(sample(10.02)), 'odometry pose jump')
        self.assertEqual(history.epoch, 1)
        self.assertIsNone(history.at(10_010_000_000))


if __name__ == '__main__':
    unittest.main()
