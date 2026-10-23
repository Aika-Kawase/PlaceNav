"""ROS message/TF integration tests without a running robot or ROS master."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import threading
import time
import json
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from placenav.nomad_selection_ros import CandidateSelector
from placenav.gnm_path_adapter import GnmPathAdapter
from placenav_viz_msgs.msg import Viz


class RosSelectionTests(unittest.TestCase):
    def setUp(self):
        self.selector = CandidateSelector.__new__(CandidateSelector)
        s = self.selector
        s.args = SimpleNamespace(nomad_base_frame="base_link", nomad_input_timeout=1.,
                                 nomad_clearance=0., nomad_continuity_weight=1., nomad_seed=0)
        s.lock = threading.Lock()
        s.map = OccupancyGrid()
        s.map.header.frame_id = "odom"
        s.map.header.stamp = rospy.Time.from_sec(10.)
        s.map.info.resolution = .05
        s.map.info.width = s.map.info.height = 120
        s.map.info.origin.position.x = s.map.info.origin.position.y = -3.
        s.map.info.origin.orientation.w = 1.
        s.map.data = [0] * 14400
        s.map_wall = time.monotonic()
        s.previous = s.previous_stamp = s.previous_subgoal = s.previous_frame = None
        tr = TransformStamped()
        tr.transform.rotation.w = 1.
        s.tf = Mock()
        s.tf.lookup_transform.return_value = tr
        s.markers, s.diagnostics = Mock(), Mock()
        self.paths = np.array([[[.3, 0], [.6, 0], [.9, 0]],
                               [[.2, .2], [.5, .4], [.8, .5]]])
        self.clock = patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(10.1))
        self.clock.start()
        self.params = patch("rospy.get_param", side_effect=lambda key, *default:
                            0. if key.endswith("padding") else
                            [[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
        self.params.start()
        self.warn = patch("rospy.logwarn_throttle")
        self.warn.start()

    def tearDown(self):
        self.clock.stop()
        self.params.stop()
        self.warn.stop()

    def test_selected_diagnostics_markers_and_timestamp(self):
        result = self.selector.select(self.paths, rospy.Time(10), 4, .1)
        np.testing.assert_array_equal(result, self.paths[0])
        data = json.loads(self.selector.diagnostics.publish.call_args[0][0].data)
        self.assertEqual(data["selected_index"], 0)
        self.assertEqual(data["subgoal_idx"], 4)
        markers = self.selector.markers.publish.call_args[0][0].markers
        self.assertEqual(len(markers), 3)
        self.assertEqual(markers[1].header.frame_id, "odom")
        self.assertEqual(markers[1].header.stamp, rospy.Time(10))

    def test_missing_stale_map_and_tf_fail_closed(self):
        self.selector.map.header.stamp = rospy.Time(8)
        self.assertIsNone(self.selector.select(self.paths, rospy.Time(10), 4, .1))
        self.selector.map.header.stamp = rospy.Time(10)
        import tf2_ros
        self.selector.tf.lookup_transform.side_effect = tf2_ros.LookupException("missing")
        self.assertIsNone(self.selector.select(self.paths, rospy.Time(10), 4, .1))
        self.selector.map = None
        self.assertIsNone(self.selector.select(self.paths, rospy.Time(10), 4, .1))

    def test_stale_query_all_blocked_and_time_reset(self):
        self.assertIsNone(self.selector.select(self.paths, rospy.Time(8), 4, .1))
        self.selector.map.data = [100] * 14400
        self.assertIsNone(self.selector.select(self.paths, rospy.Time(10), 4, .1))
        self.selector.map.data = [0] * 14400
        self.selector.previous_stamp = rospy.Time(11)
        self.selector.previous_frame = "odom"
        self.selector.previous = self.paths[1]
        self.assertIsNotNone(self.selector.select(self.paths, rospy.Time(10), 4, .1))

    def test_baseline_not_filtered_even_without_map(self):
        self.selector.map = None
        self.selector.observe_single(self.paths[0], rospy.Time(10), 4, .1)
        data = json.loads(self.selector.diagnostics.publish.call_args[0][0].data)
        self.assertEqual(data["reason"], "BASELINE_UNFILTERED")

    def test_partial_costmap_updates_are_used(self):
        from map_msgs.msg import OccupancyGridUpdate
        update = OccupancyGridUpdate()
        update.header.frame_id = "odom"
        update.header.stamp = rospy.Time.from_sec(10.05)
        update.x, update.y = 72, 60
        update.width = update.height = 1
        update.data = [100]
        self.selector.on_update(update)
        result = self.selector.select(self.paths, rospy.Time(10), 4, .1)
        np.testing.assert_array_equal(result, self.paths[1])

    def test_empty_viz_invalidates_path_without_tf(self):
        adapter = GnmPathAdapter.__new__(GnmPathAdapter)
        adapter.path_publisher = Mock()
        adapter.target_frame = "odom"
        msg = Viz()
        msg.query_timestamp = rospy.Time(10)
        adapter.viz_callback(msg)
        path = adapter.path_publisher.publish.call_args[0][0]
        self.assertEqual(path.poses, [])
        self.assertEqual(path.header.stamp, rospy.Time(10))


if __name__ == "__main__":
    unittest.main()
