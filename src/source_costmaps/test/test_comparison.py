#!/usr/bin/env python3
"""Real gates, consumed-once batching, and both STVL topologies."""
import json
import threading
import time
import unittest

import numpy as np
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix


class ComparisonTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.now = 1000.
        self.global_frame = rospy.get_param('~global_frame', 'odom')
        self.base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        self.cloud_frames = set()
        self.maps, self.messages, self.traces, self.diag = {}, {}, [], {}
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.tf = rospy.Publisher('/tf', TFMessage, queue_size=5)
        self.static = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        self.inputs = {s: rospy.Publisher(t, PointCloud2, queue_size=1) for s, t in (
            ('d435', '/realsense/depth/color/points'), ('unik3d', '/c2/depth/points'))}
        self.inject = rospy.Publisher('/obstacles/d435/points', PointCloud2, queue_size=10)
        self.subs = []
        for mode in ('points', 'layers'):
            self.subs.append(rospy.Subscriber('/obstacles/'+mode+'_map/combined/costmap', OccupancyGrid,
                                             lambda m, s=mode: self.save_map(s, m)))
        for source in ('d435', 'unik3d', 'fused'):
            self.subs.append(rospy.Subscriber('/obstacles/comparison/'+source+'/points', PointCloud2,
                                             lambda m, s=source: self.save_cloud(s, m), queue_size=100))
        self.subs.append(rospy.Subscriber('/obstacles/comparison/batch_trace', String, self.save_trace, queue_size=100))
        self.subs.append(rospy.Subscriber('/obstacles/comparison/diagnostics', DiagnosticArray, self.save_diag))
        self.running = True
        def pump():
            while self.running and not rospy.is_shutdown():
                with self.lock:
                    self.now += .02
                    now = self.now
                self.clock.publish(Clock(rospy.Time.from_sec(now)))
                self.tf.publish(TFMessage([self.transform(self.global_frame, self.base_frame, [0, 0, 0], now)]))
                time.sleep(.02)
        self.thread = threading.Thread(target=pump, daemon=True)
        self.thread.start()
        self.static.publish(TFMessage([
            self.transform(self.base_frame, 'realsense_color_optical_frame', [0, 0, 1]),
            self.transform(self.base_frame, 'c2_camera_optical_frame', [.02652, 0, 1.0175])]))
        self.rotation = quaternion_matrix([-.4813627040827183, .4865498075348136, -.5130442130488477, .5180201418507755])[:3, :3]
        self.origin = np.array([.02652, 0, 1.4686620009085125])
        self.wait(lambda: all(p.get_num_connections() for p in self.inputs.values()), 15)
        time.sleep(.5)

    def tearDown(self):
        self.running = False
        self.thread.join(2)
        for sub in self.subs:
            sub.unregister()

    @staticmethod
    def transform(parent, child, xyz, now=0):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp = rospy.Time.from_sec(now)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.w = 1
        return t

    def save_map(self, mode, msg):
        with self.lock:
            self.maps[mode] = msg

    def save_cloud(self, source, msg):
        with self.lock:
            self.cloud_frames.add(msg.header.frame_id)
            key = msg.header.stamp.to_nsec()
            self.messages.setdefault(key, {})[source] = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z')))

    def save_trace(self, msg):
        with self.lock:
            self.traces.append(json.loads(msg.data))

    def save_diag(self, msg):
        with self.lock:
            self.diag = {s.hardware_id: dict((v.key, v.value) for v in s.values) for s in msg.status}

    def wait(self, check, seconds=5):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            with self.lock:
                if check():
                    return
            time.sleep(.025)
        self.fail('Timed out waiting for condition')

    def send(self, source, xyz):
        with self.lock:
            stamp = rospy.Time.from_sec(self.now-.04)
        if source == 'unik3d':
            points = [(np.asarray(p)-self.origin)@self.rotation for p in xyz for _ in range(3)]
            frame = 'c2_camera_optical_frame'
        else:
            points = [np.asarray(p)-[0, 0, 1] for p in xyz]
            frame = 'realsense_color_optical_frame'
        self.inputs[source].publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points))

    def occupied(self, mode, xyz):
        m = self.maps.get(mode)
        if m is None:
            return False
        ix = int(np.floor((xyz[0]-m.info.origin.position.x)/m.info.resolution))
        iy = int(np.floor((xyz[1]-m.info.origin.position.y)/m.info.resolution))
        return any(0 <= ix+dx < m.info.width and 0 <= iy+dy < m.info.height and
                   m.data[(iy+dy)*m.info.width+ix+dx] == 100 for dx in (-1, 0, 1) for dy in (-1, 0, 1))

    def test_same_batches_no_reuse_missing_source_expiry_and_bad_input(self):
        d, u = [2.225, -.775, .225], [3.225, 1.225, .725]
        self.send('d435', [d, [1.225, -.775, .025]])
        self.wait(lambda: all(self.occupied(m, d) for m in ('points', 'layers')))
        with self.lock:
            n = len(self.traces)
        time.sleep(.3)
        self.assertEqual(n, len(self.traces))  # No cached-point refresh while idle.
        for _ in range(8):
            self.send('d435', [d])
            self.send('unik3d', [u])
            time.sleep(.06)
        self.wait(lambda: all(self.occupied(m, d) and self.occupied(m, u) for m in ('points', 'layers')))
        self.wait(lambda: any(t['sources']['d435']['points'] and t['sources']['unik3d']['points'] for t in self.traces))
        # Stop UniK3D; new D435 observations must not refresh its old points.
        deadline = time.monotonic()+1.2
        while time.monotonic() < deadline:
            self.send('d435', [d])
            time.sleep(.06)
        self.wait(lambda: all(not self.occupied(m, u) and self.occupied(m, d) for m in ('points', 'layers')))
        self.wait(lambda: all(not self.occupied(m, d) for m in ('points', 'layers')), 3)
        for m in self.maps.values():
            self.assertIn(-1, m.data)
        # Exercise batch validation independently of the already validated gate.
        with self.lock:
            stamp = rospy.Time.from_sec(self.now-.01)
        valid = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=self.global_frame), [d])
        self.inject.publish(valid)
        time.sleep(.04)
        self.inject.publish(valid)  # Same source acquisition must be consumed once.
        wrong = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='wrong'), [d])
        self.inject.publish(wrong)
        stale = point_cloud2.create_cloud_xyz32(Header(stamp=stamp-rospy.Duration(2), frame_id=self.global_frame), [d])
        self.inject.publish(stale)
        self.wait(lambda: 'd435' in self.diag and all(int(self.diag['d435'][k]) > 0 for k in ('order', 'invalid', 'stale')))
        time.sleep(.2)
        seen = {s: set() for s in ('d435', 'unik3d')}
        with self.lock:
            for trace in self.traces:
                parts = self.messages[trace['header_stamp_ns']]
                expected = []
                for s in ('d435', 'unik3d'):
                    for stamp_ns in trace['sources'][s]['stamps_ns']:
                        self.assertNotIn(stamp_ns, seen[s])
                        seen[s].add(stamp_ns)
                    expected += parts.get(s, [])
                self.assertEqual(expected, parts['fused'])
                self.assertEqual(len(expected), trace['points'])
        self.assertEqual([p['name'] for p in rospy.get_param('/obstacles/points_map/local_costmap/plugins')], ['fused', 'inflation'])
        self.assertEqual([p['name'] for p in rospy.get_param('/obstacles/layers_map/local_costmap/plugins')], ['d435', 'unik3d', 'inflation'])
        self.assertEqual(self.cloud_frames, {self.global_frame})
        self.assertTrue(all(m.header.frame_id == self.global_frame for m in self.maps.values()))


if __name__ == '__main__':
    rospy.init_node('test_fusion_comparison')
    rostest.rosrun('source_costmaps', rospy.get_name().strip('/'), ComparisonTest)
