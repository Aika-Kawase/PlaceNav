#!/usr/bin/env python3
"""Exercise real gates + two STVL plugins, using acquisition-time transforms."""
import math
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
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix


class LayersTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.maps, self.clouds, self.diag = {}, {}, {}
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.tf = rospy.Publisher('/tf', TFMessage, queue_size=5)
        self.static = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        self.inputs = {s: rospy.Publisher(t, PointCloud2, queue_size=1) for s, t in
                       [('d435', '/realsense/depth/color/points'), ('unik3d', '/c2/depth/points')]}
        self.subs = []
        for s in ('d435', 'unik3d', 'combined'):
            self.subs.append(rospy.Subscriber('/obstacles/source_costmaps/'+s+'/costmap', OccupancyGrid,
                                             lambda m, s=s: self.save(self.maps, s, m)))
        for s in ('d435', 'unik3d'):
            self.subs.append(rospy.Subscriber('/obstacles/'+s+'/points', PointCloud2,
                                             lambda m, s=s: self.save(self.clouds, s, m)))
            self.subs.append(rospy.Subscriber('/obstacles/'+s+'_gate/diagnostics', DiagnosticArray,
                                             lambda m, s=s: self.save(self.diag, s, m)))
        self.now = 1000.0
        self.robot_x = 0.0
        self.running = True
        def pump():
            while self.running and not rospy.is_shutdown():
                with self.lock:
                    self.now += .02; now = self.now; x = self.robot_x
                self.clock.publish(Clock(rospy.Time.from_sec(now)))
                t = self.transform('odom', 'base_link', [x, 0, 0], stamp=now)
                self.tf.publish(TFMessage([t]))
                time.sleep(.02)
        self.thread = threading.Thread(target=pump, daemon=True); self.thread.start()
        self.static.publish(TFMessage([
            self.transform('base_link', 'realsense_color_optical_frame', [0, 0, 1]),
            self.transform('base_link', 'c2_camera_optical_frame', [.02652, 0, 1.0175])]))
        self.q = [-.4813627040827183, .4865498075348136, -.5130442130488477, .5180201418507755]
        self.r = quaternion_matrix(self.q)[:3, :3]
        self.origin = np.array([.02652, 0, 1.4686620009085125])
        self.wait(lambda: all(p.get_num_connections() for p in self.inputs.values()), 15)
        time.sleep(.5)

    def tearDown(self):
        self.running = False; self.thread.join(2)
        for s in self.subs: s.unregister()

    def save(self, collection, key, msg):
        with self.lock: collection[key] = msg

    @staticmethod
    def transform(parent, child, translation, stamp=0):
        t = TransformStamped(); t.header.frame_id = parent; t.child_frame_id = child
        t.header.stamp = rospy.Time.from_sec(stamp)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
        t.transform.rotation.w = 1
        return t

    def wait(self, predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate(): return
            time.sleep(.025)
        self.fail('Timed out waiting for condition')

    def send(self, source, points, stamp=None, frame=None):
        with self.lock: now = self.now
        stamp = now-.04 if stamp is None else stamp
        if source == 'unik3d':
            points = [(np.array(p)-self.origin)@self.r for p in points]
            # Several points support a voxel, without changing its centroid.
            points = [p for p in points for _ in range(3)]
            frame = frame or 'c2_camera_optical_frame'
        else:
            points = [np.array(p)-np.array([0, 0, 1]) for p in points]
            frame = frame or 'realsense_color_optical_frame'
        msg = point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.from_sec(stamp), frame_id=frame), points)
        self.inputs[source].publish(msg)
        return stamp

    def occupied(self, source, x, y):
        with self.lock: m = self.maps.get(source)
        if m is None: return False
        ix = int(math.floor((x-m.info.origin.position.x)/m.info.resolution))
        iy = int(math.floor((y-m.info.origin.position.y)/m.info.resolution))
        # Voxel flattening can place a centroid in an adjacent map cell.
        return any(0 <= ix+dx < m.info.width and 0 <= iy+dy < m.info.height and
                   m.data[(iy+dy)*m.info.width+ix+dx] == 100 for dx in (-1, 0, 1) for dy in (-1, 0, 1))

    def diagnostic_count(self, source, key):
        with self.lock: d = self.diag.get(source)
        return int(dict((v.key, v.value) for v in d.status[0].values).get(key, 0)) if d else 0

    def test_separation_correction_expiry_and_freshness(self):
        # Low obstacle is accepted by D435, floor below 10 cm is rejected.
        d = [2.225, -.775, .225]
        u = [3.225, 1.225, .725]
        d_stamp = self.send('d435', [d, [1.225, -.775, .025]])
        self.wait(lambda: 'd435' in self.clouds)
        self.assertAlmostEqual(self.clouds['d435'].header.stamp.to_sec(), d_stamp, places=6)
        self.assertEqual(self.clouds['d435'].width, 1)
        self.wait(lambda: self.occupied('d435', *d[:2]))
        self.assertFalse(self.occupied('unik3d', *d[:2]))
        self.send('unik3d', [u]); time.sleep(.08)
        self.wait(lambda: 'unik3d' in self.clouds)
        self.assertEqual(self.clouds['unik3d'].width, 0)  # first frame cannot confirm
        u_stamp = self.send('unik3d', [u]); time.sleep(.08)
        self.wait(lambda: self.clouds['unik3d'].width > 0)
        got = list(point_cloud2.read_points(self.clouds['unik3d'], field_names=('x', 'y', 'z')))[0]
        np.testing.assert_allclose(got, u, atol=2e-5)  # exact corrected mounting, no extra scale
        self.assertAlmostEqual(self.clouds['unik3d'].header.stamp.to_sec(), u_stamp, places=6)
        self.wait(lambda: self.occupied('unik3d', *u[:2]) and self.occupied('combined', *u[:2]))
        self.assertFalse(self.occupied('d435', *u[:2]))
        # Continue D435 while UniK3D disappears; the inferred layer cannot erase it.
        end = time.monotonic()+1.0
        while time.monotonic() < end:
            self.send('d435', [d]); time.sleep(.06)
        self.wait(lambda: not self.occupied('unik3d', *u[:2]))
        self.assertTrue(self.occupied('combined', *d[:2]))
        self.assertFalse(self.occupied('combined', *u[:2]))
        self.send('unik3d', [u], stamp=self.now-2)
        self.send('d435', [d], frame='wrong_frame')
        self.wait(lambda: self.diagnostic_count('unik3d', 'stale_dropped') > 0 and
                         self.diagnostic_count('d435', 'invalid_dropped') > 0)
        # Independent expiry also removes measured occupancy when input stops.
        self.wait(lambda: not self.occupied('d435', *d[:2]), 3)
        self.wait(lambda: not self.occupied('combined', *d[:2]), 3)
        # Unknown area is not silently classified as free.
        with self.lock: grid = self.maps['combined']
        self.assertIn(-1, grid.data)
        # A moving base and a delayed measurement must use the acquisition pose.
        with self.lock: old_stamp = self.now-.05; self.robot_x = 1.0
        time.sleep(.12)
        self.send('d435', [d], stamp=old_stamp)
        self.wait(lambda: self.occupied('d435', *d[:2]))
        self.assertFalse(self.occupied('d435', d[0]+1, d[1]))


if __name__ == '__main__':
    rospy.init_node('test_source_layers')
    rostest.rosrun('source_costmaps', 'source_layers', LayersTest)
