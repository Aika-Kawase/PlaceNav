#!/usr/bin/env python3
"""Consume each gated odom observation once, then fan out the same batch.

Only newly received observations are buffered. Source distance/height filtering
has already happened at acquisition time in the two gates. No TF or clearing is
performed here. The common header uses the maximum original acquisition stamp;
all original stamps are retained in batch_trace for auditing.
"""
import json
import math
import threading
import time
from collections import deque

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String


class CloudBatch:
    SOURCES = ('d435', 'unik3d')

    def __init__(self):
        self.lock = threading.RLock()
        self.interval = float(rospy.get_param('~batch_interval', .1))
        self.limit = int(rospy.get_param('~max_pending_clouds', 64))
        self.point_limit = int(rospy.get_param('~max_pending_points', 200000))
        if not math.isfinite(self.interval) or not 0 < self.interval <= .2 or self.limit < 1 or self.point_limit < 1:
            raise ValueError('invalid batch interval/queue limit')
        self.queues = {s: deque() for s in self.SOURCES}
        self.newest = {s: 0 for s in self.SOURCES}
        self.counters = {s: {k: 0 for k in ('received', 'consumed', 'invalid', 'stale', 'order', 'overflow')}
                         for s in self.SOURCES}
        self.last_clock = 0
        self.resets = 0
        self.batch_id = 0
        self.frame = rospy.get_param('/obstacles/d435_gate/fixed_frame', 'odom')
        if not self.frame or self.frame != rospy.get_param('/obstacles/unik3d_gate/fixed_frame', 'odom'):
            raise ValueError('both gates must use the same nonempty fixed_frame')
        self.age = {s: float(rospy.get_param('/obstacles/'+s+'_gate/max_age')) for s in self.SOURCES}
        self.future = {s: float(rospy.get_param('/obstacles/'+s+'_gate/future_tolerance', .05))
                       for s in self.SOURCES}
        self.pubs = {s: rospy.Publisher('~'+s+'/points', PointCloud2, queue_size=10) for s in self.SOURCES}
        self.fused = rospy.Publisher('~fused/points', PointCloud2, queue_size=10)
        self.trace = rospy.Publisher('~batch_trace', String, queue_size=100)
        self.diagnostics = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size=10)
        self.config = {k: rospy.get_param('~'+k) for k in
                       ('fusion_mode', 'comparison_profile', 'common_voxel_size', 'common_decay')}
        self.config.update(batch_interval=self.interval, max_age=self.age, fixed_frame=self.frame,
                           header_policy='maximum_acquisition_stamp', observation_persistence=2.0)
        self.metadata = rospy.Publisher('~metadata', String, queue_size=1, latch=True)
        self.metadata.publish(String(json.dumps(self.config, sort_keys=True)))
        self.subs = [rospy.Subscriber('/obstacles/'+s+'/points', PointCloud2, self.receive, callback_args=s,
                                     queue_size=64, buff_size=8*1024*1024) for s in self.SOURCES]
        self.timer = rospy.Timer(rospy.Duration(self.interval), self.flush, reset=True)
        self.heartbeat = rospy.Timer(rospy.Duration(1), self.diagnose, reset=True)

    def clock(self, now):
        if now < self.last_clock:
            for s in self.SOURCES:
                self.queues[s].clear()
                self.newest[s] = 0
            self.resets += 1
        self.last_clock = now

    def receive(self, msg, source):
        with self.lock:
            now = rospy.Time.now().to_nsec()
            self.clock(now)
            c = self.counters[source]
            c['received'] += 1
            stamp = msg.header.stamp.to_nsec()
            age = (now-stamp)/1e9
            if msg.header.frame_id != self.frame or not stamp or not now:
                c['invalid'] += 1
                return
            if not -self.future[source] <= age <= self.age[source]:
                c['stale'] += 1
                return
            if stamp <= self.newest[source]:
                c['order'] += 1
                return
            # Only the canonical XYZ output of cloud_gate_node is accepted.
            fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
            if (fields != [(v, i*4, PointField.FLOAT32, 1) for i, v in enumerate(('x', 'y', 'z'))]
                    or msg.height != 1 or msg.point_step != 12 or msg.row_step != msg.width*12
                    or len(msg.data) != msg.row_step or msg.width > self.point_limit):
                c['invalid'] += 1
                return
            points = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4').reshape(-1, 3).copy()
            if not np.isfinite(points).all():
                c['invalid'] += 1
                return
            self.newest[source] = stamp
            q = self.queues[source]
            while q and (len(q) >= self.limit or sum(len(o[1]) for o in q)+len(points) > self.point_limit):
                q.popleft()
                c['overflow'] += 1
            q.append((stamp, points, time.monotonic()))

    def flush(self, _):
        began = time.monotonic()
        with self.lock:
            now = rospy.Time.now()
            self.clock(now.to_nsec())
            accepted = {}
            for s in self.SOURCES:
                accepted[s] = []
                while self.queues[s]:
                    obs = self.queues[s].popleft()
                    if not -self.future[s] <= (now.to_nsec()-obs[0])/1e9 <= self.age[s]:
                        self.counters[s]['stale'] += 1
                    else:
                        accepted[s].append(obs)
                        self.counters[s]['consumed'] += 1
            observations = [o for values in accepted.values() for o in values]
            if not observations:
                return  # Never republish the last batch, even an empty one.
            self.batch_id += 1
            stamp_ns = max(o[0] for o in observations)
            header = Header(seq=self.batch_id, stamp=rospy.Time(stamp_ns//10**9, stamp_ns % 10**9), frame_id=self.frame)
            arrays = []
            for s in self.SOURCES:
                if accepted[s]:
                    points = np.concatenate([o[1] for o in accepted[s]], axis=0)
                    arrays.append(points)
                    self.pubs[s].publish(point_cloud2.create_cloud_xyz32(header, points))
            combined = np.concatenate(arrays, axis=0)
            self.fused.publish(point_cloud2.create_cloud_xyz32(header, combined))
            trace = {'batch_id': self.batch_id, 'clock_ns': now.to_nsec(), 'header_stamp_ns': stamp_ns,
                     'points': len(combined), 'sources': {}, 'processing_ms': (time.monotonic()-began)*1000}
            for s in self.SOURCES:
                trace['sources'][s] = {'stamps_ns': [o[0] for o in accepted[s]],
                                       'points': sum(len(o[1]) for o in accepted[s]),
                                       'max_queue_wait_ms': max(((time.monotonic()-o[2])*1000 for o in accepted[s]), default=0),
                                       'max_acquisition_age_s': max(((now.to_nsec()-o[0])/1e9 for o in accepted[s]), default=None)}
            self.trace.publish(String(json.dumps(trace, sort_keys=True)))

    def diagnose(self, _):
        with self.lock:
            self.clock(rospy.Time.now().to_nsec())
            msg = DiagnosticArray(header=Header(stamp=rospy.Time.now()))
            for s in self.SOURCES:
                status = DiagnosticStatus(name=rospy.get_name()+'/'+s, hardware_id=s)
                missing = not self.newest[s] or (rospy.Time.now().to_nsec()-self.newest[s])/1e9 > self.age[s]
                status.level = DiagnosticStatus.WARN if missing else DiagnosticStatus.OK
                status.message = 'missing_or_expired' if missing else 'ok'
                values = dict(self.counters[s], pending=len(self.queues[s]), clock_resets=self.resets, batches=self.batch_id)
                status.values = [KeyValue(str(k), str(v)) for k, v in values.items()]
                msg.status.append(status)
            self.diagnostics.publish(msg)


if __name__ == '__main__':
    rospy.init_node('comparison')
    try:
        batch = CloudBatch()
        rospy.spin()
    except (ValueError, KeyError) as error:
        rospy.logfatal('Comparison configuration: %s', error)
        raise SystemExit(1)
