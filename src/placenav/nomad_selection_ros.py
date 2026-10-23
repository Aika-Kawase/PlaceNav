"""Synchronous NoMaD selector; debug outputs only, never publishes cmd_vel."""
import json
import math
import time
import threading
import copy

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from placenav.nomad_selection import evaluate, transform_xy


def se2(transform):
    p, q = transform.translation, transform.rotation
    # Selection is planar; reject tilted transforms rather than silently flatten.
    if abs(q.x) > .02 or abs(q.y) > .02:
        raise ValueError("selector requires planar TF/map origin")
    return np.array([p.x, p.y, 2*math.atan2(q.z, q.w)])


class CandidateSelector:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.map = None
        self.map_wall = 0.
        self.previous = None
        self.previous_frame = None
        self.previous_stamp = None
        self.previous_subgoal = None
        self.tf = tf2_ros.Buffer(rospy.Duration(15))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.sub = rospy.Subscriber(args.nomad_costmap_topic, OccupancyGrid,
                                    self.on_map, queue_size=1)
        self.update_sub = rospy.Subscriber(args.nomad_costmap_topic + "_updates",
                                          OccupancyGridUpdate, self.on_update, queue_size=10)
        self.markers = rospy.Publisher("/toponav/nomad_candidates", MarkerArray, queue_size=1)
        self.diagnostics = rospy.Publisher("/toponav/nomad_selection", String, queue_size=1)

    def on_map(self, msg):
        with self.lock:
            if self.map is not None and msg.header.stamp < self.map.header.stamp:
                # Clock rewind: discard continuity. Accept full map as new epoch.
                self.previous = None
            self.map, self.map_wall = msg, time.monotonic()

    def on_update(self, update):
        with self.lock:
            msg = self.map
            if (msg is None or update.header.frame_id != msg.header.frame_id
                    or update.header.stamp < msg.header.stamp):
                return
            if (update.x + update.width > msg.info.width
                    or update.y + update.height > msg.info.height
                    or len(update.data) != update.width*update.height):
                self.map = None  # Cannot trust an incomplete map.
                return
            updated = copy.copy(msg)
            updated.header = copy.copy(update.header)
            data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            data[update.y:update.y+update.height, update.x:update.x+update.width] = (
                np.asarray(update.data).reshape(update.height, update.width))
            updated.data = data.ravel().tolist()
            self.map, self.map_wall = updated, time.monotonic()

    def observe_single(self, path, stamp, subgoal, inference_seconds):
        """Record A without filtering/changing its output or requiring a map."""
        paths = np.asarray([path])
        self.publish_markers(paths, self.args.nomad_base_frame, stamp, 0, [])
        self.diagnostics.publish(String(data=json.dumps(dict(
            query_timestamp=stamp.to_sec(), subgoal_idx=int(subgoal),
            mode="single", selected_index=0, reason="BASELINE_UNFILTERED",
            seed=self.args.nomad_seed, candidate_count=1,
            inference_seconds=inference_seconds, selection_seconds=0.,
            candidate_xy_base=paths.tolist(), candidates=[]), allow_nan=False)))

    def select(self, candidates, stamp, subgoal, inference_seconds):
        start = time.monotonic()
        selected, records, reason = -1, [], "NO_COSTMAP"
        world, frame = None, self.args.nomad_base_frame
        with self.lock:
            msg, received = self.map, self.map_wall
        now = rospy.Time.now().to_sec()
        try:
            if not 0 < stamp.to_sec() <= now + .1 or now-stamp.to_sec() > self.args.nomad_input_timeout:
                raise ValueError("STALE_QUERY")
            if msg is None:
                raise ValueError("NO_COSTMAP")
            if (not msg.header.frame_id or msg.header.stamp.to_sec() <= 0
                    or not -.1 <= now-msg.header.stamp.to_sec() <= self.args.nomad_input_timeout
                    or time.monotonic()-received > self.args.nomad_input_timeout):
                raise ValueError("STALE_COSTMAP")
            tr = self.tf.lookup_transform(msg.header.frame_id, self.args.nomad_base_frame,
                                          stamp, rospy.Duration(.05))
            robot = se2(tr.transform)
            from geometry_msgs.msg import Transform
            origin = Transform(translation=msg.info.origin.position,
                               rotation=msg.info.origin.orientation)
            origin = se2(origin)
            grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            # Must use actual bridge footprint/configuration; never guess robot size.
            footprint = rospy.get_param("/placenav_teb_bridge/footprint")
            padding = float(rospy.get_param("/placenav_teb_bridge/footprint_padding", 0.))
            if not math.isfinite(padding) or padding < 0:
                raise ValueError("INVALID_PADDING")
            frame = msg.header.frame_id
            world = np.array([transform_xy(p, robot) for p in candidates])
            if (self.previous_frame != frame or self.previous_stamp is None
                    or stamp <= self.previous_stamp
                    or (stamp-self.previous_stamp).to_sec() > self.args.nomad_input_timeout
                    or subgoal != self.previous_subgoal):
                self.previous = None
            selected, records = evaluate(
                candidates, grid, msg.info.resolution, origin, robot, footprint,
                clearance=self.args.nomad_clearance+padding,
                previous=self.previous,
                continuity_weight=self.args.nomad_continuity_weight)
            reason = "SELECTED" if selected >= 0 else "NO_SAFE_CANDIDATE"
            # Do not publish a path that expired while we were scoring.
            if (rospy.Time.now()-stamp).to_sec() > self.args.nomad_input_timeout:
                selected, reason = -1, "STALE_DURING_SELECTION"
            elif ((rospy.Time.now()-msg.header.stamp).to_sec() > self.args.nomad_input_timeout
                  or time.monotonic()-received > self.args.nomad_input_timeout):
                selected, reason = -1, "COSTMAP_STALE_DURING_SELECTION"
        except (ValueError, KeyError, TypeError, tf2_ros.TransformException) as error:
            selected, reason = -1, str(error)
        if selected >= 0:
            self.previous = world[selected]
            self.previous_frame, self.previous_stamp = frame, stamp
            self.previous_subgoal = subgoal
        else:
            self.previous = None
        self.publish_markers(world if world is not None else candidates,
                             frame if world is not None else self.args.nomad_base_frame,
                             stamp, selected, records)
        report = dict(query_timestamp=stamp.to_sec(), subgoal_idx=int(subgoal),
                      mode="costmap", selected_index=selected, reason=reason,
                      seed=self.args.nomad_seed, candidate_count=len(candidates),
                      inference_seconds=inference_seconds,
                      selection_seconds=time.monotonic()-start, candidates=records,
                      candidate_xy_base=[
                          p.tolist() if np.isfinite(p).all() else None for p in candidates])
        self.diagnostics.publish(String(data=json.dumps(report, allow_nan=False)))
        if selected < 0:
            rospy.logwarn_throttle(1., "NoMaD selection rejected: %s", reason)
            return None
        return candidates[selected].copy()

    def publish_markers(self, candidates, frame, stamp, selected, records):
        clear = Marker()
        clear.action = Marker.DELETEALL
        output = MarkerArray(markers=[clear])
        for i, path in enumerate(candidates):
            if not np.isfinite(path).all():
                continue
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = frame, stamp
            marker.ns, marker.id = "nomad_candidates", i
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = .035 if i == selected else .015
            valid = bool(records and records[i]["valid"])
            marker.color.r = 0. if i == selected else (0.2 if valid else 1.)
            marker.color.g = 1. if i == selected else .4
            marker.color.b = 0. if i == selected else (1. if valid else .1)
            marker.color.a = 1.
            marker.lifetime = rospy.Duration(1.)
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=.05) for p in path]
            output.markers.append(marker)
        self.markers.publish(output)
