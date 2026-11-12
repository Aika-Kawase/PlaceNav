"""Metric route prediction for the topological Bayes filter (no ROS imports)."""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np

from placenav.topomap_odometry import interpolate_odometry, relative_pose


SHORT_ODOMETRY_GAP_NS = 1_000_000_000


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


class OdometryHistory:
    """Bounded history; brief gaps retain continuity but cannot be interpolated."""
    def __init__(self, max_gap=0.2, max_speed=float('inf')):
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("odom_max_gap must be finite and positive")
        self.max_gap_ns = int(max_gap * 1e9)
        if math.isnan(max_speed) or max_speed <= 0:
            raise ValueError("Odometry maximum speed must be positive")
        self.max_speed = max_speed
        self.samples = deque(maxlen=2000)
        self.epoch = 0

    def add(self, sample):
        if (sample.stamp_ns <= 0 or not sample.frame_id or not sample.child_frame_id
                or not all(math.isfinite(v) for v in sample.pose)):
            raise ValueError("Invalid odometry pose, timestamp or frames")
        reason = None
        if self.samples:
            last = self.samples[-1]
            if sample.stamp_ns < last.stamp_ns:
                reason = "timestamp moved backwards"
            elif (sample.frame_id, sample.child_frame_id) != (
                    last.frame_id, last.child_frame_id):
                reason = "coordinate frames changed"
            # A brief publication/recording gap need not reset the live frame.
            # Keep its endpoint delta (including yaw) if the speed is plausible.
            # at() still enforces max_gap_ns when interpolating an image time.
            elif sample.stamp_ns - last.stamp_ns > max(
                    self.max_gap_ns, SHORT_ODOMETRY_GAP_NS):
                reason = "odometry input gap"
            elif sample.stamp_ns == last.stamp_ns:
                self.samples.pop()
            elif math.hypot(sample.pose[0] - last.pose[0], sample.pose[1] - last.pose[1]) > (
                    self.max_speed * (sample.stamp_ns - last.stamp_ns) / 1e9):
                reason = "odometry pose jump"
        if reason:
            self.samples.clear()
            self.epoch += 1
        self.samples.append(sample)
        return reason

    def at(self, stamp_ns):
        match = interpolate_odometry(list(self.samples), stamp_ns, self.max_gap_ns)
        return match[0] if match is not None else None


class OdometryMotionModel:
    """Keep sub-node progress while exposing the original node probabilities.

    Each node owns the route interval between adjacent-node midpoints. A fine
    metric grid within these intervals retains small motion between VPR calls.
    The conditional heading offset retains in-place turns and lateral motion.
    VPR still multiplies exactly the same per-node observation likelihoods.
    """
    def __init__(self, path, image_names, noise=0.1):
        if not math.isfinite(noise) or noise < 0:
            raise ValueError("odom_noise must be finite and nonnegative")
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError("Odometry motion requires " + str(path))
        data = json.loads(path.read_text(encoding="utf-8"))
        if (data.get("schema_version") != 1 or
                data.get("units") != {"position": "m", "angle": "rad"}):
            raise ValueError("Unsupported node_odometry.json schema or units")
        nodes = data["nodes"]
        names = [Path(name).name for name in image_names]
        if not nodes or len(nodes) != len(names) or any(
                node["id"] != i or node["image"] != names[i]
                for i, node in enumerate(nodes)):
            raise ValueError("Odometry nodes do not match the VPR image order")
        poses = np.array([[node["pose"][k] for k in ("x", "y", "yaw")]
                          for node in nodes], dtype=float)
        if not np.isfinite(poses).all():
            raise ValueError("Non-finite topomap pose")
        self.child_frame_id = data.get("child_frame_id")
        if not self.child_frame_id:
            raise ValueError("Topomap child_frame_id is missing")
        distances = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
        if np.any(distances <= 0):
            raise ValueError("Consecutive odometry nodes must have distinct positions")
        self.node_s = np.r_[0., np.cumsum(distances)]
        resolution = min(0.1, float(distances.min()) / 2) if len(distances) else 0.1
        count = max(1, int(np.ceil(self.node_s[-1] / resolution)))
        self.grid_s = np.linspace(0, self.node_s[-1], count + 1) if len(nodes) > 1 else np.array([0.])
        self.owner = np.searchsorted((self.node_s[:-1] + self.node_s[1:]) / 2,
                                    self.grid_s)
        self.owner_counts = np.bincount(self.owner, minlength=len(nodes))
        self.yaw = np.interp(self.grid_s, self.node_s, np.unwrap(poses[:, 2]))
        if len(nodes) > 1:
            segment = np.clip(np.searchsorted(self.node_s, self.grid_s, side="right") - 1,
                              0, len(nodes) - 2)
            self.tangent = np.diff(poses[:, :2], axis=0)[segment] / distances[segment, None]
        else:
            self.tangent = np.array([[math.cos(poses[0, 2]), math.sin(poses[0, 2])]])
        self.noise = noise
        self.probability = None
        self.heading_offset = np.zeros(len(self.grid_s))
        self.last_odometry = None
        self.last_delta = (0., 0., 0.)

    def initialize(self, belief):
        belief = np.asarray(belief, dtype=float)
        self.probability = belief[self.owner] / self.owner_counts[self.owner]
        self.probability /= self.probability.sum()
        self.reset_odometry()

    def reset_odometry(self):
        """Reanchor the live coordinate history, preserving location belief."""
        self.last_odometry = None
        self.last_delta = (0., 0., 0.)
        self.heading_offset.fill(0)

    def node_belief(self):
        return np.bincount(self.owner, weights=self.probability,
                           minlength=len(self.node_s))

    def progress_probabilities(self, node_idx, tolerance=0.5):
        """Read global posterior mass; do not renormalize a local selection."""
        position = self.node_s[node_idx]
        return {
            'passed': float(self.probability[self.grid_s >= position].sum()),
            'near': float(self.probability[np.abs(self.grid_s - position) <= tolerance].sum()),
            'local': float(self.probability[np.abs(self.owner - node_idx) <= 1].sum()),
        }

    def predict(self, sample):
        if sample.child_frame_id != self.child_frame_id:
            raise ValueError("Live odometry child frame differs from topomap body frame")
        previous = self.last_odometry
        if previous is not None and sample == previous:
            return
        if previous is not None and (sample.stamp_ns <= previous.stamp_ns or
                (sample.frame_id, sample.child_frame_id) !=
                (previous.frame_id, previous.child_frame_id)):
            self.reset_odometry()
            previous = None
        self.last_odometry = sample
        if previous is None:
            return
        dx, dy, dyaw = relative_pose(previous.pose, sample.pose)
        self.last_delta = (dx, dy, dyaw)
        if math.hypot(dx, dy) < 1e-12:
            self.heading_offset = wrap(self.heading_offset + dyaw)
            return
        heading = self.yaw + self.heading_offset
        gx, gy = np.cos(heading) * dx - np.sin(heading) * dy, np.sin(heading) * dx + np.cos(heading) * dy
        advance = gx * self.tangent[:, 0] + gy * self.tangent[:, 1]
        absolute_heading = heading + dyaw
        cosine, sine = np.cos(absolute_heading), np.sin(absolute_heading)
        sigma = self.noise * math.hypot(dx, dy)
        # Three sigma points preserve zero mean and variance sigma**2.
        shifts = [(0., 1.)] if sigma == 0 else [
            (-math.sqrt(3) * sigma, 1/6), (0., 2/3), (math.sqrt(3) * sigma, 1/6)]
        probability = np.zeros_like(self.probability)
        cos_sum, sin_sum = np.zeros_like(probability), np.zeros_like(probability)
        for shift, weight in shifts:
            target = np.clip(self.grid_s + advance + shift, 0, self.node_s[-1])
            hi = np.clip(np.searchsorted(self.grid_s, target), 0, len(self.grid_s) - 1)
            lo = np.maximum(hi - 1, 0)
            width = self.grid_s[hi] - self.grid_s[lo]
            alpha = np.divide(target - self.grid_s[lo], width,
                              out=np.zeros_like(target), where=width > 0)
            for index, fraction in ((lo, 1 - alpha), (hi, alpha)):
                mass = self.probability * weight * fraction
                probability += np.bincount(index, weights=mass, minlength=len(probability))
                cos_sum += np.bincount(index, weights=mass * cosine, minlength=len(probability))
                sin_sum += np.bincount(index, weights=mass * sine, minlength=len(probability))
        self.probability = probability / probability.sum()
        self.heading_offset = wrap(np.arctan2(sin_sum, cos_sum) - self.yaw)

    def observe(self, likelihood):
        self.probability *= likelihood[self.owner]
        self.probability /= self.probability.sum()
        return self.node_belief()
