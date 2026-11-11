"""Small ROS-independent helpers for image-time planar odometry."""
from bisect import bisect_left
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OdomSample:
    stamp_ns: int
    pose: tuple
    frame_id: str
    child_frame_id: str


def relative_pose(previous, current):
    """Current pose in the previous robot frame: forward x, left y, yaw."""
    dx, dy = current[0] - previous[0], current[1] - previous[1]
    c, s = math.cos(previous[2]), math.sin(previous[2])
    yaw = current[2] - previous[2]
    return (c * dx + s * dy, -s * dx + c * dy,
            math.atan2(math.sin(yaw), math.cos(yaw)))


def interpolate_odometry(samples, stamp_ns, max_gap_ns):
    """Return (sample, before_ns, after_ns), or None; never extrapolate."""
    index = bisect_left([sample.stamp_ns for sample in samples], stamp_ns)
    if index < len(samples) and samples[index].stamp_ns == stamp_ns:
        sample = samples[index]
        return sample, stamp_ns, stamp_ns
    if index == 0 or index == len(samples):
        return None
    before, after = samples[index - 1], samples[index]
    gap = after.stamp_ns - before.stamp_ns
    if (gap <= 0 or gap > max_gap_ns or
            (before.frame_id, before.child_frame_id) !=
            (after.frame_id, after.child_frame_id)):
        return None
    alpha = (stamp_ns - before.stamp_ns) / gap
    yaw_delta = relative_pose(before.pose, after.pose)[2]
    yaw = before.pose[2] + alpha * yaw_delta
    pose = (before.pose[0] + alpha * (after.pose[0] - before.pose[0]),
            before.pose[1] + alpha * (after.pose[1] - before.pose[1]),
            math.atan2(math.sin(yaw), math.cos(yaw)))
    return (OdomSample(stamp_ns, pose, before.frame_id, before.child_frame_id),
            before.stamp_ns, after.stamp_ns)
