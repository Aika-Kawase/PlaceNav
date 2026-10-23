"""ROS-independent candidate scoring in metres. Not a motion controller.

OccupancyGrid values: unknown and >=99 (inscribed/lethal) are blocked.
The footprint is a conservative rectangle enclosing the configured polygon.
Sweeps include cell-area uncertainty and translation/rotation interpolation.
"""
import math
import numpy as np


def transform_xy(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


def trajectory_poses(points):
    points = np.vstack(([0., 0.], points))
    delta = np.diff(points, axis=0)
    yaw = np.arctan2(delta[:, 1], delta[:, 0])
    # Include the initial in-place turn: robot yaw is initially zero.
    return np.vstack((np.array([0., 0., 0.]),
                      np.column_stack((points, np.r_[yaw, yaw[-1]]))))


def swept_poses(poses, resolution, radius):
    yield poses[0]
    for a, b in zip(poses, poses[1:]):
        angle = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        travel = np.linalg.norm(b[:2]-a[:2]) + radius * abs(angle)
        n = max(1, int(math.ceil(travel / (resolution * .5))))
        for t in np.linspace(0., 1., n + 1)[1:]:
            yield np.r_[a[:2] + t*(b[:2]-a[:2]), a[2]+t*angle]


def evaluate(candidates, grid, resolution, origin, robot, footprint,
             clearance=.10, previous=None, continuity_weight=1.0):
    """Return selected index (-1 if none), per-candidate metrics.

    robot/previous are expressed in the grid frame. origin is map-origin SE2.
    Endpoint x is a weak body-forward preference, NOT distance to image goal.
    """
    candidates = np.asarray(candidates, dtype=float)
    grid = np.asarray(grid)
    footprint = np.asarray(footprint, dtype=float)
    if (candidates.ndim != 3 or candidates.shape[1] < 2 or candidates.shape[2] != 2
            or grid.ndim != 2 or not grid.size or resolution <= 0
            or not math.isfinite(resolution) or clearance < 0
            or not math.isfinite(clearance) or continuity_weight < 0
            or not math.isfinite(continuity_weight)
            or footprint.ndim != 2 or footprint.shape[1] != 2 or len(footprint) < 3
            or not np.isfinite(footprint).all()
            or not np.isfinite(robot).all() or not np.isfinite(origin).all()):
        raise ValueError("invalid selector geometry/configuration")
    # Enclosing rectangle, plus requested clearance and conservative cell radius.
    margin = clearance + resolution * math.sqrt(2) / 2
    low, high = footprint.min(axis=0)-margin, footprint.max(axis=0)+margin
    radius = float(np.max(np.linalg.norm(
        [[low[0], low[1]], [low[0], high[1]],
         [high[0], low[1]], [high[0], high[1]]], axis=1)))
    corners = np.array([[low[0], low[1]], [low[0], high[1]],
                        [high[0], high[1]], [high[0], low[1]]])
    # Bound collision work for malformed/huge model outputs.
    records = []
    for index, points in enumerate(candidates):
        record = dict(index=index, valid=False, reason="INVALID_GEOMETRY")
        records.append(record)
        if not np.isfinite(points).all() or np.max(np.abs(points)) > 20:
            continue
        lengths = np.linalg.norm(np.diff(np.vstack(([0., 0.], points)), axis=0), axis=1)
        if lengths.sum() < .05 or lengths.sum() > 20:
            record["reason"] = "DEGENERATE_OR_TOO_LONG"
            continue
        poses = trajectory_poses(points)
        poses[:, :2] = transform_xy(poses[:, :2], robot)
        poses[:, 2] += robot[2]
        # Convert world poses into occupancy-grid local coordinates.
        inv = np.array([-origin[0], -origin[1]])
        poses[:, :2] = transform_xy(poses[:, :2] + inv, [0, 0, -origin[2]])
        poses[:, 2] -= origin[2]
        cost, samples, reason = 0., 0, None
        for pose in swept_poses(poses, resolution, radius):
            bounds = transform_xy(corners, pose)
            start = np.floor(bounds.min(axis=0)/resolution).astype(int)
            end = np.floor(bounds.max(axis=0)/resolution).astype(int)
            if (start < 0).any() or end[0] >= grid.shape[1] or end[1] >= grid.shape[0]:
                reason = "OUTSIDE_MAP"
                break
            xx, yy = np.meshgrid(np.arange(start[0], end[0]+1),
                                 np.arange(start[1], end[1]+1))
            cells = np.column_stack((xx.ravel(), yy.ravel()))
            local = transform_xy((cells+.5)*resolution-pose[:2],
                                 [0, 0, -pose[2]])
            inside = ((local >= low) & (local <= high)).all(axis=1)
            cells = cells[inside]
            values = grid[cells[:, 1], cells[:, 0]]
            if (values < 0).any() or (values >= 99).any():
                reason = "COLLISION_OR_UNKNOWN"
                break
            cost += float(values.max(initial=0))/100.
            samples += 1
        if reason:
            record["reason"] = reason
            continue
        world = transform_xy(points, robot)
        continuity = 0.
        if previous is not None and len(previous):
            # Compare in the same fixed frame, not moving base_link coordinates.
            continuity = float(np.mean(np.min(np.linalg.norm(
                world[:, None, :] - np.asarray(previous)[None, :, :], axis=2), axis=1)))
        mean_cost = cost/max(samples, 1)
        angles = np.diff(poses[:, 2])
        turn = float(np.abs(np.arctan2(np.sin(angles), np.cos(angles))).sum())
        score = 2*mean_cost + continuity_weight*continuity + .1*turn - .2*float(points[-1, 0])
        record.update(valid=True, reason="VALID", score=score,
                      mean_cost=mean_cost, continuity_m=continuity,
                      turn_rad=turn, forward_m=float(points[-1, 0]),
                      length_m=float(lengths.sum()))
    valid = [r for r in records if r["valid"]]
    selected = min(valid, key=lambda r: (r["score"], r["index"]))["index"] if valid else -1
    return selected, records
