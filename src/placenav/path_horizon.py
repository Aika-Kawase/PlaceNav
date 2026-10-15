"""Bounded, optional extrapolation of a short local prediction for TEB guidance."""
import math


def extend_horizon(points, target=0.0, max_extension=1.5, spacing=0.1):
    points = list(points)
    if not all(math.isfinite(v) for p in points for v in p):
        raise ValueError("Non-finite path coordinates")
    if not all(math.isfinite(v) and v >= 0 for v in (target, max_extension)) or not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("Invalid path horizon settings")
    if len(points) < 2 or target == 0:
        return points
    segments = [(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])]
    length = sum(math.hypot(x, y) for x, y in segments)
    # Do not turn a stationary/backward/lateral prediction into forward motion.
    if length < 0.2 or points[-1][0] < 0.2:
        return points
    nonzero = [(x, y) for x, y in segments if math.hypot(x, y) > 1e-6]
    if not nonzero:
        return points
    dx, dy = nonzero[-1]
    norm = math.hypot(dx, dy)
    if dx / norm < 0.5:
        return points
    extra = min(max_extension, max(0.0, target-length))
    count = int(math.ceil(extra / spacing))
    end = points[-1]
    return points + [(end[0]+dx/norm*extra*i/count,
                      end[1]+dy/norm*extra*i/count) for i in range(1, count+1)]
