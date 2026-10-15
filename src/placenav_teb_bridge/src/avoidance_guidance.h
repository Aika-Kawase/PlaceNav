#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace avoidance_guidance {
struct Point { double x, y; };
inline double distance(Point a, Point b) { return std::hypot(a.x-b.x, a.y-b.y); }

// Retain the near avoidance prefix and use only a forward PlaceNav endpoint.
// This is a planning seed, never a collision-free trajectory or velocity command.
inline std::vector<Point> blend(const std::vector<Point>& seed,
                                const std::vector<Point>& route,
                                Point robot, double keep_length,
                                double max_connection)
{
  if (seed.size() < 2 || route.size() < 2) return {};
  for (const auto& path : {seed, route})
    for (auto p : path) if (!std::isfinite(p.x) || !std::isfinite(p.y)) return {};
  Point goal = route.back();
  double dx = goal.x - robot.x, dy = goal.y - robot.y;
  double norm = std::hypot(dx, dy);
  if (norm < keep_length + 0.3) return {};
  dx /= norm; dy /= norm;
  std::size_t closest = 0;
  for (std::size_t i=1; i<seed.size(); ++i)
    if (distance(seed[i], robot) < distance(seed[closest], robot)) closest=i;
  std::vector<Point> out{seed[closest]};
  double length=0;
  for (std::size_t i=closest+1; i<seed.size(); ++i) {
    Point a=out.back(), b=seed[i];
    double segment=distance(a,b);
    if (segment < 1e-9) continue;
    double take=std::min(segment, keep_length-length);
    out.push_back({a.x+(b.x-a.x)*take/segment, a.y+(b.y-a.y)*take/segment});
    length+=take;
    if (length >= keep_length-1e-9) break;
  }
  if (out.size()<2 || length<0.3) return {};
  Point end=out.back();
  // Never append a goal behind the retained avoidance prefix in route direction.
  if ((goal.x-end.x)*dx+(goal.y-end.y)*dy < 0.3) return {};
  double connection=distance(end, goal);
  if (connection>max_connection) return {};
  int count=static_cast<int>(std::ceil(connection/0.1));
  for (int i=1; i<=count; ++i)
    out.push_back({end.x+(goal.x-end.x)*i/count, end.y+(goal.y-end.y)*i/count});
  return out;
}
}
