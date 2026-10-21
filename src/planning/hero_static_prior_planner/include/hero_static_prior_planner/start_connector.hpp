#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <tuple>
#include <vector>

namespace hero_static_prior_planner
{
namespace detail
{
struct Point
{
  double x, y;
};

struct Grid
{
  unsigned int width, height;
  double resolution, origin_x, origin_y;
};

using FreeAt = std::function<bool(double, double)>;

// The connector is deliberately holonomic: the HERO chassis can translate
// sideways at constant yaw. It is not a Dubins motion primitive for a car.
inline std::vector<Point> sampleConnector(
  const Point & start, const Point & end, double step)
{
  if (!std::isfinite(step) || step <= 0.0 ||
    !std::isfinite(start.x) || !std::isfinite(start.y) ||
    !std::isfinite(end.x) || !std::isfinite(end.y))
  {
    return {};
  }
  const double distance = std::hypot(end.x-start.x, end.y-start.y);
  const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance/step)));
  std::vector<Point> points;
  points.reserve(count+1);
  for (size_t i = 0; i <= count; ++i) {
    const double f = static_cast<double>(i)/count;
    points.push_back({start.x+f*(end.x-start.x), start.y+f*(end.y-start.y)});
  }
  return points;
}

inline bool connectorIsFree(const std::vector<Point> & points, const FreeAt & dynamic_free)
{
  return !points.empty() && std::all_of(points.begin(), points.end(),
    [&](const Point & p) {return dynamic_free(p.x, p.y);});
}

// Search only nearby cell centres on the unmodified prior. Prefer the shortest
// displacement, then the direction toward the goal. No teleport, raster edit,
// cached path or second full-map planner is introduced.
inline std::optional<std::vector<Point>> findStartConnector(
  const Grid & grid, const Point & start, const Point & goal,
  double radius, double step, const FreeAt & static_free, const FreeAt & dynamic_free)
{
  if (grid.width == 0 || grid.height == 0 || !std::isfinite(grid.resolution) ||
    grid.resolution <= 0.0 || !std::isfinite(radius) || radius <= 0.0 ||
    !std::isfinite(step) || step <= 0.0 ||
    !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y) ||
    !std::isfinite(start.x) || !std::isfinite(start.y) ||
    !std::isfinite(goal.x) || !std::isfinite(goal.y))
  {
    return std::nullopt;
  }
  if (start.x < grid.origin_x || start.y < grid.origin_y ||
    start.x >= grid.origin_x+grid.width*grid.resolution ||
    start.y >= grid.origin_y+grid.height*grid.resolution ||
    !dynamic_free(start.x, start.y))
  {
    return std::nullopt;
  }
  struct Candidate
  {
    Point point;
    double distance2, goal_distance2;
  };
  std::vector<Candidate> candidates;
  const int x0 = std::max(0, static_cast<int>(std::floor(
      (start.x-radius-grid.origin_x)/grid.resolution)));
  const int y0 = std::max(0, static_cast<int>(std::floor(
      (start.y-radius-grid.origin_y)/grid.resolution)));
  const int x1 = std::min(static_cast<int>(grid.width)-1, static_cast<int>(std::floor(
      (start.x+radius-grid.origin_x)/grid.resolution)));
  const int y1 = std::min(static_cast<int>(grid.height)-1, static_cast<int>(std::floor(
      (start.y+radius-grid.origin_y)/grid.resolution)));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      Point p{grid.origin_x+(x+.5)*grid.resolution, grid.origin_y+(y+.5)*grid.resolution};
      const double d2 = std::pow(p.x-start.x, 2)+std::pow(p.y-start.y, 2);
      if (d2 <= radius*radius && static_free(p.x, p.y)) {
        candidates.push_back({p, d2,
          std::pow(p.x-goal.x, 2)+std::pow(p.y-goal.y, 2)});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    // Round-off must not break ties between symmetric left/right cell centres.
    return std::make_tuple(std::round(a.distance2*1e6), a.goal_distance2, a.point.x, a.point.y) <
           std::make_tuple(std::round(b.distance2*1e6), b.goal_distance2, b.point.x, b.point.y);
  });
  for (const auto & candidate : candidates) {
    auto points = sampleConnector(start, candidate.point, step);
    if (connectorIsFree(points, dynamic_free)) {
      return points;
    }
  }
  return std::nullopt;
}
}  // namespace detail
}  // namespace hero_static_prior_planner
