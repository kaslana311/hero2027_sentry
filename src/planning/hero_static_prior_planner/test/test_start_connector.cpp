#include "hero_static_prior_planner/start_connector.hpp"

#include <cassert>
#include <limits>
#include <string>

using hero_static_prior_planner::detail::findStartConnector;
using hero_static_prior_planner::detail::Grid;
using hero_static_prior_planner::detail::Point;

int main(int argc, char ** argv)
{
  assert(argc == 2);
  const std::string test = argv[1];
  const Grid grid{120, 80, .05, -1., -2.};
  const Point start{2., 0.}, goal{4., 0.};
  const auto prior = [](double x, double) {return x < 1.8 || x > 2.2;};
  const auto free = [](double, double) {return true;};
  if (test == "nearest_exit" || test == "new_goal") {
    const Point destination = test == "new_goal" ? Point{0., 0.} : goal;
    const auto result = findStartConnector(grid, start, destination, .6, .0125, prior, free);
    assert(result && result->size() > 2);
    assert(result->front().x == start.x && result->front().y == start.y);
    assert(prior(result->back().x, result->back().y));
    assert(std::hypot(result->back().x-start.x, result->back().y-start.y) < .23);
    assert(test == "new_goal" ? result->back().x < 1.8 : result->back().x > 2.2);
    for (size_t i = 1; i < result->size(); ++i) {
      assert(std::hypot((*result)[i].x-(*result)[i-1].x,
        (*result)[i].y-(*result)[i-1].y) <= .0125+1e-12);
    }
  } else if (test == "float_map_geometry") {
    // OccupancyGrid stores resolution as float32. Sub-micron asymmetry must
    // not select the wrong side instead of the equally near goal-facing exit.
    const Grid actual_grid{120, 80, static_cast<double>(.05F), 0., 0.};
    const auto wall = [](double x, double) {return x < 2.65 || x > 3.35;};
    const auto result = findStartConnector(actual_grid, {3., 1.}, {5., 1.},
      .6, .0125, wall, free);
    assert(result && result->back().x > 3.35);
  } else if (test == "radius_limit") {
    assert(!findStartConnector(grid, start, goal, .1, .0125, prior, free));
  } else if (test == "occupied_start") {
    const auto dynamic = [](double x, double) {return x < 1.95 || x > 2.05;};
    assert(!findStartConnector(grid, start, goal, .6, .0125, prior, dynamic));
  } else if (test == "blocked_connection" || test == "unknown_connection") {
    // Safe origin and safe anchor are insufficient: every intermediate point
    // must also pass. False can represent either occupied or unknown cells.
    const auto dynamic = [](double x, double y) {
        const double d = std::hypot(x-2., y);
        return d < .12 || d > .20;
      };
    assert(dynamic(start.x, start.y));
    assert(!findStartConnector(grid, start, goal, .6, .0125, prior, dynamic));
  } else if (test == "thin_wall") {
    const auto right_only = [](double x, double) {return x > 2.2;};
    const auto dynamic = [](double x, double) {return x < 2.075 || x > 2.10;};
    assert(!findStartConnector(grid, start, goal, .6, .0125, right_only, dynamic));
  } else if (test == "alternative_exit") {
    const auto dynamic = [](double x, double) {return x < 2.075 || x > 2.10;};
    const auto result = findStartConnector(grid, start, goal, .6, .0125, prior, dynamic);
    assert(result && result->back().x < 1.8);
    for (const auto & p : *result) {assert(dynamic(p.x, p.y));}
  } else if (test == "invalid_input") {
    assert(!findStartConnector(grid, start, goal, 0., .01, prior, free));
    assert(!findStartConnector(grid, start, goal, .6, 0., prior, free));
    assert(!findStartConnector(grid, {-2., 0.}, goal, .6, .01, prior, free));
    assert(!findStartConnector(grid, {5., 0.}, goal, .6, .01, prior, free));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    assert(!findStartConnector(grid, {nan, 0.}, goal, .6, .01, prior, free));
    assert(!findStartConnector(grid, start, {nan, 0.}, .6, .01, prior, free));
    assert(!findStartConnector({0, 80, .05, -1., -2.}, start, goal, .6, .01, prior, free));
    assert(!findStartConnector({120, 80, nan, -1., -2.}, start, goal, .6, .01, prior, free));
  } else {
    return 2;
  }
  return 0;
}
