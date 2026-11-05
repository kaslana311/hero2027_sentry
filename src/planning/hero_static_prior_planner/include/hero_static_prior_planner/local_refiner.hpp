#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace hero_static_prior_planner::refinement
{
struct Point {double x{}, y{};};
struct Region
{
  Point center;
  double yaw{}, length{}, width{};
  std::size_t id{};
};
struct Options
{
  double lookahead{12.0};
  double activation_distance{1.5};
  double max_deviation{1.25};
  double max_alignment_shift{1.0};
  double resolution{0.05};
  double robot_radius{0.17};
  double safe_distance{0.30};
  double dynamic_weight{1.0};
  double guide_weight{100.0};
  double timeout{0.12};
  std::size_t max_expansions{50000};
};
struct Raster
{
  std::size_t width{}, height{};
  double resolution{}, origin_x{}, origin_y{};
  std::vector<float> signed_distance, reference_distance;
  std::vector<unsigned char> occupied, hard, prior_blocked;
  Point position(std::size_t index) const;
  std::optional<std::size_t> index(Point p) const;
};
struct Window
{
  Region region;
  std::vector<Point> reference;
  std::size_t join_index{};
};
struct Guide
{
  bool accepted{false};
  Point center;
  double yaw{}, length{}, width{};
  std::size_t overlap_bins{};
};
struct Result
{
  bool applied{false};
  std::string reason;
  std::vector<Point> path;
  Guide guide;
  std::size_t expansions{};
};
using Deadline = std::chrono::steady_clock::time_point;
bool valid(const Options & options);
bool valid(const Region & region);
std::optional<Window> selectWindow(
  const std::vector<Point> & reference, const std::vector<Region> & regions,
  const Options & options);
Result refine(const Raster & raster, const Window & window, const Options & options,
  Deadline deadline);
// Use a short arc-length chord instead of alternating 8-connected edge yaw.
std::vector<double> pathHeadings(const std::vector<Point> & path, double span = 0.15);
}  // namespace hero_static_prior_planner::refinement
