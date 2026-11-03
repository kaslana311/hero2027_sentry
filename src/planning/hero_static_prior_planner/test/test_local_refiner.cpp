#include "hero_static_prior_planner/local_refiner.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace hero_static_prior_planner::refinement;
constexpr double pi=3.14159265358979323846;
Point rotate(Point p, double yaw)
{return {p.x*std::cos(yaw)-p.y*std::sin(yaw), p.x*std::sin(yaw)+p.y*std::cos(yaw)};}
double box(Point p, Point center, double hx, double hy)
{
  const double x=std::abs(p.x-center.x)-hx, y=std::abs(p.y-center.y)-hy;
  return std::hypot(std::max(0., x), std::max(0., y))+std::min(0., std::max(x, y));
}
double segmentDistance(Point p, Point a, Point b)
{
  const double vx=b.x-a.x, vy=b.y-a.y, d=vx*vx+vy*vy;
  const double t=d>0 ? std::clamp(((p.x-a.x)*vx+(p.y-a.y)*vy)/d, 0., 1.) : 0.;
  return std::hypot(p.x-a.x-t*vx, p.y-a.y-t*vy);
}
struct Scene
{
  Raster raster;
  Window window;
  Options options;
  double yaw{}, length{4.}, width{.9};
};
Scene scene(double drift, double yaw, bool reverse=false, bool gap=false,
  bool one_side=false, bool short_tunnel=false, bool no_observations=false)
{
  Scene s;
  s.yaw=yaw;
  s.length=short_tunnel ? .35 : 4.;
  s.options.timeout=.5;
  const double start=reverse ? s.length+1. : -1.;
  const double end=reverse ? -1. : s.length+1.;
  s.window.region={rotate({s.length/2, drift}, yaw), yaw, s.length, s.width, 0};
  // Drifted static guide, but actual start is unchanged in this common
  // physical scoring frame. Robot/cloud consistency is not broken.
  s.window.reference={rotate({start, 0.}, yaw), rotate({start*.3, drift}, yaw),
    rotate({end, drift}, yaw)};
  if (reverse) {
    s.window.reference={rotate({start, 0.}, yaw), rotate({s.length+.3, drift}, yaw),
      rotate({end, drift}, yaw)};
  }
  auto & r=s.raster;
  r.width=180; r.height=180; r.resolution=.05; r.origin_x=-2.; r.origin_y=-2.;
  const auto n=r.width*r.height;
  r.signed_distance.resize(n); r.reference_distance.resize(n);
  r.occupied.assign(n, 0); r.hard.assign(n, 0);
  for (std::size_t i=0; i<n; ++i) {
    const auto world=r.position(i), p=rotate(world, -yaw);
    double d=3.;
    if (!no_observations) {
      for (int side : {-1, 1}) {
        if (one_side && side>0) {continue;}
        if (gap && p.x>.8 && p.x<1.8 && side>0) {continue;}
        d=std::min(d, box(p, {s.length/2, side*s.width/2}, s.length/2, .03+.17));
        if (p.x>=0 && p.x<=s.length && std::abs(p.y-side*s.width/2)<.04) {r.occupied[i]=1;}
      }
      d=std::min(d, box(p, {0., 0.}, .025+.17, s.width/2+.17));
      if (std::abs(p.x)<.03 && std::abs(p.y)<s.width/2) {r.occupied[i]=1;}
    }
    r.signed_distance[i]=d;
    double dr=100.;
    for (std::size_t k=1; k<s.window.reference.size(); ++k) {
      dr=std::min(dr, segmentDistance(world, s.window.reference[k-1], s.window.reference[k]));
    }
    r.reference_distance[i]=dr;
    r.hard[i]=(i%r.width<4 || i%r.width+4>=r.width || i/r.width<4 || i/r.width+4>=r.height);
  }
  return s;
}
Result run(const Scene & s)
{
  return refine(s.raster, s.window, s.options, std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(.5)));
}
void checkPass(const Scene & s, const Result & result)
{
  if (!result.applied) {std::cerr << result.reason << '\n';}
  assert(result.applied);
  assert(result.path.front().x==s.window.reference.front().x);
  assert(result.path.front().y==s.window.reference.front().y);
  assert(result.path.back().x==s.window.reference.back().x);
  assert(result.path.back().y==s.window.reference.back().y);
  bool crossed=false;
  for (const auto p : result.path) {
    const auto actual=rotate(p, -s.yaw);
    if (actual.x>.4 && actual.x<s.length-.4) {
      crossed=true;
      assert(std::abs(actual.y)<s.width/2-.16);
    }
    const auto index=s.raster.index(p);
    assert(index && !s.raster.hard[*index]);
  }
  assert(crossed);
}
int main(int argc, char ** argv)
{
  assert(argc==2);
  const std::string test=argv[1];
  if (test=="drift_sweep") {
    for (const double drift : {-.7, -.6, -.5, .5, .6, .7}) {
      for (const double yaw : {0., pi/2}) {
        for (const bool reverse : {false, true}) {
          const auto s=scene(drift, yaw, reverse);
          checkPass(s, run(s));
        }
      }
    }
  } else if (test=="wall_gap") {
    const auto s=scene(.6, 0., false, true);
    checkPass(s, run(s));
  } else if (test=="yaw_mismatch") {
    for (double yaw_error : {-3., 3.}) {
      auto s=scene(.6, 0.);
      s.window.region.yaw=yaw_error*pi/180;
      checkPass(s, run(s));
    }
  } else if (test=="one_side_rejected") {
    const auto s=scene(.6, 0., false, false, true);
    const auto result=run(s);
    assert(!result.applied && result.reason=="no_supported_wall_pair");
  } else if (test=="lintel_is_soft") {
    const auto s=scene(0., 0., false, false, false, true);
    const auto result=run(s);
    assert(result.applied && !result.guide.accepted);
    bool soft=false;
    for (const auto p : result.path) {
      const auto i=s.raster.index(p); assert(i && !s.raster.hard[*i]);
      if (std::abs(p.x)<.06 && s.raster.signed_distance[*i]<0) {soft=true;}
    }
    assert(soft);
  } else if (test=="no_observations") {
    const auto result=run(scene(.6, 0., false, false, false, false, true));
    assert(!result.applied && result.reason=="no_dynamic_observations");
  } else if (test=="budget") {
    auto s=scene(.6, 0.); s.options.max_expansions=1;
    assert(run(s).reason=="budget_exhausted");
    assert(refine(s.raster, s.window, s.options, std::chrono::steady_clock::now()).reason=="budget_exhausted");
  } else if (test=="hard_rim") {
    auto s=scene(.6, 0.);
    for (std::size_t i=0; i<s.raster.hard.size(); ++i) {
      if (std::abs(s.raster.position(i).x-2.)<.1) {s.raster.hard[i]=1;}
    }
    const auto result=run(s);
    assert(!result.applied);
  } else if (test=="invalid") {
    auto s=scene(.6, 0.);
    s.raster.signed_distance[0]=std::numeric_limits<float>::quiet_NaN();
    assert(run(s).reason=="invalid_input");
    s=scene(.6, 0.); s.raster.hard.pop_back();
    assert(run(s).reason=="invalid_input");
    s.options.timeout=0.; assert(!valid(s.options));
    assert(!valid(Region{{0., 0.}, 0., -1., .8, 0}));
  } else if (test=="window") {
    const Options options;
    std::vector<Point> path;
    for (int k=-20; k<=130; ++k) {path.push_back({k*.05, 0.});}
    const auto w=selectWindow(path, {{{2., 0.}, 0., 4., .9, 0}}, options);
    assert(w && w->join_index>0);
    assert(w->reference.back().x>=5.8-1e-9);
    assert(!selectWindow(path, {{{20., 0.}, 0., 4., .9, 0}}, options));
    auto tiny=options; tiny.lookahead=3.;
    assert(!selectWindow(path, {{{2., 0.}, 0., 4., .9, 0}}, tiny));
  } else {return 2;}
}
