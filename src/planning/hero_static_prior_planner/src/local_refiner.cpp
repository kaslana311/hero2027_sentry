#include "hero_static_prior_planner/local_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <utility>

namespace hero_static_prior_planner::refinement
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double distance(Point a, Point b) {return std::hypot(a.x-b.x, a.y-b.y);}
Point axis(double yaw) {return {std::cos(yaw), std::sin(yaw)};}
double longitudinal(Point p, Point c, double yaw)
{return (p.x-c.x)*std::cos(yaw)+(p.y-c.y)*std::sin(yaw);}
double lateral(Point p, Point c, double yaw)
{return -(p.x-c.x)*std::sin(yaw)+(p.y-c.y)*std::cos(yaw);}
bool finite(Point p) {return std::isfinite(p.x) && std::isfinite(p.y);}
bool expired(Deadline deadline) {return std::chrono::steady_clock::now() >= deadline;}
double median(std::vector<double> values)
{
  const auto middle = values.begin()+values.size()/2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

// Fit paired longitudinal surfaces, not a complete/global localization pose.
// Both sides must bracket the robot and overlap in longitudinal support. A
// transverse lintel alone cannot pass that test. All geometry remains soft.
Guide fitGuide(const Raster & raster, const Window & window, const Options & options,
  Deadline deadline)
{
  std::vector<Point> points;
  const auto & region = window.region;
  const auto robot = window.reference.front();
  for (std::size_t i=0; i<raster.occupied.size(); ++i) {
    if (!raster.occupied[i] || raster.hard[i]) {continue;}
    const auto p = raster.position(i);
    if (distance(p, robot)>4.5 ||
      std::abs(longitudinal(p, region.center, region.yaw))>region.length/2+1.0 ||
      std::abs(lateral(p, region.center, region.yaw))>region.width/2+options.max_alignment_shift+.3)
    {continue;}
    points.push_back(p);
  }
  Guide best;
  if (points.size()<30) {return best;}
  double best_score = -std::numeric_limits<double>::infinity();
  const double extent = region.width/2+options.max_alignment_shift+.4;
  constexpr double histogram_step = .04, support_step = .15;
  const auto bins = static_cast<std::size_t>(std::ceil(2*extent/histogram_step));
  for (int angle=-8; angle<=8; ++angle) {
    if (expired(deadline)) {return {};}
    const double yaw = region.yaw+angle*kPi/180;
    std::vector<std::size_t> counts(bins, 0), peaks;
    std::vector<double> u, v;
    for (const auto p : points) {
      u.push_back(longitudinal(p, region.center, yaw));
      v.push_back(lateral(p, region.center, yaw));
      const auto bin = static_cast<long>(std::floor((v.back()+extent)/histogram_step));
      if (bin>=0 && static_cast<std::size_t>(bin)<bins) {++counts[bin];}
    }
    for (std::size_t i=1; i+1<bins; ++i) {
      if (counts[i]>=4 && counts[i]>counts[i-1] && counts[i]>=counts[i+1]) {
        peaks.push_back(i);
      }
    }
    const double robot_v = lateral(robot, region.center, yaw);
    for (std::size_t a=0; a<peaks.size(); ++a) {
      if (expired(deadline)) {return {};}
      for (std::size_t b=a+1; b<peaks.size(); ++b) {
        const double lo = -extent+(peaks[a]+.5)*histogram_step;
        const double hi = -extent+(peaks[b]+.5)*histogram_step;
        if (hi-lo<.55 || hi-lo>1.40 || std::abs(hi-lo-region.width)>.4 ||
          robot_v<=lo+.05 || robot_v>=hi-.05 ||
          std::abs((lo+hi)/2)>options.max_alignment_shift) {continue;}
        std::set<int> left, right;
        std::vector<double> lv, rv;
        for (std::size_t j=0; j<v.size(); ++j) {
          const int bin = static_cast<int>(std::floor(u[j]/support_step));
          if (std::abs(v[j]-lo)<.065) {left.insert(bin); lv.push_back(v[j]);}
          if (std::abs(v[j]-hi)<.065) {right.insert(bin); rv.push_back(v[j]);}
        }
        if (left.size()<5 || right.size()<5) {continue;}
        std::size_t overlap = 0;
        for (int bin : left) {if (right.count(bin)) {++overlap;}}
        if (overlap<4) {continue;}
        const double offset = (median(lv)+median(rv))/2;
        const double width = median(rv)-median(lv);
        if (width<.55 || width>1.4 || std::abs(offset)>options.max_alignment_shift) {continue;}
        const double score = 3*overlap+std::min(left.size(), right.size())-
          3*std::abs(offset-robot_v)-2*std::abs(width-region.width)-.05*std::abs(angle);
        if (score>best_score) {
          best_score=score;
          best={true, {region.center.x-offset*std::sin(yaw),
              region.center.y+offset*std::cos(yaw)}, yaw, region.length, width, overlap};
        }
      }
    }
  }
  return best;
}

double softPenalty(double violation)
{
  if (violation<=0) {return 0;}
  if (violation>=.01) {return violation-.005;}
  return (-.5e6*violation+1e4)*violation*violation*violation;
}
}  // namespace

Point Raster::position(std::size_t i) const
{return {origin_x+(i%width+.5)*resolution, origin_y+(i/width+.5)*resolution};}
std::optional<std::size_t> Raster::index(Point p) const
{
  if (!finite(p) || !std::isfinite(resolution) || resolution<=0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y)) {return {};}
  const double x = std::floor((p.x-origin_x)/resolution);
  const double y = std::floor((p.y-origin_y)/resolution);
  if (x<0 || y<0 || x>=width || y>=height) {return {};}
  return static_cast<std::size_t>(y)*width+static_cast<std::size_t>(x);
}
bool valid(const Options & o)
{
  const double fields[]={o.lookahead, o.activation_distance, o.max_deviation,
    o.max_alignment_shift, o.resolution, o.robot_radius, o.safe_distance,
    o.dynamic_weight, o.guide_weight, o.timeout};
  for (double value : fields) {if (!std::isfinite(value) || value<=0) {return false;}}
  return o.lookahead<=20 && o.max_deviation<=2 && o.max_alignment_shift<=1.5 &&
         o.resolution>=.025 && o.resolution<=.2 && o.robot_radius<=.5 &&
         o.timeout<=.5 && o.max_expansions>0 && o.max_expansions<=200000;
}
bool valid(const Region & r)
{return finite(r.center) && std::isfinite(r.yaw) && std::isfinite(r.length) &&
         std::isfinite(r.width) && r.length>.1 && r.length<=12 && r.width>.4 && r.width<=2;}

std::optional<Window> selectWindow(const std::vector<Point> & reference,
  const std::vector<Region> & regions, const Options & options)
{
  if (!valid(options) || reference.size()<2) {return {};}
  std::vector<double> arc(reference.size(), 0);
  for (std::size_t i=0; i<reference.size(); ++i) {
    if (!finite(reference[i])) {return {};}
    if (i) {arc[i]=arc[i-1]+distance(reference[i], reference[i-1]);}
  }
  std::optional<Window> result;
  double first_entry=std::numeric_limits<double>::infinity();
  for (const auto & region : regions) {
    if (!valid(region)) {continue;}
    const double du=std::max(0., std::abs(longitudinal(reference.front(), region.center, region.yaw))-region.length/2);
    const double dv=std::max(0., std::abs(lateral(reference.front(), region.center, region.yaw))-region.width/2);
    if (std::hypot(du, dv)>options.activation_distance) {continue;}
    for (std::size_t i=0; i<reference.size(); ++i) {
      if (arc[i]>options.lookahead) {break;}
      if (std::abs(longitudinal(reference[i], region.center, region.yaw))>region.length/2+.2 ||
        std::abs(lateral(reference[i], region.center, region.yaw))>region.width/2+.25) {continue;}
      const std::size_t next=std::min(i+1, reference.size()-1), previous=i ? i-1 : i;
      const Point tangent{reference[next].x-reference[previous].x,
        reference[next].y-reference[previous].y};
      const auto a=axis(region.yaw);
      const double along=tangent.x*a.x+tangent.y*a.y;
      if (std::abs(along)<.7*std::hypot(tangent.x, tangent.y) || std::abs(along)<1e-10) {continue;}
      const double sign=along>0 ? 1 : -1;
      std::size_t end=i;
      while (end+1<reference.size() &&
        sign*longitudinal(reference[end], region.center, region.yaw)<
        region.length/2+options.max_alignment_shift+.8) {++end;}
      // Do not reconnect to the old, shifted centreline while still inside a
      // long tunnel merely because a fixed lookahead cut the tunnel in half.
      if (arc[end]>options.lookahead || end<1 || arc[i]>=first_entry) {break;}
      first_entry=arc[i];
      result=Window{region, std::vector<Point>(reference.begin(), reference.begin()+end+1), end};
      break;
    }
  }
  return result;
}

Result refine(const Raster & raster, const Window & window, const Options & options,
  Deadline deadline)
{
  Result out;
  out.reason="invalid_input";
  const auto size=raster.width*raster.height;
  if (!valid(options) || !valid(window.region) || window.reference.size()<2 ||
    !raster.width || !raster.height || raster.width>4000 || raster.height>4000 ||
    size>200000 || raster.signed_distance.size()!=size ||
    raster.reference_distance.size()!=size || raster.occupied.size()!=size ||
    raster.hard.size()!=size) {return out;}
  for (std::size_t i=0; i<size; ++i) {
    if (!std::isfinite(raster.signed_distance[i]) ||
      !std::isfinite(raster.reference_distance[i])) {return out;}
  }
  if (expired(deadline)) {out.reason="budget_exhausted"; return out;}
  if (std::none_of(raster.occupied.begin(), raster.occupied.end(),
    [](unsigned char v) {return v!=0;})) {out.reason="no_dynamic_observations"; return out;}
  if (window.region.length>=1.5) {
    out.guide=fitGuide(raster, window, options, deadline);
    if (!out.guide.accepted) {
      out.reason=expired(deadline) ? "budget_exhausted" : "no_supported_wall_pair";
      return out;
    }
  }
  const auto start=raster.index(window.reference.front()), goal=raster.index(window.reference.back());
  const auto blocked=[&](std::size_t i) {
      return raster.hard[i] || raster.reference_distance[i]>options.max_deviation;
    };
  if (!start || !goal || blocked(*start) || blocked(*goal)) {
    out.reason="endpoint_outside_trust_or_boundary"; return out;
  }
  std::vector<double> costs(size), best(size, std::numeric_limits<double>::infinity());
  std::vector<long> parent(size, -1);
  for (std::size_t i=0; i<size; ++i) {
    costs[i]=1+options.dynamic_weight*softPenalty(options.safe_distance-raster.signed_distance[i]);
    if (out.guide.accepted) {
      const auto p=raster.position(i);
      const auto & g=out.guide;
      if (std::abs(longitudinal(p, g.center, g.yaw))<=g.length/2+.15) {
        const double excess=std::max(0., std::abs(lateral(p, g.center, g.yaw))-
          std::max(.05, g.width/2-options.robot_radius));
        costs[i]+=options.guide_weight*excess*excess;
      }
    }
  }
  struct Entry {double f, g; std::size_t index;};
  const auto compare=[](const Entry & a, const Entry & b) {
      return a.f==b.f ? a.index>b.index : a.f>b.f;
    };
  std::priority_queue<Entry, std::vector<Entry>, decltype(compare)> queue(compare);
  best[*start]=0;
  queue.push({distance(raster.position(*start), raster.position(*goal)), 0, *start});
  bool reached=false;
  while (!queue.empty()) {
    const auto entry=queue.top(); queue.pop();
    if (entry.g>best[entry.index]+1e-10) {continue;}
    if ((out.expansions%64==0 && expired(deadline)) ||
      out.expansions>=options.max_expansions) {out.reason="budget_exhausted"; return out;}
    ++out.expansions;
    if (entry.index==*goal) {reached=true; break;}
    const int x=entry.index%raster.width, y=entry.index/raster.width;
    for (int dy=-1; dy<=1; ++dy) {
      for (int dx=-1; dx<=1; ++dx) {
        if (!dx && !dy) {continue;}
        const int xx=x+dx, yy=y+dy;
        if (xx<0 || yy<0 || xx>=static_cast<int>(raster.width) || yy>=static_cast<int>(raster.height)) {continue;}
        const auto i=static_cast<std::size_t>(yy)*raster.width+xx;
        if (blocked(i) || (dx && dy && (blocked(y*raster.width+xx) || blocked(yy*raster.width+x)))) {continue;}
        const double g=entry.g+std::hypot(dx, dy)*raster.resolution*(costs[entry.index]+costs[i])*.5;
        if (g+1e-10<best[i]) {
          best[i]=g; parent[i]=entry.index;
          queue.push({g+distance(raster.position(i), raster.position(*goal)), g, i});
        }
      }
    }
  }
  if (!reached) {out.reason="no_local_route"; return out;}
  for (long i=*goal; i>=0; i=parent[i]) {out.path.push_back(raster.position(i));}
  std::reverse(out.path.begin(), out.path.end());
  if (out.path.size()<2) {out.reason="degenerate_route"; out.path.clear(); return out;}
  out.path.front()=window.reference.front();
  out.path.back()=window.reference.back();
  // Exact endpoint -> cell-centre links must also respect the hard arena rim.
  for (std::size_t segment=0; segment+1<out.path.size(); ++segment) {
    const auto a=out.path[segment], b=out.path[segment+1];
    const int n=std::max(1, static_cast<int>(std::ceil(distance(a, b)/(.5*raster.resolution))));
    for (int k=0; k<=n; ++k) {
      const auto cell=raster.index({a.x+(b.x-a.x)*k/n, a.y+(b.y-a.y)*k/n});
      if (!cell || blocked(*cell)) {out.reason="invalid_join"; out.path.clear(); return out;}
    }
  }
  if (expired(deadline)) {out.reason="budget_exhausted"; out.path.clear(); return out;}
  out.applied=true; out.reason="applied";
  return out;
}
}  // namespace hero_static_prior_planner::refinement
