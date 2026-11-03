#include "hero_static_prior_planner/static_prior_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace hero_static_prior_planner
{
namespace
{
struct Snapshot
{
  unsigned int width{}, height{};
  double resolution{}, x{}, y{};
  std::vector<unsigned char> data;
  unsigned char at(double wx, double wy) const
  {
    const double ix=std::floor((wx-x)/resolution), iy=std::floor((wy-y)/resolution);
    if (ix<0 || iy<0 || ix>=width || iy>=height) {return 255;}
    return data[static_cast<std::size_t>(iy)*width+static_cast<std::size_t>(ix)];
  }
};
Snapshot snapshot(nav2_costmap_2d::Costmap2D & map)
{
  // Snapshot each map separately. Never hold a costmap lock during matching,
  // EDT, A*, Smac, publishing, or acquisition of another map's lock.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  Snapshot out{map.getSizeInCellsX(), map.getSizeInCellsY(), map.getResolution(),
    map.getOriginX(), map.getOriginY(), {}};
  if (!out.width || !out.height || out.resolution<=0 || !std::isfinite(out.resolution)) {
    throw std::runtime_error("invalid costmap geometry");
  }
  out.data.assign(map.getCharMap(), map.getCharMap()+static_cast<std::size_t>(out.width)*out.height);
  return out;
}

refinement::Raster makeRaster(const Snapshot & dynamic, const Snapshot & boundary,
  const refinement::Window & window, const refinement::Options & options)
{
  using refinement::Point;
  double xmin=std::numeric_limits<double>::infinity(), ymin=xmin;
  double xmax=-xmin, ymax=-xmin;
  for (const auto p : window.reference) {
    xmin=std::min(xmin, p.x); xmax=std::max(xmax, p.x);
    ymin=std::min(ymin, p.y); ymax=std::max(ymax, p.y);
  }
  const double margin=options.max_deviation+.3;
  const double resolution=std::max(options.resolution, dynamic.resolution);
  xmin=std::max(dynamic.x, xmin-margin); ymin=std::max(dynamic.y, ymin-margin);
  xmax=std::min(dynamic.x+dynamic.width*dynamic.resolution, xmax+margin);
  ymax=std::min(dynamic.y+dynamic.height*dynamic.resolution, ymax+margin);
  xmin=dynamic.x+std::floor((xmin-dynamic.x)/resolution)*resolution;
  ymin=dynamic.y+std::floor((ymin-dynamic.y)/resolution)*resolution;
  refinement::Raster raster;
  raster.origin_x=xmin; raster.origin_y=ymin; raster.resolution=resolution;
  raster.width=static_cast<std::size_t>(std::ceil((xmax-xmin)/resolution));
  raster.height=static_cast<std::size_t>(std::ceil((ymax-ymin)/resolution));
  const auto size=raster.width*raster.height;
  if (!raster.width || !raster.height || raster.width>4000 || raster.height>4000 || size>200000) {
    throw std::runtime_error("local ROI exceeds cell budget");
  }
  raster.occupied.assign(size, 0); raster.hard.assign(size, 0);
  cv::Mat obstacles(raster.height, raster.width, CV_8UC1, cv::Scalar(255));
  cv::Mat boundary_free=obstacles.clone(), reference=obstacles.clone();
  // Max-pool original cells, rather than point-sampling: no thin wall or rim
  // may disappear when the search grid is coarser than the 2.5-cm Nav2 map.
  const auto subsamples=std::max(1, static_cast<int>(std::ceil(resolution/dynamic.resolution)));
  for (std::size_t i=0; i<size; ++i) {
    const auto p=raster.position(i);
    const int xx=i%raster.width, yy=i/raster.width;
    for (int sy=0; sy<subsamples; ++sy) {
      for (int sx=0; sx<subsamples; ++sx) {
        const double wx=p.x-resolution/2+(sx+.5)*resolution/subsamples;
        const double wy=p.y-resolution/2+(sy+.5)*resolution/subsamples;
        const auto cost=dynamic.at(wx, wy), rim=boundary.at(wx, wy);
        if (cost==255) {raster.hard[i]=1;}
        if (rim>=253) {boundary_free.at<unsigned char>(yy, xx)=0;}
        if (cost>=253 && cost<255) {obstacles.at<unsigned char>(yy, xx)=0;}
        if (cost==254 && rim<253) {raster.occupied[i]=1;}
      }
    }
  }
  cv::Mat outside, inside, inverse, rim_distance, reference_distance;
  cv::distanceTransform(obstacles, outside, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  cv::bitwise_not(obstacles, inverse);
  cv::distanceTransform(inverse, inside, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  cv::distanceTransform(boundary_free, rim_distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  for (std::size_t k=1; k<window.reference.size(); ++k) {
    const auto a=raster.index(window.reference[k-1]), b=raster.index(window.reference[k]);
    if (!a || !b) {throw std::runtime_error("reference endpoint outside map");}
    cv::line(reference, cv::Point(*a%raster.width, *a/raster.width),
      cv::Point(*b%raster.width, *b/raster.width), cv::Scalar(0));
  }
  cv::distanceTransform(reference, reference_distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  raster.signed_distance.resize(size); raster.reference_distance.resize(size);
  for (std::size_t i=0; i<size; ++i) {
    const int xx=i%raster.width, yy=i/raster.width;
    const double out=outside.at<float>(yy, xx), in=inside.at<float>(yy, xx);
    raster.signed_distance[i]=std::clamp((out>0 ? out : -in)*resolution, -3., 3.);
    raster.reference_distance[i]=reference_distance.at<float>(yy, xx)*resolution;
    if (rim_distance.at<float>(yy, xx)*resolution<=options.robot_radius+.5*resolution) {
      raster.hard[i]=1;
      raster.occupied[i]=0;
    }
  }
  return raster;
}
}  // namespace

void StaticPriorPlanner::configureRefinement(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  const auto declare=[&](const std::string & suffix, rclcpp::ParameterValue value, bool read_only) {
      if (!node->has_parameter(name_+suffix)) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only=read_only;
        node->declare_parameter(name_+suffix, value, descriptor);
      }
    };
  declare(".local_refinement_enabled", rclcpp::ParameterValue(false), false);
  const auto number=[&](const std::string & suffix, double & value) {
      declare(".local_refinement_"+suffix, rclcpp::ParameterValue(value), true);
      value=node->get_parameter(name_+".local_refinement_"+suffix).as_double();
    };
  number("lookahead", refinement_options_.lookahead);
  number("activation_distance", refinement_options_.activation_distance);
  number("max_deviation", refinement_options_.max_deviation);
  number("max_alignment_shift", refinement_options_.max_alignment_shift);
  number("resolution", refinement_options_.resolution);
  number("robot_radius", refinement_options_.robot_radius);
  number("safe_distance", refinement_options_.safe_distance);
  number("dynamic_weight", refinement_options_.dynamic_weight);
  number("guide_weight", refinement_options_.guide_weight);
  number("timeout", refinement_options_.timeout);
  declare(".local_refinement_max_expansions", rclcpp::ParameterValue(int64_t(50000)), true);
  const auto expansions=node->get_parameter(name_+".local_refinement_max_expansions").as_int();
  if (expansions<1) {throw std::runtime_error("local refinement expansion budget must be positive");}
  refinement_options_.max_expansions=expansions;
  if (!refinement::valid(refinement_options_)) {throw std::runtime_error("invalid local refinement options");}
  declare(".local_refinement_regions", rclcpp::ParameterValue(std::vector<double>{}), true);
  const auto flat=node->get_parameter(name_+".local_refinement_regions").as_double_array();
  if (flat.size()%5) {throw std::runtime_error("local refinement regions require x,y,yaw,length,width groups");}
  tunnel_regions_.clear();
  for (std::size_t i=0; i<flat.size(); i+=5) {
    refinement::Region region{{flat[i], flat[i+1]}, flat[i+2], flat[i+3], flat[i+4], i/5};
    if (!refinement::valid(region)) {throw std::runtime_error("invalid tunnel refinement region");}
    tunnel_regions_.push_back(region);
  }
  if (node->get_parameter(name_+".local_refinement_enabled").as_bool() && tunnel_regions_.empty()) {
    throw std::runtime_error("local refinement enabled without tunnel regions");
  }
  prior_plan_pub_=node->create_publisher<nav_msgs::msg::Path>(name_+"/static_plan", 1);
  refined_plan_pub_=node->create_publisher<nav_msgs::msg::Path>(name_+"/refined_plan", 1);
  refinement_status_pub_=node->create_publisher<std_msgs::msg::String>(name_+"/refinement_status", 1);
}

nav_msgs::msg::Path StaticPriorPlanner::refinePlan(const nav_msgs::msg::Path & prior)
{
  const auto node=parent_.lock();
  if (!node || !refinement_status_pub_ || !refinement_status_pub_->is_activated()) {return prior;}
  const auto begin=std::chrono::steady_clock::now();
  const auto deadline=begin+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(refinement_options_.timeout));
  auto output=prior;
  std::string reason="outside_tunnel_regions";
  refinement::Result result;
  std::optional<refinement::Window> window;
  try {
    std::vector<refinement::Point> points;
    for (const auto & pose : prior.poses) {points.push_back({pose.pose.position.x, pose.pose.position.y});}
    window=refinement::selectWindow(points, tunnel_regions_, refinement_options_);
    if (window) {
      reason="dynamic_map_not_current";
      if (dynamic_costmap_ && dynamic_costmap_->isCurrent() &&
        prior.header.frame_id==dynamic_costmap_->getGlobalFrameID())
      {
        std::shared_ptr<nav2_costmap_2d::CostmapLayer> boundary;
        for (const auto & layer : *dynamic_costmap_->getLayeredCostmap()->getPlugins()) {
          const auto name=layer->getName();
          if (name.size()>=14 && name.compare(name.size()-14, 14, "boundary_layer")==0) {
            boundary=std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(layer);
          }
        }
        reason="boundary_layer_unavailable";
        if (boundary) {
          auto options=refinement_options_;
          options.robot_radius=std::max(options.robot_radius,
            dynamic_costmap_->getLayeredCostmap()->getInscribedRadius());
          const auto live=snapshot(*dynamic_costmap_->getCostmap());
          const auto rim=snapshot(*boundary);
          const auto raster=makeRaster(live, rim, *window, options);
          result=refinement::refine(raster, *window, options, deadline);
          reason=result.reason;
          if (result.applied) {
            std::vector<geometry_msgs::msg::PoseStamped> poses;
            poses.reserve(result.path.size()+prior.poses.size()-window->join_index-1);
            for (std::size_t i=0; i<result.path.size(); ++i) {
              auto pose=prior.poses.front(); pose.header=prior.header;
              pose.pose.position.x=result.path[i].x; pose.pose.position.y=result.path[i].y;
              if (i+1<result.path.size()) {
                const double yaw=std::atan2(result.path[i+1].y-result.path[i].y,
                  result.path[i+1].x-result.path[i].x);
                pose.pose.orientation.x=0; pose.pose.orientation.y=0;
                pose.pose.orientation.z=std::sin(yaw/2); pose.pose.orientation.w=std::cos(yaw/2);
              } else {pose=prior.poses[window->join_index];}
              poses.push_back(pose);
            }
            poses.front()=prior.poses.front();
            poses.insert(poses.end(), prior.poses.begin()+window->join_index+1, prior.poses.end());
            output.poses=std::move(poses);
          }
        }
      }
    }
  } catch (const std::exception & error) {
    reason=std::string("fallback_exception: ")+error.what();
    output=prior;
  }
  const double ms=std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::ostringstream detail;
  detail << "result=" << reason << " region=" << (window ? int(window->region.id) : -1)
         << " guide=" << result.guide.accepted << " expansions=" << result.expansions
         << " ms=" << ms << " soft_dynamic=true";
  if (result.guide.accepted) {
    detail << " width=" << result.guide.width << " overlap=" << result.guide.overlap_bins;
  }
  std_msgs::msg::String status; status.data=detail.str();
  prior_plan_pub_->publish(prior);
  refined_plan_pub_->publish(output);
  refinement_status_pub_->publish(status);
  RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
    "SPLIT local refinement: %s", status.data.c_str());
  return output;
}
}  // namespace hero_static_prior_planner
