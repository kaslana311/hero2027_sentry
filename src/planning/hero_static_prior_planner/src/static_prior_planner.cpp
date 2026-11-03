#include "hero_static_prior_planner/static_prior_planner.hpp"
#include "hero_static_prior_planner/start_connector.hpp"

#include <cmath>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hero_static_prior_planner
{
using lifecycle_msgs::msg::State;

namespace
{
bool freeAt(
  nav2_costmap_2d::Costmap2D & map, nav2_smac_planner::GridCollisionChecker & checker,
  double x, double y, float yaw_bin, bool allow_unknown)
{
  unsigned int mx, my;
  return std::isfinite(x) && std::isfinite(y) && map.worldToMap(x, y, mx, my) &&
         !checker.inCollision(static_cast<float>(mx), static_cast<float>(my),
           yaw_bin, allow_unknown);
}

bool validRadius(double radius)
{
  return std::isfinite(radius) && radius > 0.0 && radius <= 2.0;
}

// Check actual Smac primitives, not an extra isotropic footprint dilation.
// Two distinct expansions reject the one-step cul-de-sac seen on 2026-10-24,
// while a narrow corridor only needs clearance in a traversable direction.
bool hasLocalDeparture(
  nav2_costmap_2d::Costmap2D & map, nav2_smac_planner::GridCollisionChecker & checker,
  nav2_smac_planner::HybridMotionTable & motions, double x, double y, float yaw_bin)
{
  using Node = nav2_smac_planner::NodeHybrid;
  unsigned int mx, my;
  if (!map.worldToMap(x, y, mx, my)) {return false;}
  Node root(0);
  root.setPose({static_cast<float>(mx), static_cast<float>(my), yaw_bin});
  const auto valid = [&](const nav2_smac_planner::MotionPose & p) {
      return std::isfinite(p._x) && std::isfinite(p._y) && std::isfinite(p._theta) &&
             p._x >= 0 && p._y >= 0 && p._x < map.getSizeInCellsX() &&
             p._y < map.getSizeInCellsY() && p._theta >= 0 &&
             p._theta < motions.num_angle_quantization &&
             !checker.inCollision(p._x, p._y, p._theta, false);
    };
  const auto index = [&](float px, float py, float heading) {
      return Node::getIndex(static_cast<unsigned int>(px), static_cast<unsigned int>(py),
        static_cast<unsigned int>(heading), map.getSizeInCellsX(),
        motions.num_angle_quantization);
    };
  const auto origin = index(root.pose.x, root.pose.y, root.pose.theta);
  for (const auto & first : motions.getProjections(&root)) {
    if (!valid(first)) {continue;}
    const auto first_index = index(first._x, first._y, first._theta);
    if (first_index == origin) {continue;}
    Node next(first_index);
    next.setPose({first._x, first._y, first._theta});
    for (const auto & second : motions.getProjections(&next)) {
      if (!valid(second)) {continue;}
      const auto second_index = index(second._x, second._y, second._theta);
      if (second_index != origin && second_index != first_index) {return true;}
    }
  }
  return false;
}
}  // namespace

StaticPriorPlanner::~StaticPriorPlanner()
{
  // Also release threads after failed configure or shutdown, when the host
  // might not have called our cleanup hook.
  try {
    cleanup();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("StaticPriorPlanner"),
      "Static prior cleanup failed: %s", error.what());
  }
}

void StaticPriorPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  const auto node = parent.lock();
  if (!node || !costmap_ros) {
    throw std::runtime_error("StaticPriorPlanner requires a live planner server");
  }
  parent_ = parent;
  name_ = name;
  dynamic_costmap_ = costmap_ros;
  configureRefinement(node);
  nav2_util::declare_parameter_if_not_declared(
    node, name+".start_recovery_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name+".start_recovery_radius", rclcpp::ParameterValue(0.60));
  if (!validRadius(node->get_parameter(name+".start_recovery_radius").as_double())) {
    throw std::runtime_error("Split start_recovery_radius must be finite and in (0, 2] metres");
  }
  recovery_parameters_ = node->add_on_set_parameters_callback(
    [name](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        if (p.get_name() == name+".start_recovery_radius" &&
          (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
          !validRadius(p.as_double())))
        {
          result.successful = false;
          result.reason = "start_recovery_radius must be finite and in (0, 2] metres";
        }
      }
      return result;
    });
  const std::string parent_namespace = node->get_namespace();
  const std::string prior_topic =
    (parent_namespace == "/" ? "" : parent_namespace) + "/planning_prior/map";
  prior_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "static_prior_costmap", parent_namespace, "static_prior_costmap");

  // A missing/private-namespace YAML must fail, not silently instantiate the
  // default obstacle layer or subscribe to the dynamic /map.
  const auto plugins = prior_costmap_->get_parameter("plugins").as_string_array();
  if (plugins != std::vector<std::string>{"prior_static_layer", "inflation_layer"}) {
    throw std::runtime_error("Missing split static_prior_costmap parameters");
  }
  if (prior_costmap_->configure().id() != State::PRIMARY_STATE_INACTIVE) {
    throw std::runtime_error("Unable to configure independent static prior costmap");
  }
  if (prior_costmap_->getGlobalFrameID() != costmap_ros->getGlobalFrameID() ||
    prior_costmap_->get_parameter("rolling_window").as_bool() ||
    prior_costmap_->get_parameter("prior_static_layer.plugin").as_string() !=
    "nav2_costmap_2d::StaticLayer" ||
    prior_costmap_->get_parameter("prior_static_layer.map_topic").as_string() != prior_topic)
  {
    throw std::runtime_error("Split requires a fixed static prior in the navigation frame");
  }
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(prior_costmap_);
  planner_ = std::make_unique<nav2_smac_planner::SmacPlannerHybrid>();
  // NEVER forward the supplied dynamic global costmap to Smac. Reusing the
  // original plugin name also preserves every GridBased search parameter.
  planner_->configure(parent, std::move(name), std::move(tf), prior_costmap_);
  RCLCPP_INFO(node->get_logger(),
    "SPLIT: Smac uses %s; MINCO keeps global_costmap/costmap_raw", prior_topic.c_str());
}

void StaticPriorPlanner::activate()
{
  if (!planner_ || !prior_costmap_ ||
    prior_costmap_->activate().id() != State::PRIMARY_STATE_ACTIVE)
  {
    throw std::runtime_error("Unable to activate independent static prior costmap");
  }
  planner_->activate();
  prior_plan_pub_->on_activate();
  refined_plan_pub_->on_activate();
  refinement_status_pub_->on_activate();
  active_ = true;
}

void StaticPriorPlanner::deactivate()
{
  if (active_ && planner_) {
    planner_->deactivate();
  }
  active_ = false;
  if (prior_plan_pub_) {prior_plan_pub_->on_deactivate();}
  if (refined_plan_pub_) {refined_plan_pub_->on_deactivate();}
  if (refinement_status_pub_) {refinement_status_pub_->on_deactivate();}
  if (prior_costmap_ && prior_costmap_->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    prior_costmap_->deactivate();
  }
}

void StaticPriorPlanner::cleanup()
{
  deactivate();
  recovery_parameters_.reset();
  if (planner_) {
    planner_->cleanup();
    planner_.reset();
  }
  if (prior_costmap_ && prior_costmap_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    prior_costmap_->cleanup();
  }
  costmap_thread_.reset();
  prior_costmap_.reset();
  dynamic_costmap_.reset();
  prior_plan_pub_.reset();
  refined_plan_pub_.reset();
  refinement_status_pub_.reset();
  tunnel_regions_.clear();
  parent_.reset();
}

nav_msgs::msg::Path StaticPriorPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto prior = createStaticPlan(start, goal);
  const auto node = parent_.lock();
  if (node && node->get_parameter(name_+".local_refinement_enabled").as_bool() &&
    !prior.poses.empty())
  {
    return refinePlan(prior);
  }
  return prior;
}

nav_msgs::msg::Path StaticPriorPlanner::createStaticPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!active_ || !prior_costmap_->isCurrent()) {
    throw nav2_core::PlannerException(
      "Static prior is not ready; refusing to fall back to the dynamic global costmap");
  }
  const auto node = parent_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Split planner parent is unavailable");
  }
  if (!node->get_parameter(name_+".start_recovery_enabled").as_bool()) {
    return planner_->createPlan(start, goal);
  }
  const double radius = node->get_parameter(name_+".start_recovery_radius").as_double();
  const auto bins = node->get_parameter(name_+".angle_quantization_bins").as_int();
  const double yaw = tf2::getYaw(start.pose.orientation);
  if (!validRadius(radius) || bins < 1 || !std::isfinite(yaw)) {
    throw nav2_core::PlannerException("Invalid split start-recovery parameters/pose");
  }
  const float yaw_bin = static_cast<float>(
    std::fmod(std::round(yaw/(2.0*M_PI/bins))+bins, bins));
  auto * prior = prior_costmap_->getCostmap();
  // Match Smac's own pose validity test, including radius/footprint and the
  // configured heading bin. A successful original plan is never replaced.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> prior_lock(*prior->getMutex());
  nav2_smac_planner::GridCollisionChecker prior_checker(prior, bins, node);
  prior_checker.setFootprint(prior_costmap_->getRobotFootprint(),
    prior_costmap_->getUseRadius(), nav2_smac_planner::findCircumscribedCost(prior_costmap_));
  // A corridor may be traversable without room on all eight sides. Lazily
  // build a private Smac primitive table; never modify NodeHybrid's shared
  // search table or run another full-map planner to select an anchor.
  std::unique_ptr<nav2_smac_planner::HybridMotionTable> departure_model;
  const auto prior_free = [&](double x, double y) {
      if (!freeAt(*prior, prior_checker, x, y, yaw_bin, false)) {return false;}
      if (!departure_model) {
        departure_model = std::make_unique<nav2_smac_planner::HybridMotionTable>();
        departure_model->num_angle_quantization = 0;
        departure_model->min_turning_radius = 0;
        nav2_smac_planner::SearchInfo search{};
        search.minimum_turning_radius =
          node->get_parameter(name_+".minimum_turning_radius").as_double()/prior->getResolution();
        auto width = prior->getSizeInCellsX(), height = prior->getSizeInCellsY();
        auto quantizations = static_cast<unsigned int>(bins);
        const auto model = node->get_parameter(name_+".motion_model_for_search").as_string();
        if (model == "DUBIN") {
          departure_model->initDubin(width, height, quantizations, search);
        } else if (model == "REEDS_SHEPP") {
          departure_model->initReedsShepp(width, height, quantizations, search);
        } else {
          throw nav2_core::PlannerException("Split recovery requires a supported Smac motion model");
        }
      }
      return hasLocalDeparture(*prior, prior_checker, *departure_model, x, y, yaw_bin);
    };
  if (freeAt(*prior, prior_checker, start.pose.position.x, start.pose.position.y,
    yaw_bin, node->get_parameter(name_+".allow_unknown").as_bool()))
  {
    prior_lock.unlock();
    auto original = planner_->createPlan(start, goal);
    if (!original.poses.empty()) {
      return original;
    }
    prior_lock.lock();
    if (prior_free(start.pose.position.x, start.pose.position.y)) {
      // A locally manoeuvrable start does not justify bypassing a disconnected
      // static route, an occupied goal, or the original search limits.
      return original;
    }
    RCLCPP_WARN(node->get_logger(),
      "SPLIT start recovery: collision-free start (%.3f, %.3f) has no Smac route "
      "and has no local Smac departure; checking bounded connector",
      start.pose.position.x, start.pose.position.y);
  }
  if (start.header.frame_id != prior_costmap_->getGlobalFrameID() ||
    goal.header.frame_id != prior_costmap_->getGlobalFrameID() ||
    dynamic_costmap_->getGlobalFrameID() != prior_costmap_->getGlobalFrameID() ||
    !dynamic_costmap_->isCurrent())
  {
    throw nav2_core::PlannerException("Split start recovery needs current maps in the same frame");
  }
  auto * dynamic = dynamic_costmap_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> dynamic_lock(*dynamic->getMutex());
  nav2_smac_planner::GridCollisionChecker dynamic_checker(dynamic, bins, node);
  dynamic_checker.setFootprint(dynamic_costmap_->getRobotFootprint(),
    dynamic_costmap_->getUseRadius(), nav2_smac_planner::findCircumscribedCost(dynamic_costmap_));
  const auto dynamic_free = [&](double x, double y) {
      return freeAt(*dynamic, dynamic_checker, x, y, yaw_bin, false);
    };
  const double step = .5*std::min(prior->getResolution(), dynamic->getResolution());
  const detail::Point actual{start.pose.position.x, start.pose.position.y};
  const detail::Grid grid{prior->getSizeInCellsX(), prior->getSizeInCellsY(),
    prior->getResolution(), prior->getOriginX(), prior->getOriginY()};
  const auto connector = detail::findStartConnector(grid, actual,
    {goal.pose.position.x, goal.pose.position.y}, radius, step, prior_free, dynamic_free);
  // Never hold the dynamic map while Smac acquires its own locks/searches.
  dynamic_lock.unlock();
  prior_lock.unlock();
  if (!connector) {
    throw nav2_core::PlannerException(
      "SPLIT start recovery: no locally traversable prior anchor with a dynamic-free connector within " +
      std::to_string(radius) + " m; maps unchanged");
  }
  auto anchor = start;
  anchor.pose.position.x = connector->back().x;
  anchor.pose.position.y = connector->back().y;
  // One recovery search, with unchanged Smac budget, heuristic and static
  // obstacles. A valid-but-trapped start may first have tried its original
  // search; there is no loop over alternative full-map searches.
  auto path = planner_->createPlan(anchor, goal);
  if (path.poses.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "SPLIT start recovery: static anchor (%.3f, %.3f) has no Smac route to goal; maps unchanged",
      anchor.pose.position.x, anchor.pose.position.y);
    return path;
  }
  const auto first = path.poses.front().pose.position;
  if (std::hypot(first.x-anchor.pose.position.x, first.y-anchor.pose.position.y) >
    prior->getResolution()*.01 ||
    std::hypot(first.x-actual.x, first.y-actual.y) > radius+1e-6)
  {
    throw nav2_core::PlannerException("Smac returned a discontinuous split recovery anchor");
  }
  // The map may change during the global search. Recheck the actual returned
  // join before publishing; do not use a stale dynamic-free decision.
  const auto prefix = detail::sampleConnector(actual, {first.x, first.y}, step);
  dynamic_lock.lock();
  if (!dynamic_costmap_->isCurrent() || !detail::connectorIsFree(prefix, dynamic_free)) {
    throw nav2_core::PlannerException("SPLIT start recovery connector became blocked; refusing path");
  }
  dynamic_lock.unlock();
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(prefix.size()-1+path.poses.size());
  for (auto p = prefix.begin(); p != std::prev(prefix.end()); ++p) {
    auto pose = start;
    pose.header = path.header;
    pose.pose.position.x = p->x;
    pose.pose.position.y = p->y;
    poses.push_back(pose);
  }
  poses.insert(poses.end(), path.poses.begin(), path.poses.end());
  path.poses = std::move(poses);
  RCLCPP_WARN(node->get_logger(),
    "SPLIT start recovery: (%.3f, %.3f) -> (%.3f, %.3f), %.3f m dynamic-checked "
    "connector; static prior unchanged",
    actual.x, actual.y, first.x, first.y, std::hypot(first.x-actual.x, first.y-actual.y));
  return path;
}

}  // namespace hero_static_prior_planner

PLUGINLIB_EXPORT_CLASS(hero_static_prior_planner::StaticPriorPlanner, nav2_core::GlobalPlanner)
