#include "hero_static_prior_planner/static_prior_planner.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hero_static_prior_planner
{
using lifecycle_msgs::msg::State;

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
  active_ = true;
}

void StaticPriorPlanner::deactivate()
{
  if (active_ && planner_) {
    planner_->deactivate();
  }
  active_ = false;
  if (prior_costmap_ && prior_costmap_->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    prior_costmap_->deactivate();
  }
}

void StaticPriorPlanner::cleanup()
{
  deactivate();
  if (planner_) {
    planner_->cleanup();
    planner_.reset();
  }
  if (prior_costmap_ && prior_costmap_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    prior_costmap_->cleanup();
  }
  costmap_thread_.reset();
  prior_costmap_.reset();
}

nav_msgs::msg::Path StaticPriorPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!active_ || !prior_costmap_->isCurrent()) {
    throw nav2_core::PlannerException(
      "Static prior is not ready; refusing to fall back to the dynamic global costmap");
  }
  return planner_->createPlan(start, goal);
}

}  // namespace hero_static_prior_planner

PLUGINLIB_EXPORT_CLASS(hero_static_prior_planner::StaticPriorPlanner, nav2_core::GlobalPlanner)
