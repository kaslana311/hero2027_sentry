#pragma once

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "nav2_util/node_thread.hpp"

namespace hero_static_prior_planner
{

// Adapt the map input only. Keep Smac's search, parameters, path output and
// planner_server actions unchanged. Loaded exclusively by costmap_mode:=split.
class StaticPriorPlanner : public nav2_core::GlobalPlanner
{
public:
  ~StaticPriorPlanner() override;
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> prior_costmap_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  std::unique_ptr<nav2_smac_planner::SmacPlannerHybrid> planner_;
  bool active_{false};
};

}  // namespace hero_static_prior_planner
