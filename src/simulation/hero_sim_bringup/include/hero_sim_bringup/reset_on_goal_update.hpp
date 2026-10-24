// Copyright 2026 HERO contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef HERO_SIM_BRINGUP__RESET_ON_GOAL_UPDATE_HPP_
#define HERO_SIM_BRINGUP__RESET_ON_GOAL_UPDATE_HPP_

#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace hero_sim_bringup
{

// Scope this decorator to the navigation pipeline, not the recovery subtree.
// A new fixed-frame destination must not inherit a completed FollowPath from
// the old destination while its own ComputePathToPose/SmoothPath is pending.
class ResetOnGoalUpdate : public BT::DecoratorNode
{
public:
  ResetOnGoalUpdate(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal")};
  }

  void halt() override
  {
    BT::DecoratorNode::halt();
    have_goal_ = false;
  }

private:
  BT::NodeStatus tick() override
  {
    const auto input = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!input) {
      throw BT::RuntimeError("ResetOnGoalUpdate: missing/invalid goal: ", input.error());
    }
    const auto & goal = input.value();
    // These simulation entries use fixed map-frame destinations. Re-stamping
    // the same frame/pose does not change that destination and must not stop
    // the robot. A changed frame, position OR orientation does reset it.
    if (have_goal_ &&
      (goal.header.frame_id != goal_.header.frame_id || goal.pose != goal_.pose))
    {
      // The normal BT halt contract cancels running action leaves and resets
      // PipelineSequence's progress cursor. It must complete before any new
      // tick; a plain metadata update or an extra retry cannot isolate goals.
      resetChild();
    }
    goal_ = goal;
    have_goal_ = true;
    setStatus(BT::NodeStatus::RUNNING);
    return child_node_->executeTick();
  }

  bool have_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;
};

}  // namespace hero_sim_bringup
#endif  // HERO_SIM_BRINGUP__RESET_ON_GOAL_UPDATE_HPP_
