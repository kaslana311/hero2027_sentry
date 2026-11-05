// Copyright 2026 Jinbo Liu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_NAV2_PLUGINS__BT_NODES__IS_STUCK_CONDITION_HPP_
#define PB_NAV2_PLUGINS__BT_NODES__IS_STUCK_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb_nav2_plugins
{

/**
 * @brief BT condition node that checks for either a physical motion stall or
 *        an obstacle overlap on the local costmap.
 *
 * A physical stall is latched when a translational command stays above
 * `command_speed_threshold` for `stall_duration` while odometry displacement
 * remains below `stall_displacement_threshold`. The original costmap-based
 * test is retained as an independent second trigger.
 *
 * This filtering prevents momentary lidar noise or brief contact from
 * triggering a full escape recovery.
 *
 * Returns SUCCESS when the robot is confirmed stuck, FAILURE otherwise.
 */
class IsStuckCondition : public BT::ConditionNode
{
public:
  IsStuckCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsStuckCondition() = delete;

  ~IsStuckCondition() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "costmap_topic", "local_costmap/costmap_raw", "Raw costmap topic to subscribe to"),
      BT::InputPort<double>(
        "check_interval", 0.2, "Minimum seconds between consecutive stuck checks"),
      BT::InputPort<int>(
        "consecutive_hits", 3, "Number of consecutive high-cost checks required to confirm stuck"),
      BT::InputPort<std::string>(
        "cmd_vel_topic", "cmd_vel_nav2_result", "Commanded velocity topic to monitor"),
      BT::InputPort<std::string>(
        "odom_topic", "odometry/filtered", "Odometry topic used to measure actual motion"),
      BT::InputPort<double>(
        "command_speed_threshold", 0.08, "Minimum translational command speed for stall detection"),
      BT::InputPort<double>(
        "stall_displacement_threshold", 0.03,
        "Minimum odometry displacement expected during the stall window"),
      BT::InputPort<double>(
        "stall_duration", 1.5, "Seconds of command without enough displacement to confirm a stall"),
      BT::InputPort<double>(
        "data_timeout", 0.5, "Maximum age of command and odometry samples in seconds"),
      BT::InputPort<double>(
        "latch_timeout", 30.0, "Maximum age of a latched motion stall in seconds"),
      BT::InputPort<std::string>(
        "motion_stall_topic", "", "Optional live physical-stall signal; empty preserves legacy behavior"),
    };
  }

private:
  void costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);
  void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void updateMotionStallLocked(const rclcpp::Time & now);
  void resetMotionWindowLocked();
  void publishMotionStateLocked(const rclcpp::Time & now, bool force = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_stall_pub_;

  // Latest costmap (guarded by mutex_)
  std::mutex mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Parameters (read once from BT ports)
  double check_interval_{0.2};
  int consecutive_hits_{3};
  double command_speed_threshold_{0.08};
  double stall_displacement_threshold_{0.03};
  double stall_duration_{1.5};
  double data_timeout_{0.5};
  double latch_timeout_{30.0};

  // Motion-stall state (guarded by mutex_)
  bool have_cmd_vel_{false};
  bool have_odom_{false};
  double latest_command_speed_{0.0};
  double latest_odom_x_{0.0};
  double latest_odom_y_{0.0};
  rclcpp::Time latest_cmd_vel_time_;
  rclcpp::Time latest_odom_time_;
  bool motion_window_active_{false};
  rclcpp::Time motion_window_start_;
  double motion_window_start_x_{0.0};
  double motion_window_start_y_{0.0};
  bool motion_stall_latched_{false};
  rclcpp::Time motion_stall_latched_time_;
  double latched_stall_duration_{0.0};
  double latched_stall_displacement_{0.0};
  double latched_command_speed_{0.0};
  // Independent of the BT-consumed latch. Zero commands during cancellation
  // must not release the inhibition; actual displacement does.
  bool warm_start_inhibited_{false};
  double inhibited_at_x_{0.0}, inhibited_at_y_{0.0};
  double last_signal_time_{-1.0};

  // Filtering state
  int hit_count_{0};
  rclcpp::Time last_check_time_;
  bool first_tick_{true};
};

}  // namespace pb_nav2_plugins

#endif  // PB_NAV2_PLUGINS__BT_NODES__IS_STUCK_CONDITION_HPP_
