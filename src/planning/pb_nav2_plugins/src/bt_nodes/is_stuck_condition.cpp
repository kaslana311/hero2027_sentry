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

#include "pb_nav2_plugins/bt_nodes/is_stuck_condition.hpp"

#include <cmath>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

namespace pb_nav2_plugins
{

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

IsStuckCondition::IsStuckCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), first_tick_(true)
{
  // Obtain the ROS node handle from the BT blackboard (set by bt_navigator)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Read BT port values
  std::string costmap_topic;
  std::string cmd_vel_topic;
  std::string odom_topic;
  getInput("costmap_topic", costmap_topic);
  getInput("check_interval", check_interval_);
  getInput("consecutive_hits", consecutive_hits_);
  getInput("cmd_vel_topic", cmd_vel_topic);
  getInput("odom_topic", odom_topic);
  getInput("command_speed_threshold", command_speed_threshold_);
  getInput("stall_displacement_threshold", stall_displacement_threshold_);
  getInput("stall_duration", stall_duration_);
  getInput("data_timeout", data_timeout_);
  getInput("latch_timeout", latch_timeout_);

  // Set up a dedicated callback group so the subscription callback runs on
  // its own executor thread – identical pattern to Nav2's IsBatteryLow and
  // IsStuckCondition nodes.
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, rclcpp::SensorDataQoS(),
    std::bind(&IsStuckCondition::costmapCallback, this, std::placeholders::_1), sub_options);
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10),
    std::bind(&IsStuckCondition::cmdVelCallback, this, std::placeholders::_1), sub_options);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&IsStuckCondition::odometryCallback, this, std::placeholders::_1), sub_options);

  // Spin the callback group executor in a background thread
  callback_group_executor_thread_ = std::thread([this]() { callback_group_executor_.spin(); });

  RCLCPP_INFO(
    node_->get_logger(), "IsStuckCondition: costmap='%s', cmd='%s', odom='%s', stall=%.2fs/%.3fm",
    costmap_topic.c_str(), cmd_vel_topic.c_str(), odom_topic.c_str(), stall_duration_,
    stall_displacement_threshold_);
}

IsStuckCondition::~IsStuckCondition()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Costmap callback (runs on the executor thread)
// ---------------------------------------------------------------------------

void IsStuckCondition::costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_costmap_ = msg;
}

void IsStuckCondition::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_command_speed_ = std::hypot(msg->linear.x, msg->linear.y);
  latest_cmd_vel_time_ = node_->get_clock()->now();
  have_cmd_vel_ = true;
  updateMotionStallLocked(latest_cmd_vel_time_);
}

void IsStuckCondition::odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_x_ = msg->pose.pose.position.x;
  latest_odom_y_ = msg->pose.pose.position.y;
  latest_odom_time_ = node_->get_clock()->now();
  have_odom_ = true;
  updateMotionStallLocked(latest_odom_time_);
}

void IsStuckCondition::resetMotionWindowLocked() { motion_window_active_ = false; }

void IsStuckCondition::updateMotionStallLocked(const rclcpp::Time & now)
{
  if (!have_cmd_vel_ || !have_odom_ || motion_stall_latched_) {
    return;
  }

  const double cmd_age = (now - latest_cmd_vel_time_).seconds();
  const double odom_age = (now - latest_odom_time_).seconds();
  if (
    cmd_age < 0.0 || odom_age < 0.0 || cmd_age > data_timeout_ || odom_age > data_timeout_ ||
    latest_command_speed_ < command_speed_threshold_) {
    resetMotionWindowLocked();
    return;
  }

  if (!motion_window_active_) {
    motion_window_active_ = true;
    motion_window_start_ = now;
    motion_window_start_x_ = latest_odom_x_;
    motion_window_start_y_ = latest_odom_y_;
    return;
  }

  const double displacement =
    std::hypot(latest_odom_x_ - motion_window_start_x_, latest_odom_y_ - motion_window_start_y_);
  if (displacement >= stall_displacement_threshold_) {
    motion_window_start_ = now;
    motion_window_start_x_ = latest_odom_x_;
    motion_window_start_y_ = latest_odom_y_;
    return;
  }

  const double elapsed = (now - motion_window_start_).seconds();
  if (elapsed >= stall_duration_) {
    motion_stall_latched_ = true;
    motion_stall_latched_time_ = now;
    latched_stall_duration_ = elapsed;
    latched_stall_displacement_ = displacement;
    latched_command_speed_ = latest_command_speed_;
    resetMotionWindowLocked();
  }
}

// ---------------------------------------------------------------------------
// tick()
// ---------------------------------------------------------------------------

BT::NodeStatus IsStuckCondition::tick()
{
  // ---- 1. Consume a recent physical-stall latch, if present ----
  nav2_msgs::msg::Costmap::SharedPtr costmap;
  bool motion_stall = false;
  double stall_elapsed = 0.0;
  double stall_displacement = 0.0;
  double stall_command_speed = 0.0;
  const rclcpp::Time now = node_->get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    costmap = latest_costmap_;
    if (motion_stall_latched_) {
      const double latch_age = (now - motion_stall_latched_time_).seconds();
      if (latch_age >= 0.0 && latch_age <= latch_timeout_) {
        motion_stall = true;
        stall_elapsed = latched_stall_duration_;
        stall_displacement = latched_stall_displacement_;
        stall_command_speed = latched_command_speed_;
      }
      motion_stall_latched_ = false;
    }
  }

  if (motion_stall) {
    RCLCPP_WARN(
      node_->get_logger(),
      "IsStuckCondition: commanded-motion stall confirmed "
      "(cmd=%.3f m/s, displacement=%.3f m in %.2f s)",
      stall_command_speed, stall_displacement, stall_elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  if (!costmap) {
    // No costmap received yet – cannot determine stuck status
    return BT::NodeStatus::FAILURE;
  }

  // ---- 2. Get the robot pose in the costmap frame ----
  // The costmap metadata provides the frame and grid parameters.
  // We use TF to obtain the robot pose in the costmap's frame_id.
  geometry_msgs::msg::TransformStamped transform;
  try {
    auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    transform =
      tf_buffer->lookupTransform(costmap->header.frame_id, "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "IsStuckCondition: TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;

  // ---- 3. Convert world coords to grid indices ----
  const double origin_x = costmap->metadata.origin.position.x;
  const double origin_y = costmap->metadata.origin.position.y;
  const double resolution = costmap->metadata.resolution;
  const unsigned int size_x = costmap->metadata.size_x;
  const unsigned int size_y = costmap->metadata.size_y;

  const int mx = static_cast<int>((robot_x - origin_x) / resolution);
  const int my = static_cast<int>((robot_y - origin_y) / resolution);

  if (
    mx < 0 || my < 0 || static_cast<unsigned int>(mx) >= size_x ||
    static_cast<unsigned int>(my) >= size_y) {
    // Robot is outside the costmap – reset and return FAILURE
    hit_count_ = 0;
    first_tick_ = true;
    return BT::NodeStatus::FAILURE;
  }

  // ---- 4. Read cost at the robot's cell ----
  const unsigned int idx = static_cast<unsigned int>(my) * size_x + static_cast<unsigned int>(mx);
  const uint8_t cost = costmap->data[idx];

  // 253 = INSCRIBED_INFLATED_OBSTACLE
  const bool is_in_obstacle = (cost >= 253);

  // ---- 5. Temporal filtering ----
  if (!is_in_obstacle) {
    // Robot is NOT in an obstacle region – reset the counter
    hit_count_ = 0;
    first_tick_ = true;
    return BT::NodeStatus::FAILURE;
  }

  // Robot IS in an obstacle region (cost >= 253)
  if (first_tick_) {
    // First detection – start the timer
    hit_count_ = 1;
    last_check_time_ = now;
    first_tick_ = false;
    return BT::NodeStatus::FAILURE;
  }

  // Check if enough time has passed since the last counted hit
  const double elapsed = (now - last_check_time_).seconds();
  if (elapsed >= check_interval_) {
    hit_count_++;
    last_check_time_ = now;
  }

  if (hit_count_ >= consecutive_hits_) {
    RCLCPP_WARN(
      node_->get_logger(), "IsStuckCondition: robot confirmed STUCK (cost=%u, hits=%d)", cost,
      hit_count_);
    // Reset so that next time it starts fresh
    hit_count_ = 0;
    first_tick_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace pb_nav2_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb_nav2_plugins::IsStuckCondition>("IsStuckCondition");
}
