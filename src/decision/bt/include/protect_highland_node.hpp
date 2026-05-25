#pragma once

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

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace BT;

/**
 * @brief 高地保护行为节点
 *
 * 专门用于保护登高地的己方机器人：
 *   1. 不断接收要保护的机器人坐标
 *   2. 利用场景特性，从目标机器人坐标开始往后（X轴反向）沿步进一步步搜索
 *   3. 匹配代价地图（Occ_threshold），找到的第一个可行点作为掩护保护位置
 *   4. 如果距离目标保护点大于容忍阈值，下发导航请求；若近，待在原地
 */
class ProtectHighlandNode : public StatefulActionNode {
   public:
    ProtectHighlandNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config) {
        // 获取 ROS 节点
        auto node_result =
            config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!node_result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = node_result;

        // 初始化 Nav2 action client
        nav_client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                node_, "navigate_to_pose");

        // 初始化 TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::string costmap_topic = "/global_costmap/costmap";

        // 创建独立 callback group（不阻塞主线程）
        cb_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        cb_executor_.add_callback_group(cb_group_,
                                        node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        // 订阅 OccupancyGrid costmap
        costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(costmap_mutex_);
                latest_costmap_ = msg;
            },
            sub_opts);

        // 启动 callback executor 线程
        cb_thread_ = std::thread([this]() { cb_executor_.spin(); });

        RCLCPP_INFO(node_->get_logger(),
                    "ProtectHighlandNode: 初始化完成, costmap='%s'",
                    costmap_topic.c_str());
    }

    ~ProtectHighlandNode() override {
        cb_executor_.cancel();
        if (cb_thread_.joinable()) {
            cb_thread_.join();
        }
    }

    static PortsList providedPorts() {
        return {
            InputPort<double>("target_x", "被保护目标的X坐标"),
            InputPort<double>("target_y", "被保护目标的Y坐标"),
            InputPort<int>("occ_threshold", 50, "OccupancyGrid cost 可行阈值"),
            InputPort<double>("nav_update_interval", 1.0,
                              "导航目标最小更新间隔（秒）"),
            InputPort<std::string>("costmap_topic", "/global_costmap/costmap",
                                   "代价地图话题"),
        };
    }

    NodeStatus onStart() override {
        getInput("occ_threshold", occ_threshold_);
        if (!getInput("nav_update_interval", nav_update_interval_)) {
            nav_update_interval_ = 1.0;
        }

        goal_handle_.reset();
        nav_active_ = false;

        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(),
                         "ProtectHighlandNode: Nav2 服务不可用");
            return NodeStatus::FAILURE;
        }

        last_nav_time_ =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

        RCLCPP_INFO(node_->get_logger(),
                    "ProtectHighlandNode: 开始执行高地保护任务");
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        // 节流机制
        double since_last_nav = (node_->now() - last_nav_time_).seconds();
        if (since_last_nav < nav_update_interval_) {
            return NodeStatus::RUNNING;
        }

        double target_protect_x, target_protect_y;
        if (!getInput("target_x", target_protect_x) ||
            !getInput("target_y", target_protect_y)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "ProtectHighlandNode: 缺少目标坐标输入");
            return NodeStatus::RUNNING;
        }

        double self_x, self_y;
        if (!getCurrentPosition(self_x, self_y)) {
            return NodeStatus::RUNNING;
        }

        // 计算保护点
        double protect_pos_x, protect_pos_y;
        if (!computeProtectPoint(target_protect_x, target_protect_y,
                                 protect_pos_x, protect_pos_y)) {
            return NodeStatus::RUNNING;
        }

        // 如果距离目标保护点小于 0.3 米，无需发送导航目标
        double dist_to_target =
            std::hypot(self_x - protect_pos_x, self_y - protect_pos_y);
        if (dist_to_target < 0.3) {
            return NodeStatus::RUNNING;
        }

        // 发送导航目标
        sendNavigationGoal(protect_pos_x, protect_pos_y);
        last_nav_time_ = node_->now();

        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        cancelNavigation();
        RCLCPP_WARN(node_->get_logger(),
                    "ProtectHighlandNode: 高地保护被中断停止");
    }

   private:
    /**
     * @brief 基于目标的坐标，优先沿 X 负分量（后退）寻找第一个代价可行点
     */
    bool computeProtectPoint(double px, double py, double& out_x,
                             double& out_y) {
        nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap = latest_costmap_;
        }

        if (!costmap) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "ProtectHighlandNode: 缺少 costmap");
            return false;
        }

        double max_search_dist = 5.0;  // 限定寻找范围
        double step = 0.2;

        for (double d = 0.0; d <= max_search_dist; d += step) {
            double cx = px - d;
            double cy = py;

            if (isCostmapFree(costmap, cx, cy)) {
                out_x = cx;
                out_y = cy;
                return true;
            }
        }

        return false;
    }

    bool isCostmapFree(const nav_msgs::msg::OccupancyGrid::SharedPtr& costmap,
                       double wx, double wy) const {
        if (!costmap) return false;

        double res = costmap->info.resolution;
        double orig_x = costmap->info.origin.position.x;
        double orig_y = costmap->info.origin.position.y;
        int gw = costmap->info.width;
        int gh = costmap->info.height;

        int mx = static_cast<int>((wx - orig_x) / res);
        int my = static_cast<int>((wy - orig_y) / res);

        if (mx < 0 || mx >= gw || my < 0 || my >= gh) {
            return false;
        }

        int index = my * gw + mx;
        int cost = costmap->data[index];

        return (cost >= 0 && cost <= occ_threshold_);
    }

    bool getCurrentPosition(double& x, double& y) {
        try {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("map", "base_link",
                                            tf2::TimePointZero);
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "ProtectHighlandNode TF Error: %s", ex.what());
            return false;
        }
    }

    void sendNavigationGoal(double x, double y) {
        std::lock_guard<std::mutex> lock(nav_mutex_);

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<
            nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<
                       nav2_msgs::action::NavigateToPose>>
                       handle) {
                if (!handle) {
                    nav_active_ = false;
                }
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::NavigateToPose>::WrappedResult&) {
                std::lock_guard<std::mutex> lock(nav_mutex_);
                nav_active_ = false;
            };

        goal_handle_ =
            nav_client_->async_send_goal(goal_msg, send_goal_options);
        nav_active_ = true;
    }

    void cancelNavigation() {
        std::lock_guard<std::mutex> lock(nav_mutex_);
        if (nav_active_ && goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            nav_active_ = false;
        }
    }

    rclcpp::Node::SharedPtr node_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
        nav_client_;
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    std::mutex nav_mutex_;
    bool nav_active_{false};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    std::mutex costmap_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor cb_executor_;
    std::thread cb_thread_;

    int occ_threshold_{50};
    double nav_update_interval_{1.0};
    rclcpp::Time last_nav_time_;
};
