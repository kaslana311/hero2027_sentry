#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

// 包含刚才自定义的NavOutput消息
#include "interfaces/msg/nav_output.hpp"

using namespace BT;

/**
 * @brief 发送目标TF及云台模式的单次同步动作节点
 */
class PublishNavOutputNode : public SyncActionNode {
   public:
    PublishNavOutputNode(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 创建独立的 TF 缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建发布者
        publisher_ = node_->create_publisher<interfaces::msg::NavOutput>(
            "nav_output", 10);
    }

    static PortsList providedPorts() {
        return {InputPort<std::string>("target_tf_name",
                                       "Target TF coordinate frame name"),
                InputPort<int>("mode",
                               "0: Nav, 1: Outpost, 2: Small Energy, 3: Large "
                               "Energy, 4: Disable")};
    }

    NodeStatus tick() override {
        std::string target_tf_name;
        int mode;
        if (!getInput("target_tf_name", target_tf_name) ||
            !getInput("mode", mode)) {
            RCLCPP_ERROR(node_->get_logger(),
                         "PublishNavOutputNode: Missing required inputs!");
            return NodeStatus::FAILURE;
        }

        try {
            // 获取 target_tf_name 在 base_link 下的坐标
            auto transform_stamped = tf_buffer_->lookupTransform(
                "base_link", target_tf_name, tf2::TimePointZero,
                rclcpp::Duration::from_seconds(0.5));

            interfaces::msg::NavOutput msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = "base_link";
            msg.mode = mode;

            msg.point.header = msg.header;
            msg.point.pose.position.x =
                transform_stamped.transform.translation.x;
            msg.point.pose.position.y =
                transform_stamped.transform.translation.y;
            msg.point.pose.position.z =
                transform_stamped.transform.translation.z;
            msg.point.pose.orientation = transform_stamped.transform.rotation;

            publisher_->publish(msg);
            RCLCPP_INFO(node_->get_logger(),
                        "Published NavOutput: tf=%s, mode=%d",
                        target_tf_name.c_str(), mode);

            return NodeStatus::SUCCESS;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(),
                        "PublishNavOutputNode: TF transform failed: %s",
                        ex.what());
            return NodeStatus::FAILURE;
        }
    }

   private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<interfaces::msg::NavOutput>::SharedPtr publisher_;
};

/**
 * @brief 能量机关击打状态机节点（含有20s超时与状态轮询的持续性动作节点）
 */
class RuneStateMachineNode : public StatefulActionNode {
   public:
    RuneStateMachineNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), current_rune_status_(1) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 订阅能量机关状态，假设话题为 RuneStatusSync，数据类型
        // std_msgs::msg::UInt8
        status_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
            "RuneStatusSync", 10,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                current_rune_status_ = msg->data;
            });

        // 发布串口触发指令的话题
        activate_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "RuneActivateCommand", 10);
    }

    static PortsList providedPorts() {
        return {};  // 暂无必要的XML配置端口输入
    }

    NodeStatus onStart() override {
        // 进入打符状态：记录时间
        start_time_ = node_->now();

        // 发布开始打符请求 (activate = true)
        publishActivateCommand(true);
        RCLCPP_INFO(node_->get_logger(),
                    "RuneStateMachineNode Started, triggered activate=true");

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto now = node_->now();
        auto duration = (now - start_time_).seconds();

        // 当状态变为 3：已激活
        if (current_rune_status_ == 3) {
            RCLCPP_INFO(node_->get_logger(), "Rune Activated successfully!");
            publishActivateCommand(false);
            return NodeStatus::SUCCESS;
        }

        // 超过 20s 未达到状态3，即使在激活中也算失败退出
        if (duration > 20.0) {
            RCLCPP_WARN(node_->get_logger(), "Rune Activation TIMEOUT (20s)!");
            publishActivateCommand(false);
            return NodeStatus::FAILURE;
        }

        // 继续循环 (状态还在 1未激活 或 2正在激活)
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // 当节点被如掉血逃跑等高级行为打断时，安全地发出取消指令
        RCLCPP_INFO(node_->get_logger(),
                    "RuneStateMachineNode Halted (interrupted)");
        publishActivateCommand(false);
    }

   private:
    void publishActivateCommand(bool request_activate) {
        std_msgs::msg::Bool msg;
        msg.data = request_activate;
        activate_pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Time start_time_;

    // 内部状态维护
    int current_rune_status_;  // 1：未激活 2：正在激活 3：已激活

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr activate_pub_;
};
