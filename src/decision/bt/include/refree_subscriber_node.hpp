// referee_subscriber_node.hpp
#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include "bt_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <serial_interfaces/msg/refree.hpp> 
#include "serial_interfaces/msg/radar.hpp"
#include "serial_interfaces/msg/tovision.hpp"


using namespace BT;

// 裁判系统订阅节点
class RefereeSubscriberNode : public StatefulActionNode
{
public:
    RefereeSubscriberNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config)
    {
        // 从blackboard获取ROS2节点
        auto result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!result)
        {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = result;
        
        publisher_ = node_->create_publisher<serial_interfaces::msg::Tovision>("/tovision", 10);
        refree_subscriber_ = node_->create_subscription<serial_interfaces::msg::Refree>("refree_msg", 10,
            [this](const serial_interfaces::msg::Refree::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_msg_ = *msg;
                new_data_available_ = true;
                RCLCPP_INFO(rclcpp::get_logger("refree_sub"),"订阅消息中！！");
                // 发送控制与自瞄模式给与
                // 联盟赛暂定
                // auto message = serial_interfaces::msg::Tovision();
                // message.control_mode = 5;
                // message.sight_mode = 1;
                // message.hit_priority = {0, 0, 0, 0, 0, 0, 0, 0};
                // message.expect_spin_degree = 0;
                // message.dishit = {0, 0, 0, 0, 0, 0, 0, 0};
                // this->publisher_->publish(message);

            });
        RCLCPP_INFO(rclcpp::get_logger("refree_node"),"裁判订阅节点启动");
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("cruise_state", "更新后的巡航状态"),
            OutputPort<int>("competition_type", "比赛类型"),
            OutputPort<int>("competition_stage", "比赛状态"),
            OutputPort<int>("stage_remaining_time", "比赛剩余时间"),
            OutputPort<int>("robot_id", ""),
            OutputPort<int>("allow_bullet_17mm", "允许17mm发弹量"),
            OutputPort<int>("current_hp", "当前血量"),
        };
    }

    NodeStatus onStart() override
    {
        new_data_available_ = false;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (new_data_available_)
        {
            // 更新blackboard数据
            setOutput("competition_type", latest_msg_.competition_type);
            setOutput("competition_stage", latest_msg_.competition_stage);
            setOutput("stage_remaining_time", latest_msg_.stage_time);
            setOutput("robot_id", latest_msg_.robot_id);
            setOutput("allow_bullet_17mm", latest_msg_.bullet_17mm);
            RCLCPP_INFO(rclcpp::get_logger("refree_node"),"17mm:%d",latest_msg_.bullet_17mm);

            setOutput("current_hp", latest_msg_.current_hp);

            new_data_available_ = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<serial_interfaces::msg::Refree>::SharedPtr refree_subscriber_;
    rclcpp::Publisher<serial_interfaces::msg::Tovision>::SharedPtr publisher_;
    serial_interfaces::msg::Refree latest_msg_;
    std::mutex mutex_;
    bool new_data_available_ = false;
};

// 雷达订阅节点
class RadarSubscriberNode : public BT::StatefulActionNode
{
public:
    RadarSubscriberNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config)
    {
        // 从blackboard获取ROS2节点
        auto result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = result;

        // 创建雷达消息订阅器
        radar_subscriber_ = node_->create_subscription<serial_interfaces::msg::Radar>(
            "radar_msg", 10,
            [this](const serial_interfaces::msg::Radar::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_radar_msg_ = *msg;
                new_data_available_ = true;
                RCLCPP_INFO(node_->get_logger(), "收到雷达数据");
            });
    }

    static PortsList providedPorts()
    {
        return {
            OutputPort<double>("robot_1_x", "敌方英雄的X坐标"),
            OutputPort<double>("robot_1_y", "敌方英雄的Y坐标"),
        };
    }

    NodeStatus onStart() override
    {
        new_data_available_ = false;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (new_data_available_)
        {
            // 更新blackboard数据
            setOutput("robot_1_x", latest_radar_msg_.robot_1_position.x);
            setOutput("robot_1_y", latest_radar_msg_.robot_1_position.y);
            new_data_available_ = false;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<serial_interfaces::msg::Radar>::SharedPtr radar_subscriber_;
    serial_interfaces::msg::Radar latest_radar_msg_;
    std::mutex mutex_;
    bool new_data_available_ = false;
};