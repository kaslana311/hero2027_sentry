#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

using namespace BT;

/**
 * @brief 打击可行性分析节点
 *
 * 该节点通过曼哈顿距离和敌方血量百分比来计算打击的可行性。
 * 公式：可行性 = 距离权重 * (1 - 标准化曼哈顿距离) + 血量权重 * (1 -
 * 敌方血量百分比)
 *
 * 距离越近，可行性越高
 * 敌方血量越低，可行性越高
 */
class FeasibilityCalculateNode : public ConditionNode {
   public:
    FeasibilityCalculateNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return {// 输入端口
                InputPort<double>("current_x", "当前机器人X坐标"),
                InputPort<double>("current_y", "当前机器人Y坐标"),
                InputPort<double>("enemy_x", "敌方机器人X坐标"),
                InputPort<double>("enemy_y", "敌方机器人Y坐标"),
                InputPort<int>("enemy_hp", "敌方机器人血量"),
                InputPort<int>("enemy_max_hp", "敌方机器人最大血量，默认500"),
                InputPort<double>("distance_weight", "距离权重"),
                InputPort<double>("hp_weight", "血量权重"),
                InputPort<double>("max_distance",
                                  "最大考虑距离，超过此距离视为不可行"),
                InputPort<double>("feasibility_threshold",
                                  "可行性阈值，高于此值视为可行"),

                // 输出端口 - 可以将计算结果输出
                OutputPort<double>("feasibility_score", "计算出的可行性分数")};
    }

    NodeStatus tick() override {
        // 获取输入参数
        double current_x, current_y, enemy_x, enemy_y;
        if (!getInput("current_x", current_x) ||
            !getInput("current_y", current_y) ||
            !getInput("enemy_x", enemy_x) || !getInput("enemy_y", enemy_y)) {
            RCLCPP_ERROR(rclcpp::get_logger("feasibility_calculate"),
                         "缺少坐标输入参数");
            return NodeStatus::FAILURE;
        }

        int enemy_hp = 0, enemy_max_hp = 500;
        if (!getInput("enemy_hp", enemy_hp)) {
            RCLCPP_ERROR(rclcpp::get_logger("feasibility_calculate"),
                         "缺少敌方血量参数");
            return NodeStatus::FAILURE;
        }
        getInput("enemy_max_hp", enemy_max_hp);  // 可选参数，默认500

        // 获取权重和阈值参数，设置默认值
        double distance_weight = 0.6;
        double hp_weight = 0.4;
        double max_distance = 10.0;
        double feasibility_threshold = 0.5;

        getInput("distance_weight", distance_weight);
        getInput("hp_weight", hp_weight);
        getInput("max_distance", max_distance);
        getInput("feasibility_threshold", feasibility_threshold);

        // 计算曼哈顿距离
        double manhattan_distance =
            std::abs(current_x - enemy_x) + std::abs(current_y - enemy_y);

        // 如果超出最大距离，直接判定为不可行
        if (manhattan_distance > max_distance) {
            setOutput("feasibility_score", 0.0);
            RCLCPP_DEBUG(rclcpp::get_logger("feasibility_calculate"),
                         "目标距离过远 (%.2f > %.2f)，判定为不可行",
                         manhattan_distance, max_distance);
            return NodeStatus::FAILURE;
        }

        // 归一化距离 (0-1范围，0表示距离为0，1表示距离为max_distance)
        double normalized_distance = manhattan_distance / max_distance;

        // 计算敌方血量比例 (0-1范围，0表示血量为0，1表示血量满)
        double enemy_hp_ratio =
            static_cast<double>(enemy_hp) / static_cast<double>(enemy_max_hp);

        // 计算可行性分数 (距离越近、敌方血量越低，分数越高)
        double feasibility_score =
            distance_weight * (1.0 - normalized_distance) +
            hp_weight * (1.0 - enemy_hp_ratio);

        // 设置输出值
        setOutput("feasibility_score", feasibility_score);

        RCLCPP_DEBUG(rclcpp::get_logger("feasibility_calculate"),
                     "可行性分析: 距离=%.2f, 敌方血量比例=%.2f, "
                     "可行性分数=%.2f (阈值=%.2f)",
                     manhattan_distance, enemy_hp_ratio, feasibility_score,
                     feasibility_threshold);

        // 根据阈值决定是否可行
        if (feasibility_score >= feasibility_threshold) {
            return NodeStatus::SUCCESS;  // 可行
        } else {
            return NodeStatus::FAILURE;  // 不可行
        }
    }
};

/**
 * @brief 基于Navigation2实际路径长度的打击可行性分析节点
 *
 * 该节点通过Nav2路径规划获取实际路径长度和敌方血量百分比来计算打击的可行性。
 * 公式：可行性 = 距离权重 * (1 - 标准化路径长度) + 血量权重 * (1 -
 * 敌方血量百分比)
 *
 * 距离越近，可行性越高
 * 敌方血量越低，可行性越高
 */
class NavPathFeasibilityNode : public ConditionNode {
   public:
    NavPathFeasibilityNode(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config), path_computing_(false) {
        // 从blackboard获取ROS2节点
        auto result = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        if (!result) {
            throw RuntimeError("Missing ROS2 node in blackboard");
        }
        node_ = result;

        // 创建Nav2计算路径的Action客户端
        compute_path_client_ =
            rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
                node_, "compute_path_to_pose");

        // 等待服务可用
        if (!compute_path_client_->wait_for_action_server(
                std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(),
                        "ComputePathToPose action "
                        "server不可用，使用曼哈顿距离作为备选方案");
            use_manhattan_distance_ = true;
        } else {
            use_manhattan_distance_ = false;
        }
    }

    static PortsList providedPorts() {
        return {// 输入端口
                InputPort<double>("current_x", "当前机器人X坐标"),
                InputPort<double>("current_y", "当前机器人Y坐标"),
                InputPort<double>("enemy_x", "敌方机器人X坐标"),
                InputPort<double>("enemy_y", "敌方机器人Y坐标"),
                InputPort<int>("enemy_hp", "敌方机器人血量"),
                InputPort<int>("enemy_max_hp", "敌方机器人最大血量，默认500"),
                InputPort<double>("distance_weight", "距离权重"),
                InputPort<double>("hp_weight", "血量权重"),
                InputPort<double>("max_distance",
                                  "最大考虑距离，超过此距离视为不可行"),
                InputPort<double>("feasibility_threshold",
                                  "可行性阈值，高于此值视为可行"),
                InputPort<std::string>("robot_frame",
                                       "机器人坐标系名称，默认base_link"),
                InputPort<std::string>("map_frame", "地图坐标系名称，默认map"),

                // 输出端口 - 可以将计算结果输出
                OutputPort<double>("feasibility_score", "计算出的可行性分数"),
                OutputPort<double>("path_length", "计算出的路径长度")};
    }

    NodeStatus tick() override {
        // 获取输入参数
        double current_x, current_y, enemy_x, enemy_y;
        if (!getInput("current_x", current_x) ||
            !getInput("current_y", current_y) ||
            !getInput("enemy_x", enemy_x) || !getInput("enemy_y", enemy_y)) {
            RCLCPP_ERROR(node_->get_logger(), "缺少坐标输入参数");
            return NodeStatus::FAILURE;
        }

        int enemy_hp = 0, enemy_max_hp = 500;
        if (!getInput("enemy_hp", enemy_hp)) {
            RCLCPP_ERROR(node_->get_logger(), "缺少敌方血量参数");
            return NodeStatus::FAILURE;
        }
        getInput("enemy_max_hp", enemy_max_hp);  // 可选参数，默认500

        // 获取权重和阈值参数，设置默认值
        double distance_weight = 0.6;
        double hp_weight = 0.4;
        double max_distance = 10.0;
        double feasibility_threshold = 0.5;

        getInput("distance_weight", distance_weight);
        getInput("hp_weight", hp_weight);
        getInput("max_distance", max_distance);
        getInput("feasibility_threshold", feasibility_threshold);

        // 获取坐标系信息
        std::string robot_frame = "base_link";
        std::string map_frame = "map";
        getInput("robot_frame", robot_frame);
        getInput("map_frame", map_frame);

        // 获取路径长度
        double path_length;
        if (use_manhattan_distance_) {
            // 备选：使用曼哈顿距离
            path_length =
                std::abs(current_x - enemy_x) + std::abs(current_y - enemy_y);
        } else {
            // 使用Nav2路径规划获取实际路径长度
            path_length = computePathLength(current_x, current_y, enemy_x,
                                            enemy_y, map_frame);
        }

        // 设置路径长度输出
        setOutput("path_length", path_length);

        // 如果超出最大距离，直接判定为不可行
        if (path_length > max_distance) {
            setOutput("feasibility_score", 0.0);
            RCLCPP_DEBUG(node_->get_logger(),
                         "目标路径过长 (%.2f > %.2f)，判定为不可行",
                         path_length, max_distance);
            return NodeStatus::FAILURE;
        }

        // 归一化距离 (0-1范围，0表示距离为0，1表示距离为max_distance)
        double normalized_distance = path_length / max_distance;

        // 计算敌方血量比例 (0-1范围，0表示血量为0，1表示血量满)
        double enemy_hp_ratio =
            static_cast<double>(enemy_hp) / static_cast<double>(enemy_max_hp);

        // 计算可行性分数 (距离越近、敌方血量越低，分数越高)
        double feasibility_score =
            distance_weight * (1.0 - normalized_distance) +
            hp_weight * (1.0 - enemy_hp_ratio);

        // 设置输出值
        setOutput("feasibility_score", feasibility_score);

        RCLCPP_DEBUG(node_->get_logger(),
                     "可行性分析: 路径长度=%.2f, 敌方血量比例=%.2f, "
                     "可行性分数=%.2f (阈值=%.2f)",
                     path_length, enemy_hp_ratio, feasibility_score,
                     feasibility_threshold);

        // 根据阈值决定是否可行
        if (feasibility_score >= feasibility_threshold) {
            return NodeStatus::SUCCESS;  // 可行
        } else {
            return NodeStatus::FAILURE;  // 不可行
        }
    }

   private:
    // 计算从当前位置到敌方位置的路径长度
    double computePathLength(double start_x, double start_y, double goal_x,
                             double goal_y, const std::string& frame_id) {
        // 如果已经有计算正在进行，使用曼哈顿距离作为临时替代
        if (path_computing_) {
            return std::abs(start_x - goal_x) + std::abs(start_y - goal_y);
        }

        // 创建起点和终点
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = frame_id;
        start_pose.header.stamp = node_->now();
        start_pose.pose.position.x = start_x;
        start_pose.pose.position.y = start_y;
        start_pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // 使用零偏航角
        start_pose.pose.orientation.x = q.x();
        start_pose.pose.orientation.y = q.y();
        start_pose.pose.orientation.z = q.z();
        start_pose.pose.orientation.w = q.w();

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = frame_id;
        goal_pose.header.stamp = node_->now();
        goal_pose.pose.position.x = goal_x;
        goal_pose.pose.position.y = goal_y;
        goal_pose.pose.position.z = 0.0;

        goal_pose.pose.orientation =
            start_pose.pose.orientation;  // 使用相同的朝向

        // 创建并发送计算路径请求
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.start = start_pose;
        goal_msg.goal = goal_pose;
        goal_msg.use_start = true;

        // 设置正在计算标志
        path_computing_ = true;

        // 设置发送计算请求的超时时间
        auto send_goal_options = rclcpp_action::Client<
            nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::ComputePathToPose>::SharedPtr&
                       goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(node_->get_logger(), "计算路径请求被拒绝");
                }
            };

        send_goal_options.feedback_callback =
            [](rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::ComputePathToPose>::SharedPtr,
               const std::shared_ptr<
                   const nav2_msgs::action::ComputePathToPose::Feedback>) {};

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                   nav2_msgs::action::ComputePathToPose>::WrappedResult&
                       result) {
                path_computing_ = false;

                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "计算路径失败，错误码: %d",
                                 static_cast<int>(result.code));
                    return;
                }

                // 计算路径长度
                double length = 0.0;
                const auto& path = result.result->path;

                // 如果路径为空或只有一个点，返回0
                if (path.poses.size() <= 1) {
                    {
                        std::lock_guard<std::mutex> lock(path_mutex_);
                        last_path_length_ = 0.0;
                    }
                    return;
                }

                // 计算路径中相邻点之间的欧氏距离总和
                for (size_t i = 1; i < path.poses.size(); ++i) {
                    const auto& prev = path.poses[i - 1].pose.position;
                    const auto& curr = path.poses[i].pose.position;

                    double dx = curr.x - prev.x;
                    double dy = curr.y - prev.y;
                    double segment_length = std::sqrt(dx * dx + dy * dy);

                    length += segment_length;
                }

                // 存储计算结果
                {
                    std::lock_guard<std::mutex> lock(path_mutex_);
                    last_path_length_ = length;
                }

                RCLCPP_DEBUG(node_->get_logger(), "计算路径长度: %.2f 米",
                             length);
            };

        // 发送计算路径请求
        auto goal_handle_future =
            compute_path_client_->async_send_goal(goal_msg, send_goal_options);

        // 获取上次计算的路径长度
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            return last_path_length_ > 0.0 ? last_path_length_
                                           : std::abs(start_x - goal_x) +
                                                 std::abs(start_y - goal_y);
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr
        compute_path_client_;

    std::mutex path_mutex_;
    double last_path_length_ = 0.0;
    bool path_computing_;
    bool use_manhattan_distance_;
};

/**
 * @brief 四边形内点检查节点
 *
 * 该节点检查一个点是否在给定四边形内。
 * 四边形由四个顶点坐标定义，点的坐标作为输入。
 */
class PointInQuadrilateralCondition : public ConditionNode {
   public:
    PointInQuadrilateralCondition(const std::string& name,
                                  const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return {InputPort<double>("p1_x", "四边形第一个顶点X坐标"),
                InputPort<double>("p1_y", "四边形第一个顶点Y坐标"),
                InputPort<double>("p2_x", "四边形第二个顶点X坐标"),
                InputPort<double>("p2_y", "四边形第二个顶点Y坐标"),
                InputPort<double>("p3_x", "四边形第三个顶点X坐标"),
                InputPort<double>("p3_y", "四边形第三个顶点Y坐标"),
                InputPort<double>("p4_x", "四边形第四个顶点X坐标"),
                InputPort<double>("p4_y", "四边形第四个顶点Y坐标"),
                InputPort<double>("target_x", "目标对象X坐标"),
                InputPort<double>("target_y", "目标对象Y坐标")};
    }

    NodeStatus tick() override {
        // 获取四边形四个顶点坐标
        float p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y;
        float target_x, target_y;

        if (!getInput("p1_x", p1_x) || !getInput("p1_y", p1_y) ||
            !getInput("p2_x", p2_x) || !getInput("p2_y", p2_y) ||
            !getInput("p3_x", p3_x) || !getInput("p3_y", p3_y) ||
            !getInput("p4_x", p4_x) || !getInput("p4_y", p4_y) ||
            !getInput("target_x", target_x) ||
            !getInput("target_y", target_y)) {
            std::cout << "检查错误: 缺少输入坐标值" << std::endl;
            return NodeStatus::FAILURE;
        }

        // 执行点在四边形内的判断
        bool result = isPointInQuadrilateral(
            target_x, target_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, p4_x, p4_y);

        std::cout << "点位置检查: 目标点(" << target_x << "," << target_y << ")"
                  << (result ? " 在四边形内" : " 在四边形外") << std::endl;

        return result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

   private:
    struct Point {
        float x, y;
        Point(float x = 0, float y = 0) : x(x), y(y) {}
    };

    // 判断点是否在四边形内部
    bool isPointInQuadrilateral(float px, float py, float p1_x, float p1_y,
                                float p2_x, float p2_y, float p3_x, float p3_y,
                                float p4_x, float p4_y) {
        // 构建四边形顶点数组
        std::vector<Point> quadPoints = {Point(p1_x, p1_y), Point(p2_x, p2_y),
                                         Point(p3_x, p3_y), Point(p4_x, p4_y)};

        // 使用射线法（Ray Casting Algorithm）
        int intersections = 0;

        for (int i = 0; i < 4; i++) {
            Point p1 = quadPoints[i];
            Point p2 = quadPoints[(i + 1) % 4];

            // 检查射线与边的交点
            if (rayIntersectsSegment(px, py, p1.x, p1.y, p2.x, p2.y)) {
                intersections++;
            }
        }

        // 奇数个交点表示点在多边形内部
        return (intersections % 2) == 1;
    }

    // 判断从点(px, py)向右发出的射线是否与线段相交
    bool rayIntersectsSegment(float px, float py, float x1, float y1, float x2,
                              float y2) {
        // 确保y1 <= y2
        if (y1 > y2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        // 点的y坐标不在线段y范围内
        if (py < y1 || py > y2) {
            return false;
        }

        // 避免在顶点处的边界情况
        if (py == y1 || py == y2) {
            return false;
        }

        // 计算射线与线段的交点x坐标
        float intersectionX = x1 + (py - y1) * (x2 - x1) / (y2 - y1);

        // 交点在射线右侧
        return intersectionX > px;
    }
};
