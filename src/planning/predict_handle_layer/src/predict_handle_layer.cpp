#include "predict_handle_layer/predict_handle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

using namespace predict_handle_layer;

PredictHandleLayer::PredictHandleLayer() : nav2_costmap_2d::CostmapLayer() {}
PredictHandleLayer::~PredictHandleLayer() {}

void PredictHandleLayer::onInitialize() {
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    // 声明参数
    declareParameter("tracker_topic", rclcpp::ParameterValue("/tracker"));
    declareParameter("predict_time", rclcpp::ParameterValue(1.0));

    declareParameter("max_cost", rclcpp::ParameterValue(200));
    declareParameter("min_cost", rclcpp::ParameterValue(100));

    std::string tracker_topic;
    node->get_parameter(name_ + ".tracker_topic", tracker_topic);
    node->get_parameter(name_ + ".predict_time", predict_time_);

    node->get_parameter(name_ + ".max_cost", max_cost_);
    node->get_parameter(name_ + ".min_cost", min_cost_);
    default_value_ = 0;
    rolling_window_ = layered_costmap_->isRolling();
    was_reset_ = false;

    // // 关键：设置默认值为 NO_INFORMATION 或 FREE_SPACE
    // default_value_ = nav2_costmap_2d::FREE_SPACE;  // 或
    // nav2_costmap_2d::FREE_SPACE

    matchSize();

    // 关键：初始化后立即清零
    // resetMaps();

    rclcpp::QoS qos(5);
    qos.best_effort();
    tracker_sub_ = node->create_subscription<interfaces::msg::TrackerOutput>(
        tracker_topic, qos,
        [this](const interfaces::msg::TrackerOutput::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(tracker_mutex_);
            last_tracker_msg_ = *msg;
            has_new_data_ = true;
        });

    last_update_time_ = node->get_clock()->now();

    RCLCPP_INFO(node->get_logger(), "PredictHandleLayer initialized, topic: %s",
                tracker_topic.c_str());
}

void PredictHandleLayer::updateBounds(double robot_x, double robot_y,
                                      double /*robot_yaw*/, double* min_x,
                                      double* min_y, double* max_x,
                                      double* max_y) {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    if (rolling_window_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2,
                     robot_y - getSizeInMetersY() / 2);
    }
    *min_x = std::min(*min_x, robot_x - getSizeInMetersX() / 2);
    *min_y = std::min(*min_y, robot_y - getSizeInMetersY() / 2);
    *max_x = std::max(*max_x, robot_x + getSizeInMetersX() / 2);
    *max_y = std::max(*max_y, robot_y + getSizeInMetersY() / 2);
    // 获取最新数据
    interfaces::msg::TrackerOutput tracker_msg;
    {
        std::lock_guard<std::mutex> lock(tracker_mutex_);
        // 关键：如果没有新数据或没有目标，直接返回
        // 此时 costmap 已被 resetMaps() 清零，不会污染 master
        if (!has_new_data_) {
            // 不更新边界，让 updateCosts 跳过
            // std::cout<<"no new  date
            // kkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"<<std::endl;
            return;
        }
        previous_targets_.clear();
        tracker_msg = last_tracker_msg_;
        last_tracker_msg_.targets.clear();
        has_new_data_ = false;
    }
    // TODO: 根据信息估计,动态轨迹预测,避开障碍物等
    //  计算预测轨迹,敌方机器人沿着对应速度方向直线运动,预测一段时间t,对应代价区域为长方形,代价数值随着预测距离递减,限制在200
    //  到100之间
    //轨迹区域映射为栅格地图的格子,加入到 以 target.id 为键的 previous_targets_
    //映射中,用于填入代价地图
    for (const auto& target : tracker_msg.targets) {
        double x0 = target.position.x + robot_x;
        double y0 = target.position.y + robot_y;

        double vx = target.velocity.x;
        double vy = target.velocity.y;
        double speed = std::hypot(vx, vy);

        double len = (speed * predict_time_);  // 预测位移的一半
        double radius = std::max(target.radius_1, target.radius_2);
        int num = static_cast<int>(len / radius) + 1;
        for (int i = 1; i <= num; i++) {
            double nx = x0 + vx * predict_time_ * i / num;
            double ny = y0 + vy * predict_time_ * i / num;
            previous_targets_[target.id].emplace_back(circle{
                nx, ny, radius, static_cast<unsigned char>(240 - i * 10)});
        }
    }
}

void PredictHandleLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                     int min_i, int min_j, int max_i,
                                     int max_j) {
    if (previous_targets_.empty()) return;
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    for (auto it = previous_targets_.begin(); it != previous_targets_.end();
         ++it) {
        for (size_t i = 0; i < it->second.size(); i++) {
            // std::cout<<"target id: "<<it->first<<" circle x:
            // "<<it->second[i].x<<" y: "<<it->second[i].y<<" radius:
            // "<<it->second[i].radius<<" cost:
            // "<<(int)it->second[i].cost<<std::endl;
            std::cout << "start print  " << std::endl;
            unsigned int min_xi, min_yi, max_xi, max_yi;
            if (worldToMap(it->second[i].x - it->second[i].radius,
                           it->second[i].y - it->second[i].radius, min_xi,
                           min_yi) == false)
                continue;
            if (worldToMap(it->second[i].x + it->second[i].radius,
                           it->second[i].y + it->second[i].radius, max_xi,
                           max_yi) == false)
                continue;
            //便利矩形区域内的栅格
            std::cout << "target id: " << it->first
                      << " circle x: " << it->second[i].x
                      << " y: " << it->second[i].y
                      << " radius: " << it->second[i].radius
                      << " cost: " << (int)it->second[i].cost << std::endl;
            for (unsigned int mx = min_xi; mx <= max_xi; ++mx) {
                for (unsigned int my = min_yi; my <= max_yi; ++my) {
                    if (master_grid.getCost(mx, my) >= it->second[i].cost)
                        continue;

                    master_grid.setCost(mx, my, it->second[0].cost);
                }
            }
        }
    }
}

void PredictHandleLayer::reset() {
    was_reset_ = true;
    // resetMaps();
}

void PredictHandleLayer::activate() {}

void PredictHandleLayer::deactivate() {}

void PredictHandleLayer::PredictForward() {}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(predict_handle_layer::PredictHandleLayer,
                       nav2_costmap_2d::Layer)