#include "tracker_layer/tracker_layer.hpp"

#include <deque>
#include <iostream>
#include <iterator>
#include <ostream>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 确保包含这个具体的头文件

// #include <nav2_costmap_2d/footprint.hpp>
namespace tracker_layer {
Trackerlayer::Trackerlayer() {}
Trackerlayer::~Trackerlayer() {}
void Trackerlayer::onInitialize() {
    std::string topics_string;

    declareParameter("pointcloud_topic", rclcpp::ParameterValue("pointcloud"));
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    global_frame_ = layered_costmap_->getGlobalFrameID();
    default_value_ = 0;
    rolling_window_ = layered_costmap_->isRolling();
    was_reset_ = false;
    matchSize();
    node->get_parameter(name_ + ".pointcloud_topic", topics_string);
    rclcpp::QoS qos(10);
    qos.best_effort();
    clock_ = node->get_clock();  // 保存时钟指针
    tracker_sub_ = node->create_subscription<interfaces::msg::TrackerOutput>(
        topics_string, qos,
        std::bind(&Trackerlayer::trackerCallback, this, std::placeholders::_1));
}
void Trackerlayer::updateBounds(double robot_x, double robot_y,
                                double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y) {
    //...
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2,
                     robot_y - getSizeInMetersY() / 2);
    *min_x = std::min(*min_x, robot_x - getSizeInMetersX() / 2);
    *min_y = std::min(*min_y, robot_y - getSizeInMetersY() / 2);
    *max_x = std::max(*max_x, robot_x + getSizeInMetersX() / 2);
    *max_y = std::max(*max_y, robot_y + getSizeInMetersY() / 2);
    // std::cerr << "updateBounds called,  " << std::endl;
    {
        std::lock_guard<std::mutex> lock(mutex_obs);
        cache_obs_list_ = cell_obs_list_;
    }
    for (auto it = cache_obs_list_.begin(); it != cache_obs_list_.end(); ++it) {
        double wx, wy;
        mapToWorld(it->second[0].mx, it->second[0].my, wx, wy);
        touch(wx, wy, min_x, min_y, max_x, max_y);
    }
    // std::cerr << "cache_obs_list_ size after cleanup: " <<
    // cache_obs_list_.size() << std::endl;
}

// void Trackerlayer::matchSize() {}

void Trackerlayer::reset() {}

void Trackerlayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                               int min_i, int min_j, int max_i, int max_j) {
    // std::lock_guard<std::mutex> lock(mutex_cloud);
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (cache_obs_list_.empty()) {
        return;
    }
    // std::cerr << "updateCosts called, cache_obs_list_ size: " <<
    // cache_obs_list_.size() << std::endl;
    for (auto& obs_cells : cache_obs_list_) {
        for (auto& cell : obs_cells.second) {
            double wx, wy;
            mapToWorld(cell.mx, cell.my, wx, wy);
            if (worldToMap(wx, wy, cell.mx, cell.my)) {
                if (master_grid.getCost(cell.mx, cell.my) < cell.cost)
                    master_grid.setCost(cell.mx, cell.my, cell.cost);
            }
        }
    }
}
void Trackerlayer::deactivate() {}
void Trackerlayer::activate() {}
void Trackerlayer::trackerCallback(
    const interfaces::msg::TrackerOutput::SharedPtr msg) {
    tracker_cbk_count_++;
    if (msg->targets.empty()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_obs);
        for (auto it = cell_obs_list_.begin(); it != cell_obs_list_.end();) {
            if (tracker_cbk_count_ - it->first > 4) {
                it = cell_obs_list_.erase(it);
            } else {
                ++it;
            }
        }
    }
    // std::cerr << "trackerCallback called first , targets size: " <<
    // msg->targets.size() << std::endl; Target.position 和 velocity
    // 位于一个特殊坐标系下： 原点在 base_link，但方向与 odom 系对齐（即
    // base_link 旋转角归零）
    // geometry_msgs::msg::TransformStamped tf_global_frame_base;
    geometry_msgs::msg::TransformStamped tf_map_odom;
    if (global_frame_ == "odom") {
    if (tf_) {
        try {
            // tf_global_frame_base = tf_->lookupTransform(
            //     global_frame_, "base_link", tf2::TimePointZero);
            tf_map_odom =
                tf_->lookupTransform("odom","map",  tf2::TimePointZero);

        } catch (const tf2::TransformException& ex) {
            std::cerr << "Transform error: " << ex.what() << std::endl;
            return;
        }
    } else {
        return;
    }
    }

    tf2::Quaternion q_odom_map;
    tf2::fromMsg(tf_map_odom.transform.rotation, q_odom_map);

    tf2::Matrix3x3 rot_odom_map(q_odom_map);

    // tf2::Vector3 pos_base_global_frame(
    //     tf_global_frame_base.transform.translation.x,
    //     tf_global_frame_base.transform.translation.y,
    //     tf_global_frame_base.transform.translation.z);
    tf2::Vector3 pos_odom_map(
    tf_map_odom.transform.translation.x,
    tf_map_odom.transform.translation.y,
    tf_map_odom.transform.translation.z);

    for (auto& target : msg->targets) {
        tf2::Vector3 target_pos_aligned(target.position.x, target.position.y,
                                        target.position.z);
        tf2::Vector3 target_pos_map;
        if (global_frame_ == "odom"){
        target_pos_map =rot_odom_map * target_pos_aligned + pos_odom_map;}
        else{
            target_pos_map=target_pos_aligned;
            }
        // --- 速度处理 ---
        // 目标速度同 odom 方向，因此只需进行 odom 到 global_frame_ 的旋转转换
        tf2::Vector3 target_vel_global_frame_;
        if (global_frame_ == "odom") {
            tf2::Vector3 target_vel_aligned(
                target.velocity.x, target.velocity.y, target.velocity.z);
            target_vel_global_frame_ = rot_odom_map * target_vel_aligned;
        } else {
            target_vel_global_frame_ = tf2::Vector3(
                target.velocity.x, target.velocity.y, target.velocity.z);
        }

        Eigen::Vector2d vel(target_vel_global_frame_.x(),
                            target_vel_global_frame_.y());

        DynamicObstacle dyobs;
        dyobs.position =
            Eigen::Vector2d(target_pos_map.x(), target_pos_map.y());
        dyobs.velocity = Eigen::Vector2d(target_vel_global_frame_.x(),
                                         target_vel_global_frame_.y());
        dyobs.radius = std::max(target.radius_1, target.radius_2);
        // dyobs.last_seen = target.header.stamp;
        {
            int res = gaussian(dyobs);
            if (res == -1) {
                std::cerr << "obs discarded" << std::endl;
            } else if (res == -2||res ==-3) {
                std::cerr << "obs has no effect on map, discarded" <<" num "<<res<< std::endl;
            }
            
             else {
                std::cerr << "obs processed and added to list" << std::endl;
            }
        }
    }
}
int Trackerlayer::gaussian(DynamicObstacle& obs) {
    unsigned int center_mx, center_my;
    if (!worldToMap(obs.position.x(), obs.position.y(), center_mx, center_my)) {
        return -1;  //-1 表示舍弃该障碍物
    }

    double v_norm = obs.velocity.norm();
    if(v_norm<0.5&&v_norm>0.1) {obs.velocity=obs.velocity.normalized()*0.1;
    v_norm=0.1;
    }
    double angle = atan2(obs.velocity.y(), obs.velocity.x());
    double cos_a = cos(angle);
    double sin_a = sin(angle);

    // 参数设置
    double r_obs = obs.radius/2;  // 基础半径
    double k_speed = 0.8;       // 速度影响系数，可调
    double mu = 0.5;            // 公式中的方向权重因子项
    double w_obs = 255.0;       // 峰值代价

    // 长轴（速度方向）受速度影响呈对数增长，短轴和负长轴保持半径
    double sigma_front = r_obs + k_speed * std::log(1.0 + v_norm);
    if(sigma_front>r_obs+0.3) sigma_front=r_obs+0.3;
    double sigma_side = r_obs;
    double sigma_back = r_obs;

    // 确定搜索范围（例如 3 sigma 范围）
    double max_sigma = std::max(sigma_front, sigma_side);
    int cell_range = static_cast<int>(ceil(3.0 * max_sigma / resolution_));
    std::vector<pointt> points_in_obs;
    for (int dx = -cell_range; dx <= cell_range; dx++) {
        for (int dy = -cell_range; dy <= cell_range; dy++) {
            int imx = static_cast<int>(center_mx) + dx;
            int imy = static_cast<int>(center_my) + dy;
            unsigned int mx = static_cast<unsigned int>(imx);
            unsigned int my = static_cast<unsigned int>(imy);
            double wx, wy;
            mapToWorld(mx, my, wx, wy);
            if (!worldToMap(wx, wy, mx, my)) {
                continue;  // 越界检查
            }

            // 计算相对位移 r
            Eigen::Vector2d r(wx - obs.position.x(), wy - obs.position.y());
            double r_len = r.norm();
            if (r_len < r_obs) {
                //障碍物本身占据的区域直接赋予最大代价，无需计算高斯衰减和方向权重
                points_in_obs.emplace_back(mx, my,
                                           nav2_costmap_2d::LETHAL_OBSTACLE);
                continue;
            }

            // 旋转到障碍物局部坐标系：rx 为沿速度方向，ry 为垂直速度方向
            double rx = r.x() * cos_a + r.y() * sin_a;
            double ry = -r.x() * sin_a + r.y() * cos_a;

            // 选择各向异性的 sigma
            double s_x = (rx > 0) ? sigma_front : sigma_back;
            double s_y = sigma_side;

            // 计算指数项 E: ||r||^2 / 2sigma^2 (各向异性版本)
            double exponent =
                (rx * rx) / (2.0 * s_x * s_x) + (ry * ry) / (2.0 * s_y * s_y);

            // 计算方向项 D: (v_obs . r) / (||v_obs|| * ||r||)
            double D = 0.0;
            if (v_norm > 1e-4) {
                D = r.dot(obs.velocity) / (v_norm * r_len);
            }

            // 最终公式: J = w_obs * exp(-E) * (1 + mu * D)
            double cost_f = w_obs * std::exp(-exponent) * (1.0 + mu * D);

            // 限制代价在 0 - 230
            unsigned int final_cost =
                static_cast<unsigned int>(std::clamp(cost_f, 0.0, 255.0));

            if (final_cost > 10) {
                points_in_obs.emplace_back(mx, my, final_cost);
            }
        }
    }
    if (points_in_obs.empty()) {
        return -2;  //-2 表示该障碍物对地图没有影响，可以舍弃
    }
    {
        std::lock_guard<std::mutex> lock(mutex_obs);
        cell_obs_list_.emplace_back(tracker_cbk_count_, points_in_obs);
    }
    // std::cerr << "ooooooooooooooooooooooooooooooooooooooooooooooooooend" <<
    // std::endl;
    return 0;
}
}  // namespace tracker_layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tracker_layer::Trackerlayer, nav2_costmap_2d::Layer)