
#include "spin_control/spin_control.hpp"
// static double spin_control::get_yaw(){return point_base_link.yaw;}
double spin_control::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
void spin_control::init() {
    // this->declare_parameter<double>("Kp",0.0001);
    // this->declare_parameter<double>("Ki",0.2);
    // this->declare_parameter<double>("Kd",0.3);
    // this->declare_parameter<double>("expect_vel",0.5);
    // this->declare_parameter<double>("actual_vel",0.0);
    // this->get_parameter("Kp",pid.Kp);
    // this->get_parameter("Ki",pid.Ki);
    // this->get_parameter("Kd",pid.Kd);
    // this->get_parameter("expect_vel",pid.expect_vel);
    // this->get_parameter("actual_vel",pid.actual_vel);
    this->declare_parameter<double>("speed", 1.0);
    this->declare_parameter<double>("tolerance", 0.25);
    this->declare_parameter<double>("lookahead_dist", 0.3);
    this->declare_parameter<double>("lookahead_time", 1.5);
    this->declare_parameter<double>("min_lookahead_dist", 0.15);
    this->declare_parameter<double>("max_lookahead_dist", 0.2);
    this->declare_parameter<bool>("use_velocity_scaled_lookahead_dist", false);

    this->get_parameter("speed", speed_);
    this->get_parameter("tolerance", tolerance_);
    this->get_parameter("lookahead_dist", lookahead_dist_);
    this->get_parameter("lookahead_time", lookahead_time_);
    this->get_parameter("min_lookahead_dist", min_lookahead_dist_);
    this->get_parameter("max_lookahead_dist", max_lookahead_dist_);
    this->get_parameter("speed", speed_);
    this->get_parameter("use_velocity_scaled_lookahead_dist",
                        use_velocity_scaled_lookahead_dist_);

    shaobing.is_arrived = true;
}
void spin_control::base_link_callback() {
    try {
        // 获取base_link坐标
        geometry_msgs::msg::TransformStamped msg;
        msg =
            tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        // transform_pub->publish(msg);
        base_link_map.x = msg.transform.translation.x;
        base_link_map.y = msg.transform.translation.y;
        base_link_map.z = msg.transform.translation.z;
        geometry_msgs::msg::PoseStamped base_link_point;
        base_link_point.header.frame_id = "map";
        auto clock = std::make_shared<rclcpp::Clock>();
        base_link_point.header.stamp = this->now();
        // pursuit_point.header=pursuit_pose.header;
        base_link_point.pose.position.x = msg.transform.translation.x;
        base_link_point.pose.position.y = msg.transform.translation.y;
        base_link_point.pose.position.z = msg.transform.translation.z;
        transform_pub->publish(base_link_point);
        // 使用四元数转换为欧拉角
        tf2::Quaternion q(msg.transform.rotation.x, msg.transform.rotation.y,
                          msg.transform.rotation.z, msg.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        shaobing.robot_x = base_link_map.x;
        shaobing.robot_y = base_link_map.y;
        shaobing.robot_yaw = yaw;

        // RCLCPP_INFO(this->get_logger(),"base_link:\nx:%f,y:%f,z:%f,roll:%f,pitch:%f,yaw:%f",base_link_map.x,base_link_map.y,base_link_map.z,base_link_map.roll,base_link_map.pitch,base_link_map.yaw);
        // 消息类型转换
        tf2::Transform transform;
        tf2::fromMsg(msg.transform, transform);

        // 计算map相对于base的平移矩阵,求逆变换
        tf2::Transform map_to_base_link = transform.inverse();

        // 给雷达用的
        base_link_to_map_translation = transform.getOrigin();
        base_link_to_map_rotation.setRotation(transform.getRotation());

        // 获取平移矩阵和旋转矩阵
        map_to_base_link_translation = map_to_base_link.getOrigin();
        tf2::Quaternion rotation = map_to_base_link.getRotation();
        map_to_base_link_rotation.setRotation(rotation);

    } catch (tf2::TransformException& e) {
        // RCLCPP_ERROR(this->get_logger(), "Failed to transform vehicle pose:
        // %s", e.what());
    }
}
// 获取全局路径,对每个路径点做新坐标系的变换
void spin_control::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = msg;
    point_i = 0;
}
// void spin_control::optimize_speed(const double & dist_error, const double &
// lookahead_dist, const double & speed, const double & pose_cost)
// {
//     //弦长
//     double
//     chord=pow(distance(path_in_base_position.getX(),path_in_base_position.getY(),base_link_map.x,base_link_map.y),2);
//     //横向误差 追踪路径点在地图中的横坐标减base_link在map中的横坐标
//     double lateral_err=6;
//     //曲率
//     double curvature=2*lateral_err/chord;
//     double radius=1/curvature;
//     //根据曲率来约束速度
//     RCLCPP_INFO(rclcpp::get_logger("spin_control"),"curvature=%f",curvature);

// }
void spin_control::calculate_cmd_vel() {
    if (!path_) {
        // RCLCPP_WARN(this->get_logger(), "No path received yet.");
        std::cout << "没有路径，此时到达" << shaobing.is_arrived << std::endl;
        robot_pub->publish(shaobing);

        return;
    }
    // RCLCPP_WARN(this->get_logger(), "path size :%d",path_->poses.size());

    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::PoseStamped pursuit_point;
    const double lookahead_dist = getLookAheadDistance(speed_);
    // RCLCPP_INFO(this->get_logger(),"LOOKAHEAD_DISTANCE : %f",lookahead_dist);
    auto pursuit_pose = getLookAheadPoint(lookahead_dist, path_);
    // RCLCPP_INFO(this->get_logger(), "x: %f,y:
    // %f",pursuit_pose.pose.position.x,pursuit_pose.pose.position.y);
    pursuit_point.header.frame_id = "map";
    auto clock = std::make_shared<rclcpp::Clock>();
    pursuit_point.header.stamp = this->now();
    // pursuit_point.header=pursuit_pose.header;
    pursuit_point.point.x = pursuit_pose.pose.position.x;
    pursuit_point.point.y = pursuit_pose.pose.position.y;
    pursuit_point.point.z = pursuit_pose.pose.position.z;
    pursuit_point_pub->publish(pursuit_point);
    tf2::Vector3 every_point(pursuit_pose.pose.position.x,
                             pursuit_pose.pose.position.y,
                             pursuit_pose.pose.position.z);
    // 计算路径点在base_link下的坐标
    path_in_base_position =
        map_to_base_link_rotation * every_point + map_to_base_link_translation;
    // 计算雷达与路径点的夹角
    point_base_link.yaw =
        atan2(path_in_base_position.getY(), path_in_base_position.getX());
    double yaw = point_base_link.yaw / M_PI * 180;
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f",point_base_link.yaw/M_PI*180);
    double dis =
        distance(base_link_map.x, base_link_map.y,
                 path_->poses[path_->poses.size() - 1].pose.position.x,
                 path_->poses[path_->poses.size() - 1].pose.position.y);
    // RCLCPP_WARN(this->get_logger(), "dis:%f",dis);

    // pursuit_pose==path_->poses[path_->poses.size()-1]&&
    if (dis < tolerance_) {
        shaobing.is_arrived = true;
        RCLCPP_INFO(this->get_logger(), "到达目标点");
        std::cout << "到达目标点" << std::endl;

        // std::cout << "daoda" << std::endl;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
    } else {
        shaobing.is_arrived = false;
        cmd_vel.linear.x = speed_ * cos(point_base_link.yaw);
        cmd_vel.linear.y = speed_ * sin(point_base_link.yaw);
    }
    cmd_vel.linear.z =
        atan2(path_in_base_position.getX(), path_in_base_position.getY());
    if (yaw > -180 && yaw < -90) {
        shaobing.expect_spin_degree = -270 - yaw;
    } else {
        shaobing.expect_spin_degree = 90 - yaw;
    }

    std::cout << "daoda" << shaobing.is_arrived << std::endl;

    cmd_pub->publish(cmd_vel);
    robot_pub->publish(shaobing);
}

double spin_control::getLookAheadDistance(double speed) {
    // 如果使用基于速度缩放的前瞻距离，则计算并限制距离值
    // 否则，使用静态的前瞻距离
    double lookahead_distance = lookahead_dist_;
    if (use_velocity_scaled_lookahead_dist_) {
        lookahead_distance = speed * lookahead_time_;
        lookahead_distance =
            clamp(lookahead_distance, min_lookahead_dist_, max_lookahead_dist_);
    }
    return lookahead_distance;
}
// geometry_msgs::msg::PoseStamped spin_control::getLookAheadPoint(const double
// & lookahead_dist, const nav_msgs::msg::Path::SharedPtr & path)
// {
//     // 找到第一个距离大于前瞻距离的姿态
//     for(;point_i<path->poses.size();point_i++)
//     {
//         if(distance(path->poses[point_i].pose.position.x,path->poses[point_i].pose.position.y,base_link_map.x,base_link_map.y)>=
//         lookahead_dist)
//         {
//         first_pose=path->poses[point_i];
//         break;

//         }
//     }

//     return first_pose;

// }
geometry_msgs::msg::PoseStamped spin_control::getLookAheadPoint(
    const double& lookahead_dist, const nav_msgs::msg::Path::SharedPtr& path) {
    geometry_msgs::msg::PoseStamped first_pose;

    // 异常情况处理
    if (!path || path->poses.empty()) {
        return first_pose;  // 返回空姿态
    }

    // 获取当前机器人位置 (基于pose_tf结构)
    const pose_tf& current = base_link_map;

    // 1. 查找最近路径点
    auto closest_it = std::min_element(
        path->poses.begin(), path->poses.end(),
        [&](const auto& a, const auto& b) {
            const double dx_a = a.pose.position.x - current.x;
            const double dy_a = a.pose.position.y - current.y;
            const double dx_b = b.pose.position.x - current.x;
            const double dy_b = b.pose.position.y - current.y;
            return (dx_a * dx_a + dy_a * dy_a) < (dx_b * dx_b + dy_b * dy_b);
        });

    // 2. 沿路径累计距离搜索
    double accum_dist = 0.0;
    auto target_it = closest_it;

    while (target_it != path->poses.end()) {
        // 从最近点的下一个点开始累计距离
        if (target_it != closest_it) {
            const auto& prev_pose = std::prev(target_it)->pose.position;
            const auto& curr_pose = target_it->pose.position;
            accum_dist += std::hypot(curr_pose.x - prev_pose.x,
                                     curr_pose.y - prev_pose.y);
        }

        // 找到首个超过预瞄距离的点
        if (accum_dist >= lookahead_dist) {
            break;
        }

        ++target_it;
    }

    // 处理边界情况
    if (target_it != path->poses.end()) {
        first_pose = *target_it;
    } else {
        first_pose = path->poses.back();
        // 可选：添加距离插值逻辑
    }

    return first_pose;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<spin_control>();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        // node->base_link_callback();
    }

    rclcpp::shutdown();
    return 0;
}