#include "nav_cv_bridge/autoaim_tracker.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav_cv_bridge {

AutoaimTracker::AutoaimTracker(const rclcpp::NodeOptions& options)
    : Node("autoaim_tracker", options) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tracker_sub_ = this->create_subscription<interfaces::msg::TrackerOutput>(
        "/tracker/autoaim", rclcpp::SensorDataQoS(),
        std::bind(&AutoaimTracker::trackerCallback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::SensorDataQoS(),
        std::bind(&AutoaimTracker::odomCallback, this, std::placeholders::_1));

    tracker_pub_ = this->create_publisher<interfaces::msg::TrackerOutput>(
        "/tracker/absolute_targets", rclcpp::SensorDataQoS());

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/absolute_targets_marker", rclcpp::QoS(1));

    RCLCPP_INFO(this->get_logger(), "AutoaimTracker node initialized.");
}

void AutoaimTracker::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = *msg;
    has_odom_ = true;
}

void AutoaimTracker::trackerCallback(
    interfaces::msg::TrackerOutput::SharedPtr msg) {
    if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for odom...");
        return;
    }

    nav_msgs::msg::Odometry odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom = current_odom_;
    }

    geometry_msgs::msg::TransformStamped map_to_base;
    geometry_msgs::msg::TransformStamped map_to_odom;
    try {
        map_to_base =
            tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        map_to_odom =
            tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "TF Error: %s", ex.what());
        return;
    }

    Eigen::Vector3d p_B_M(map_to_base.transform.translation.x,
                          map_to_base.transform.translation.y,
                          map_to_base.transform.translation.z);

    Eigen::Quaterniond q_B_M(
        map_to_base.transform.rotation.w, map_to_base.transform.rotation.x,
        map_to_base.transform.rotation.y, map_to_base.transform.rotation.z);

    Eigen::Quaterniond q_O_M(
        map_to_odom.transform.rotation.w, map_to_odom.transform.rotation.x,
        map_to_odom.transform.rotation.y, map_to_odom.transform.rotation.z);

    Eigen::Matrix3d R_O_M = q_O_M.toRotationMatrix();
    Eigen::Matrix3d R_B_M = q_B_M.toRotationMatrix();

    // Odom twist is in base_link frame
    Eigen::Vector3d v_B_B(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                          odom.twist.twist.linear.z);
    Eigen::Vector3d v_B_M = R_B_M * v_B_B;

    auto abs_msg = std::make_shared<interfaces::msg::TrackerOutput>();
    abs_msg->header = msg->header;
    abs_msg->header.frame_id = "map";

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& target : msg->targets) {
        Eigen::Vector3d p_obs(target.position.x, target.position.y,
                              target.position.z);
        Eigen::Vector3d v_obs(target.velocity.x, target.velocity.y,
                              target.velocity.z);

        Eigen::Vector3d p_T_M = p_B_M + R_O_M * p_obs;
        Eigen::Vector3d v_T_M = v_B_M + R_O_M * v_obs;

        interfaces::msg::Target abs_target = target;
        abs_target.position.x = p_T_M.x();
        abs_target.position.y = p_T_M.y();
        abs_target.position.z = p_T_M.z();
        abs_target.velocity.x = v_T_M.x();
        abs_target.velocity.y = v_T_M.y();
        abs_target.velocity.z = v_T_M.z();

        abs_msg->targets.push_back(abs_target);

        // Marker (圆柱体)
        visualization_msgs::msg::Marker marker;
        marker.header = abs_msg->header;
        marker.ns = "dyn_obs_cylinder";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = abs_target.position.x;
        marker.pose.position.y = abs_target.position.y;
        marker.pose.position.z = 0.3;  // 圆柱体中心高度
        marker.pose.orientation.w = 1.0;

        // 取 target 的较大半径作为圆柱半径
        double radius = std::max(target.radius_1, target.radius_2);
        marker.scale.x = radius * 2.0;  // 直径
        marker.scale.y = radius * 2.0;
        marker.scale.z = 0.6;  // 高度
        marker.color.r = 1.0f;
        marker.color.g = 0.3f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.3);
        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker velocity_marker;
        velocity_marker.header = abs_msg->header;
        velocity_marker.ns = "velocities";
        velocity_marker.id = id++;
        velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
        velocity_marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point;
        start_point.x = abs_target.position.x;
        start_point.y = abs_target.position.y;
        start_point.z = 0.3;

        geometry_msgs::msg::Point end_point;
        end_point.x = abs_target.position.x + abs_target.velocity.x;
        end_point.y = abs_target.position.y + abs_target.velocity.y;
        end_point.z = 0.3;

        velocity_marker.points.push_back(start_point);
        velocity_marker.points.push_back(end_point);
        velocity_marker.scale.x = 0.05;
        velocity_marker.scale.y = 0.1;
        velocity_marker.scale.z = 0.1;
        velocity_marker.color.a = 1.0;
        velocity_marker.color.r = 0.0;
        velocity_marker.color.g = 1.0;
        velocity_marker.color.b = 0.0;
        marker_array.markers.push_back(velocity_marker);
    }

    tracker_pub_->publish(*abs_msg);
    marker_pub_->publish(marker_array);
}

}  // namespace nav_cv_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_cv_bridge::AutoaimTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}