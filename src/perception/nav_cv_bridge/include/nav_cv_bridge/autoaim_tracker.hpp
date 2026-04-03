#ifndef NAV_CV_BRIDGE__AUTOAIM_TRACKER_HPP_
#define NAV_CV_BRIDGE__AUTOAIM_TRACKER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <interfaces/msg/tracker_output.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace nav_cv_bridge {

class AutoaimTracker : public rclcpp::Node {
   public:
    explicit AutoaimTracker(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void trackerCallback(interfaces::msg::TrackerOutput::SharedPtr msg);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<interfaces::msg::TrackerOutput>::SharedPtr
        tracker_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<interfaces::msg::TrackerOutput>::SharedPtr tracker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;

    std::mutex odom_mutex_;
    nav_msgs::msg::Odometry current_odom_;
    bool has_odom_{false};
};

}  // namespace nav_cv_bridge

#endif  // NAV_CV_BRIDGE__AUTOAIM_TRACKER_HPP_
