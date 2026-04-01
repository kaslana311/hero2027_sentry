#ifndef HERO_LIDAR_SCAN_NODE_HPP_
#define HERO_LIDAR_SCAN_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hero_lidar_scan {

class HeroLidarScanNode : public rclcpp::Node {
   public:
    explicit HeroLidarScanNode(const rclcpp::NodeOptions& options);

   private:
    // 里程计回调
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 双雷达同步回调
    void lidarSyncCallback(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar1_msg,
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar2_msg);

    // 位姿插值查询（基于传入的副本，无锁）
    bool getPoseAtTimeLocal(
        const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
        double target_time, Eigen::Matrix4d& pose);

    // 处理单个雷达点云去畸变
    void processLidar(
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
        const Eigen::Matrix4d& T_ext, const Eigen::Matrix4d& pose_end,
        const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
        pcl::PointCloud<pcl::PointXYZI>& cloud_out);

    // 订阅器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar1_sub_;
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar2_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        livox_ros_driver2::msg::CustomMsg, livox_ros_driver2::msg::CustomMsg>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 里程计队列（线程安全）
    std::deque<nav_msgs::msg::Odometry> odom_buffer_;
    std::mutex odom_mutex_;
    size_t max_odom_buffer_size_;

    // 外参矩阵
    Eigen::Matrix4d T_lidar1_to_base_;
    Eigen::Matrix4d T_lidar2_to_base_;
};

}  // namespace hero_lidar_scan

#endif  // HERO_LIDAR_SCAN_NODE_HPP_
