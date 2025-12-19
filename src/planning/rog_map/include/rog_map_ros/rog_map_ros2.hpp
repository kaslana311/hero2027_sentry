/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

// #ifndef USE_ROS1
// #ifndef USE_ROS2
// #error "Please define either USE_ROS1 or USE_ROS2, but not both."
// #endif
// #endif

// #ifdef USE_ROS1
// #ifdef USE_ROS2
// #error "Cannot use both USE_ROS1 and USE_ROS2 at the same time. Please define
// only one." #endif #endif

// #ifdef USE_ROS2

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <rog_map/rog_map.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <super_utils/color_msg_utils.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rog_map
{
using namespace super_utils;

class ROGMapROS : public ROGMap
{
  rclcpp::Node::SharedPtr nh_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> br_map_ego_;

  const double getSystemWalltimeNow() override { return nh_->get_clock()->now().seconds(); }

  void getSystemWalltimeNow(rclcpp::Time & _in) { _in = nh_->get_clock()->now(); };

  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    int unfinished_frame_cnt{0};
    Pose pc_pose;
    PointCloud pc;
    rclcpp::TimerBase::SharedPtr update_timer;
    mutex updete_lock;
  } rc_;
  struct CostPub
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_pc_pub;
  } cost_pub_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    updateRobotState(std::make_pair(
      Vec3f(
        odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z),
      Quatf(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)));

    // geometry_msgs::msg::TransformStamped transformStamped;
    // transformStamped.header.stamp = nh_->get_clock()->now();
    // transformStamped.header.frame_id = "odom";
    // transformStamped.child_frame_id = "drone";
    // transformStamped.transform.translation.x =
    // odom_msg->pose.pose.position.x; transformStamped.transform.translation.y
    // = odom_msg->pose.pose.position.y;
    // transformStamped.transform.translation.z =
    // odom_msg->pose.pose.position.z; transformStamped.transform.rotation.x =
    // odom_msg->pose.pose.orientation.x; transformStamped.transform.rotation.y
    // = odom_msg->pose.pose.orientation.y;
    // transformStamped.transform.rotation.z =
    // odom_msg->pose.pose.orientation.z; transformStamped.transform.rotation.w
    // = odom_msg->pose.pose.orientation.w;
    // br_map_ego_->sendTransform(transformStamped);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!robot_state_.rcv) {
      std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback." << RESET
                << std::endl;
      return;
    }
    double cbk_t = nh_->get_clock()->now().seconds();
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
      std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
      return;
    }
    PointCloud temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
  }

  void updateCallback()
  {
    auto start_time = std::chrono::steady_clock::now();
    auto send_time = nh_->get_clock()->now();
    if (map_empty_) {
      static double last_print_t = nh_->get_clock()->now().seconds();
      double cur_t = nh_->get_clock()->now().seconds();
      if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
        std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET
                  << std::endl;
        last_print_t = cur_t;
      }
      return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
      return;
    }

    if (rc_.unfinished_frame_cnt > 1) {
      std::cout << YELLOW
                << " -- [ROG WARN] Unfinished frame cnt > 1, the map may not "
                   "work in real-time"
                << RESET << std::endl;
    }
    static PointCloud temp_pc;
    static Pose temp_pose;

    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    updateProbMap(temp_pc, temp_pose);
    /************ */
    // Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    // Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;
    vec_E<Vec3f> occ_map;
    auto localMap_start_time = std::chrono::steady_clock::now();
    // boundBoxByLocalMap(box_min, box_max);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    // boxSearch(box_min, box_max, OCCUPIED, occ_map);
    GetObsFromTimeList(occ_map);
    auto localMap_end_time = std::chrono::steady_clock::now();
    double localMap_time_consuming =
      std::chrono::duration_cast<std::chrono::milliseconds>(localMap_end_time - localMap_start_time)
        .count();
    std::cout << GREEN << " -- [ROG INFO] Local map extraction in " << localMap_time_consuming
              << " ms, occ pts num: " << occ_map.size() << RESET << std::endl;
    auto cloud_start_time = std::chrono::steady_clock::now();
    if (!occ_map.empty()) {
      vecEVec3fToPC2(occ_map, cloud_msg);
      cloud_msg.header.stamp = send_time;
      cost_pub_.cost_pc_pub->publish(cloud_msg);
    }
    auto cloud_end_time = std::chrono::steady_clock::now();
    double cloud_time_consuming =
      std::chrono::duration_cast<std::chrono::milliseconds>(cloud_end_time - cloud_start_time)
        .count();
    std::cout << GREEN << " -- [ROG INFO] PointCloud2 conversion and publishing in "
              << cloud_time_consuming << " ms." << RESET << std::endl;
    /************* */
    auto end_time = std::chrono::steady_clock::now();
    double time_consuming =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << GREEN << " -- [ROG INFO] Map updated in " << time_consuming << " ms." << RESET
              << std::endl;
    writeTimeConsumingToLog(time_log_file_);
  }
  void GetObsFromTimeList(vec_E<Vec3f> & occ_map)
  {
    occ_map.reserve(time_stamp_lists.size());
    for (auto it = time_stamp_lists.begin(); it != time_stamp_lists.end(); ++it) {
      Vec3i id_g;
      hashIdToGlobalIndex(it->first, id_g);
      Vec3f pos;
      globalIndexToPos(id_g, pos);
      occ_map.push_back(pos);
    }
  }
  void vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud)
  {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = nh_->get_clock()->now();
    cloud.header.frame_id = "odom";
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  ROGMapROS(const rclcpp::Node::SharedPtr nh) : nh_(nh)
  {
    // TODO: The current implementation uses a lenient QoS configuration for
    // message transmission.
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    nh->declare_parameter<std::string>("cfg_path", "rog_map_cfg.yaml");
    std::string cfg_path;
    nh->get_parameter("cfg_path", cfg_path);
    cfg_ = rog_map::Config(cfg_path);

    cost_pub_.cost_pc_pub =
      nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", qos);
    init();
    /// Initialize visualization module

    if (cfg_.ros_callback_en) {
      rc_.odom_me_cbk_group =
        nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group =
        nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, qos, std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      rc_.cloud_sub = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic, qos, std::bind(&ROGMapROS::cloudCallback, this, std::placeholders::_1),
        so);
      rc_.update_cbk_group =
        nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = nh_->create_wall_timer(
        std::chrono::milliseconds(1),  // 0.001秒，即1毫秒
        std::bind(&ROGMapROS::updateCallback, this), rc_.update_cbk_group);
    }
  }

private:
};
}  // namespace rog_map
#endif  // ROG_MAP_ROS_HPP
