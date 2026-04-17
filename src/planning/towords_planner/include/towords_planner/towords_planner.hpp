#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <queue>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <thread>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "towords_planner/theta_star.hpp"

using rcl_interfaces::msg::ParameterType;

namespace towords_planner
{
struct Recover_point
{
  unsigned int mx;
  unsigned int my;
  int mode = -1;
};
class towords_planner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("towords_ThetaStarPlanner")};
  std::string global_frame_, name_;
  bool use_final_approach_orientation_;
  ///////////////////////////////////////
  std::thread * pub_data_thread_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr keypoint_pub_;
  inline void publishDataThread()
  {
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      //监听tf,得到当前位置姿态
      geometry_msgs::msg::TransformStamped current_pose;
      try {
        current_pose = tf_->lookupTransform(
          "base_footprint",   // 目标坐标系 (Target Frame)
          "map",              // 源坐标系 (Source Frame)
          tf2::TimePointZero  // 关键：表示获取 buffer 中最新的那一帧
        );
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Exception in publishDataThread: %s", e.what());
        continue;
      }
      //恢复状态判断
      if (rcp_.mode != -1) {
        if (planner_->CalDistance2RecoveryPoint(
              current_pose.transform.translation.x, current_pose.transform.translation.y, rcp_.mx,
              rcp_.my)) {
          std_msgs::msg::Int32 msg;
          msg.data = 0;
          keypoint_pub_->publish(msg);
          std::cerr << "HHHHHHHH 发布模式回复消息: " << 0 << std::endl;
          rcp_.mode = -1;
        }
      }
      KeyPoint key_point_in, key_point_out;
      //判断是否有匹配的关键点
      if (planner_->CheckHasKeyPoint(key_point_in, key_point_out)) {
        int mode;
        planner_->AdjustMode(key_point_in, key_point_out, mode);
        if (mode == 0) continue;
        //有的话,计算距离,如果在范围内就发布模式切换消息
        if (planner_->CalDistance2KeyPoint(
              current_pose.transform.translation.x, current_pose.transform.translation.y,
              key_point_in)) {
          std_msgs::msg::Int32 msg;
          msg.data = mode;
          keypoint_pub_->publish(msg);
          std::cerr << "FFFFFFFF 发布模式切换消息: " << mode << std::endl;
          rcp_.mode = mode;
          rcp_.mx = key_point_in.x;
          rcp_.my = key_point_in.y;
        }
      }
    }
  }
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  Recover_point rcp_;
  std::unique_ptr<theta_star::ThetaStar> planner_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
   * @brief the function responsible for calling the algorithm and retrieving a path from it
   * @return global_path is the planned path to be taken
   */
  void getPlan(nav_msgs::msg::Path & global_path);

  /**
   * @brief interpolates points between the consecutive waypoints of the path
   * @param raw_path is used to send in the path received from the planner
   * @param dist_bw_points is used to send in the interpolation_resolution (which has been set as the costmap resolution)
   * @return the final path with waypoints at a distance of the value of interpolation_resolution of each other
   */
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<coordsW> & raw_path, const double & dist_bw_points);

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);
};
}  // namespace towords_planner
