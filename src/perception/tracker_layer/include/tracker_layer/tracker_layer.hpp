#pragma once
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

#include "interfaces/msg/tracker_output.hpp"
namespace tracker_layer
{

struct DynamicObstacle
{
  Eigen::Vector2d position;  ///< 障碍物中心（map 坐标系，米）
  Eigen::Vector2d velocity;  ///< 障碍物速度（map 坐标系，m/s）
  double radius = 0.0;       ///< 膨胀后的碰撞半径（米）
  rclcpp::Time last_seen;
};
struct pointt
{
  unsigned int mx;
  unsigned int my;
  unsigned int cost;
  pointt(unsigned int mx, unsigned int my, unsigned int cost) : mx(mx), my(my), cost(cost) {}
};
class Trackerlayer : public nav2_costmap_2d::CostmapLayer
{
public:
  Trackerlayer();
  virtual ~Trackerlayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  // void updateOrigin(double new_origin_x, double new_origin_y);
  // virtual void matchSize();
  virtual void reset();
  virtual bool isClearable() { return true; }
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void deactivate();
  virtual void activate();

protected:
  // void updateFootprint(
  //   double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
  //   double * max_x, double * max_y);

private:
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  std::string global_frame_;
  bool rolling_window_;
  bool was_reset_;
  bool footprint_clearing_enabled_;
  std::mutex mutex_obs;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<interfaces::msg::TrackerOutput>::SharedPtr tracker_sub_;
  void trackerCallback(const interfaces::msg::TrackerOutput::SharedPtr msg);
  int gaussian(DynamicObstacle & obs);
  std::deque<std::pair<long long, std::vector<pointt>>> cell_obs_list_,
    cache_obs_list_;  // 存储当前帧所有动态障碍物占据的栅格坐标（mx, my）
  long long tracker_cbk_count_ = 0;
};
}  // namespace tracker_layer