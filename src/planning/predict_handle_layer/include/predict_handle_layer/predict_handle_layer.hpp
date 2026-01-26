#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "interfaces/msg/target.hpp"
#include "interfaces/msg/tracker_output.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace predict_handle_layer
{
class PredictHandleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PredictHandleLayer();
  ~PredictHandleLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  virtual void reset() override;
  virtual bool isClearable() override { return true; }
  virtual void deactivate() override;
  virtual void activate() override;

private:
  void PredictForward();
  void markPredictedTrajectory(double x0, double y0, double x1, double y1, double speed);
  void markCircle(double wx, double wy, double radius, unsigned char cost);

  bool rolling_window_{false};
  bool was_reset_{false};
  bool has_new_data_{false};
  std::vector<std::pair<int, int>> moves{{0, 1}, {1, 0},  {0, -1}, {-1, 0},
                                         {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  // 参数
  double predict_time_{1.0};
  double robot_radius_{0.3};
  int max_cost_{200};
  int min_cost_{100};
  double update_frequency_{5.0};

  // 订阅
  rclcpp::Subscription<interfaces::msg::TrackerOutput>::SharedPtr tracker_sub_;
  interfaces::msg::TrackerOutput last_tracker_msg_;
  std::mutex tracker_mutex_;
  rclcpp::Time last_update_time_;
  struct circle
  {
    double x;
    double y;
    double radius;
    unsigned char cost;
  };
  std::map<std::string, std::vector<circle>> previous_targets_;
};

}  // namespace predict_handle_layer