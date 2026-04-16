/**
 * @file bspline_smoother.cpp
 * @brief Nav2 B-spline 平滑器实现
 */

#include "Bstyle_smoother/bspline_smoother.hpp"

#include <algorithm>
#include <cmath>

#include "Bstyle_smoother/non_uniform_bspline.h"
#include "nav2_util/node_utils.hpp"

namespace nav2_bspline_smoother
{

void BSplineSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_sub_ = costmap_sub;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  clock_ = node->get_clock();
  logger_ = node->get_logger();

  optimizer_.setParam(std::make_shared<fast_planner::BsplineConfig>(config_bspline_));
  // // 声明并获取参数
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name_ + ".bspline_order", rclcpp::ParameterValue(3));
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name_ + ".lambda_1", rclcpp::ParameterValue(1.0));
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name_ + ".lambda_2", rclcpp::ParameterValue(0.5));
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name_ + ".lambda_3", rclcpp::ParpathToBsplineambda_1_);
  // node->get_parameter(name_ + ".lambda_2", lambda_2_);
  // node->get_parameter(name_ + ".lambda_3", lambda_3_);
  // node->get_parameter(name_ + ".dist_threshold", dist_threshold_);
  // node->get_parameter(name_ + ".max_vel", max_vel_);
  // node->get_parameter(name_ + ".max_acc", max_acc_);
  // node->get_parameter(name_ + ".max_iterations", max_iterations_);
  // node->get_parameter(name_ + ".min_point_spacing", min_point_spacing_);
}

void BSplineSmoother::cleanup() { RCLCPP_INFO(logger_, "Cleaning up BSplineSmoother"); }

void BSplineSmoother::activate() { RCLCPP_INFO(logger_, "Activating BSplineSmoother"); }

void BSplineSmoother::deactivate() { RCLCPP_INFO(logger_, "Deactivating BSplineSmoother"); }

bool BSplineSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & /*max_time*/)
{
  // 检查路径长度
  auto start_time = clock_->now();
  if (path.poses.size() < static_cast<size_t>(2 + 1)) {
    RCLCPP_WARN(logger_, "Path too short for B-spline smoothing");
    return true;  // 路径太短不需要平滑
  }

  // 获取 costmap
  auto costmap_ptr = costmap_sub_->getCostmap();
  if (!costmap_ptr) {
    RCLCPP_ERROR(logger_, "Failed to get costmap");
    return false;
  }

  optimizer_.setEnvironment(costmap_ptr);
  // 强制重建 ESDF 快照, 确保本次优化基于一致的距离场
  optimizer_.rebuildESDF();

  // costmap_ = costmap_ptr.get();

  // 将路径转换为 B-spline 控制点
  Eigen::MatrixXd ctrl_pts;
  double interval;
  pathToBspline(path, ctrl_pts, interval);

  // 执行优化
  if (!optimizeBspline(ctrl_pts, interval)) {
    RCLCPP_WARN(
      rclcpp::get_logger("BsplineOptimizer"),
      "B-spline optimization failed, returning original path");
    return true;
  }

  // 将优化后的 B-spline 转换回路径
  fast_planner::NonUniformBspline optimized_bspline(ctrl_pts, bspline_order_, interval);
  nav_msgs::msg::Path original_path = path;
  bsplineToPath(optimized_bspline, path, original_path);
  auto end_time = clock_->now();
  RCLCPP_INFO(
    rclcpp::get_logger("BsplineOptimizer"), "B-spline smoothing completed in %.2f ms",
    (end_time - start_time).nanoseconds() / 1e6);
  return true;
}

void BSplineSmoother::pathToBspline(
  const nav_msgs::msg::Path & path, Eigen::MatrixXd & ctrl_pts, double & interval)
{
  size_t n = path.poses.size();

  // 计算路径总长度
  double total_dist = 0.0;
  for (size_t i = 1; i < n; ++i) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    total_dist += std::sqrt(dx * dx + dy * dy);
  }

  // 确定合理的控制点数量
  constexpr int MIN_CTRL_PTS = 10;
  constexpr int MAX_CTRL_PTS = 150;
  constexpr double MIN_INTERVAL = 0.05;
  constexpr double MAX_INTERVAL = 0.25;

  // 每 0.3m 一个路径采样点 (parameterizeToBspline2D 会再加 2 个控制点)
  // 不能太密, 否则矩阵求解不稳定; 不能太稀, 否则曲线穿障碍物
  int desired_pts = static_cast<int>(total_dist / 0.3) + 1;
  desired_pts = std::clamp(desired_pts, MIN_CTRL_PTS - 2, MAX_CTRL_PTS - 2);

  // 计算采样间距（用于重采样路径）
  double sample_spacing = total_dist / static_cast<double>(std::max(desired_pts - 1, 1));

  // 时间间隔 = 采样间距 / 期望速度，但要限制在合理范围内
  interval = std::clamp(sample_spacing / max_vel_, MIN_INTERVAL, MAX_INTERVAL);

  // 重采样路径点，使其更均匀
  std::vector<Eigen::Vector2d> point_set;
  point_set.reserve(desired_pts + 1);

  // 添加起点
  point_set.emplace_back(path.poses[0].pose.position.x, path.poses[0].pose.position.y);

  double accumulated_dist = 0.0;
  double next_sample_dist = sample_spacing;

  for (size_t i = 1; i < n; ++i) {
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    double seg_len = std::sqrt(dx * dx + dy * dy);

    double prev_accumulated = accumulated_dist;
    accumulated_dist += seg_len;

    // 在当前段上采样
    while (next_sample_dist <= accumulated_dist &&
           static_cast<int>(point_set.size()) < desired_pts) {
      double t = (next_sample_dist - prev_accumulated) / seg_len;
      double x = path.poses[i - 1].pose.position.x + t * dx;
      double y = path.poses[i - 1].pose.position.y + t * dy;
      point_set.emplace_back(x, y);
      next_sample_dist += sample_spacing;
    }
  }

  // 添加终点
  point_set.emplace_back(path.poses.back().pose.position.x, path.poses.back().pose.position.y);

  // 起点和终点速度/加速度全部设为零
  // 非零起始速度会让 QR 求解在边界处产生偏移较大的控制点
  // 导致优化时前 order_ 个固定控制点位置不佳（可能靠近障碍物或偏转严重）
  // 零速度约束让 B-spline 起点处控制点紧密聚集在路径起点附近, 更安全
  Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
  Eigen::Vector2d end_vel = Eigen::Vector2d::Zero();
  Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
  Eigen::Vector2d end_acc = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector2d> derivatives = {start_vel, end_vel, start_acc, end_acc};

  fast_planner::NonUniformBspline::parameterizeToBspline2D(
    interval, point_set, derivatives, ctrl_pts);
ctrl_pts(0,0)=path.poses[0].pose.position.x;
ctrl_pts(0,1)=path.poses[0].pose.position.y;
  RCLCPP_INFO(
    rclcpp::get_logger("BSplineSmoother"),
    "Path: %.2fm, %zu pts -> B-spline: %ld ctrl pts, interval=%.3f", total_dist, n, ctrl_pts.rows(),
    interval);
}

void BSplineSmoother::bsplineToPath(
  const fast_planner::NonUniformBspline & bspline, nav_msgs::msg::Path & path,
  const nav_msgs::msg::Path & original_path)
{
  // 输出路径采样: 取原始点数的 2 倍或至少 100 个点, 保证丝滑
  size_t num_points = std::max(original_path.poses.size() * 2, static_cast<size_t>(100));

  double um, um_p;
  // 使用 const_cast 因为 getTimeSpan 没有 const 修饰符
  const_cast<fast_planner::NonUniformBspline &>(bspline).getTimeSpan(um, um_p);
  double duration = um_p - um;
  double dt = duration / static_cast<double>(num_points - 1);

  path.header = original_path.header;
  path.poses.resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    double t = um + dt * static_cast<double>(i);
    Eigen::VectorXd pos = const_cast<fast_planner::NonUniformBspline &>(bspline).evaluateDeBoor(t);

    path.poses[i].header = original_path.header;
    path.poses[i].pose.position.x = pos(0);
    path.poses[i].pose.position.y = pos(1);
    path.poses[i].pose.position.z = 0.0;

    // 计算朝向
    if (i < num_points - 1) {
      double next_t = um + dt * static_cast<double>(i + 1);
      Eigen::VectorXd next_pos =
        const_cast<fast_planner::NonUniformBspline &>(bspline).evaluateDeBoor(next_t);
      double yaw = std::atan2(next_pos(1) - pos(1), next_pos(0) - pos(0));

      path.poses[i].pose.orientation.x = 0.0;
      path.poses[i].pose.orientation.y = 0.0;
      path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
      path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      // 最后一个点使用前一个点的朝向
      path.poses[i].pose.orientation = path.poses[i - 1].pose.orientation;
    }
  }
}

bool BSplineSmoother::optimizeBspline(Eigen::MatrixXd & ctrl_pts, double interval)
{
  // ---- 起始段安全修正: 将靠近障碍物的前几个控制点推离 ----
  // 前 2 个控制点(P0,P1)在优化中固定不动, 如果它们在障碍物附近会很危险
  // 检查前 order_ 个控制点并修正
  const double safe_dist = config_bspline_.dist0_ * 0.8;  // 稍小于 dist0 即触发修正
  for (int i = 0; i < std::min(bspline_order_, static_cast<int>(ctrl_pts.rows())); ++i) {
    Eigen::Vector2d pos(ctrl_pts(i, 0), ctrl_pts(i, 1));
    double dist;
    Eigen::Vector2d grad;
    optimizer_.evaluateEDTWithGrad(pos, -1.0, dist, grad);

    if (dist < safe_dist && dist > -10.0) {
      // 沿梯度方向（远离障碍物）推离
      double push_dist = safe_dist - dist + 0.05;  // 多推一点余量
      if (grad.norm() > 1e-6) {
        ctrl_pts(i, 0) += grad(0) * push_dist;
        ctrl_pts(i, 1) += grad(1) * push_dist;
        RCLCPP_INFO(
          rclcpp::get_logger("BSplineSmoother"),
          "Pushed ctrl_pt[%d] away from obstacle by %.3fm (dist was %.3f)", i, push_dist, dist);
      }
    }
  }

  // 保存原始控制点作为引导路径 (防止优化结果偏离原始路径太远)
  std::vector<Eigen::Vector2d> guide_pts;
  guide_pts.reserve(ctrl_pts.rows());
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    guide_pts.emplace_back(ctrl_pts(i, 0), ctrl_pts(i, 1));
  }
  optimizer_.setGuidePath(guide_pts);

  // 单次完整优化: 平滑 + 避障 + 可行性 + 终点 + 引导
  // parameterizeToBspline2D 已经给了高质量初始控制点, 不需要分阶段
  int cf = fast_planner::BsplineOptimizer::NORMAL_PHASE;
  cf |= fast_planner::BsplineOptimizer::ENDPOINT;
  cf |= fast_planner::BsplineOptimizer::GUIDE;
  ctrl_pts = optimizer_.BsplineOptimizeTraj(ctrl_pts, interval, cf, 1, 1);

  return true;
}

}  // namespace nav2_bspline_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_bspline_smoother::BSplineSmoother, nav2_core::Smoother)
