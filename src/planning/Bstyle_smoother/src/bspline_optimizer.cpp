#ifndef FAST_PLANNER__BSPLINE_OPTIMIZER_HPP_
#define FAST_PLANNER__BSPLINE_OPTIMIZER_HPP_

#include "Bstyle_smoother/bspline_optimizer.h"

#include <algorithm>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nlopt.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fast_planner
{

const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT = (1 << 3);
const int BsplineOptimizer::GUIDE = (1 << 4);
const int BsplineOptimizer::WAYPOINTS = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
  BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(std::shared_ptr<BsplineConfig> nh)
{
  lambda1_ = nh->lambda1_;
  lambda2_ = nh->lambda2_;
  lambda3_ = nh->lambda3_;
  lambda4_ = nh->lambda4_;
  lambda5_ = nh->lambda5_;
  lambda6_ = nh->lambda6_;
  lambda7_ = nh->lambda7_;
  lambda8_ = nh->lambda8_;
  dist0_ = nh->dist0_;
  max_vel_ = nh->max_vel_;
  max_acc_ = nh->max_acc_;
  visib_min_ = nh->visib_min_;
  dlmin_ = nh->dlmin_;
  wnl_ = nh->wnl_;

  max_iteration_num_[0] = nh->max_iteration_num1_;
  max_iteration_num_[1] = nh->max_iteration_num2_;
  max_iteration_num_[2] = nh->max_iteration_num3_;
  max_iteration_num_[3] = nh->max_iteration_num4_;
  max_iteration_time_[0] = nh->max_iteration_time1_;
  max_iteration_time_[1] = nh->max_iteration_time2_;
  max_iteration_time_[2] = nh->max_iteration_time3_;
  max_iteration_time_[3] = nh->max_iteration_time4_;

  algorithm1_ = nh->algorithm1_;
  algorithm2_ = nh->algorithm2_;
  order_ = nh->order_;
}

void BsplineOptimizer::setEnvironment(const std::shared_ptr<nav2_costmap_2d::Costmap2D> env)
{
  edt_environment_ = env;
}

void BsplineOptimizer::rebuildESDF()
{
  esdf_valid_ = false;
  buildESDFCache();
}

// ============ 核心改进：用 OpenCV distanceTransform 构建真正的 ESDF ============
void BsplineOptimizer::buildESDFCache()
{
  if (!edt_environment_) {
    esdf_valid_ = false;
    return;
  }

  esdf_size_x_ = static_cast<int>(edt_environment_->getSizeInCellsX());
  esdf_size_y_ = static_cast<int>(edt_environment_->getSizeInCellsY());
  esdf_resolution_ = edt_environment_->getResolution();
  esdf_origin_x_ = edt_environment_->getOriginX();
  esdf_origin_y_ = edt_environment_->getOriginY();

  // Step 1: 构建二值障碍物图 (0=障碍物, 255=自由空间)
  // 注意: OpenCV 的 distanceTransform 计算的是非零像素到最近零像素的距离
  // 我们要的是每个点到最近障碍物的距离, 所以障碍物=0, 自由=255
  cv::Mat obstacle_map(esdf_size_y_, esdf_size_x_, CV_8UC1);

  unsigned char * costmap_data = edt_environment_->getCharMap();
  for (int y = 0; y < esdf_size_y_; ++y) {
    for (int x = 0; x < esdf_size_x_; ++x) {
      // costmap 的索引: index = y * size_x + x
      unsigned char cost = costmap_data[y * esdf_size_x_ + x];
      // LETHAL_OBSTACLE(254) 和 INSCRIBED_INFLATED_OBSTACLE(253) 以及 NO_INFORMATION(255) 都视为障碍物
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        obstacle_map.at<uchar>(y, x) = 0;  // 障碍物
      } else {
        obstacle_map.at<uchar>(y, x) = 255;  // 自由空间
      }
    }
  }

  // Step 2: 计算距离变换 (每个自由空间像素到最近障碍物的距离，单位：像素)
  cv::Mat dist_pixels;
  cv::distanceTransform(obstacle_map, dist_pixels, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  // Step 3: 同时计算障碍物内部的距离（取反图做一次）
  cv::Mat free_map;
  cv::bitwise_not(obstacle_map, free_map);
  cv::Mat dist_inside_pixels;
  cv::distanceTransform(free_map, dist_inside_pixels, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  // Step 4: 合成完整的 Signed Distance Field (SDF)
  // 自由空间: 正距离; 障碍物内部: 负距离
  esdf_cache_ = cv::Mat(esdf_size_y_, esdf_size_x_, CV_32FC1);
  for (int y = 0; y < esdf_size_y_; ++y) {
    for (int x = 0; x < esdf_size_x_; ++x) {
      float d_free = dist_pixels.at<float>(y, x);
      float d_inside = dist_inside_pixels.at<float>(y, x);
      // 自由空间 d_free > 0, d_inside == 0
      // 障碍物内部 d_free == 0, d_inside > 0
      esdf_cache_.at<float>(y, x) = (d_free - d_inside) * static_cast<float>(esdf_resolution_);
    }
  }

  // Step 5: 计算 ESDF 的梯度场 (Sobel, 指向距离增大方向即远离障碍物方向)
  cv::Mat grad_x_raw, grad_y_raw;
  cv::Sobel(esdf_cache_, grad_x_raw, CV_32F, 1, 0, 3);  // d(esdf)/dx
  cv::Sobel(esdf_cache_, grad_y_raw, CV_32F, 0, 1, 3);  // d(esdf)/dy

  // 归一化梯度（Sobel 3x3 的尺度因子为 1/8）
  esdf_grad_x_ = grad_x_raw / 8.0f;
  esdf_grad_y_ = grad_y_raw / 8.0f;

  esdf_valid_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("BsplineOptimizer"), "ESDF cache built: %dx%d, resolution=%.3f",
    esdf_size_x_, esdf_size_y_, esdf_resolution_);
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd & points)
{
  control_points_ = points;
  dim_ = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double & ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int & max_num_id, const int & max_time_id)
{
  max_num_id_ = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int & cost_code)
{
  cost_function_ = cost_code;

  // print optimized cost function
  std::string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  RCLCPP_INFO_STREAM(rclcpp::get_logger("BsplineOptimizer"), "cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const std::vector<Eigen::Vector2d> & guide_pt)
{
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(
  const std::vector<Eigen::Vector2d> & waypts, const std::vector<int> & waypt_idx)
{
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(
  const Eigen::MatrixXd & points, const double & ts, const int & cost_function, int max_num_id,
  int max_time_id)
{
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize()
{
  /* initialize solver */
  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - start_fixed_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) * (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
                           control_points_.row(pt_num - 1));
  } else {
    variable_num_ = std::max(0, dim_ * (pt_num - start_fixed_ - order_));
  }

  // std::cout << "Variable Num " << variable_num_<< std::endl;
  // std::cout << "Is quadratic " << isQuadratic()<< std::endl;

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);  // 收紧: 控制点相对变化 < 1e-6 才停
  opt.set_ftol_rel(1e-5);  // 收紧: 代价函数相对变化 < 1e-7 才停

  std::vector<double> q(variable_num_);
  for (int i = start_fixed_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - start_fixed_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    std::vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 2.0;  // 控制点最多偏移 2m (减小搜索空间, 提升稳定性)
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
    RCLCPP_INFO(
      rclcpp::get_logger("BsplineOptimizer"), "NLopt result=%d, final_cost=%.4f, iter=%d",
      static_cast<int>(result), final_cost, iter_num_);
  } catch (std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("BsplineOptimizer"), "[Optimization]: nlopt exception: %s", e.what());
  }

  for (int i = start_fixed_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - start_fixed_) + j];
    }
  }
  if (!(cost_function_ & GUIDE))
    RCLCPP_INFO_STREAM(rclcpp::get_logger("BsplineOptimizer"), "iter num: " << iter_num_);
}

// void BsplineOptimizer::calcSmoothnessCost(
//   const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
// {
//   cost = 0.0;
//   Eigen::Vector2d zero(0, 0);
//   std::fill(gradient.begin(), gradient.end(), zero);

//   int n = static_cast<int>(q.size());

//   // ---- 加速度平滑项 (二阶差分): 主要的平滑力量, 惩罚曲率变化 ----
//   // 这是保证丝滑的核心: 让相邻控制点的间距变化均匀
//   float xishu = 1.0;
//   for (int i = 0; i < n - 2; i++) {
//     Eigen::Vector2d acc = xishu * (q[i + 2] - 2.0 * q[i + 1] + q[i]);

//     cost += acc.squaredNorm();
//     Eigen::Vector2d temp_a = 2.0 * acc;

//     gradient[i + 0] += temp_a;
//     gradient[i + 1] += -2.0 * temp_a;
//     gradient[i + 2] += temp_a;
//   }

//   // ---- Jerk 平滑项 (三阶差分): 惩罚曲率的突变 (使曲率变化连续) ----
//   double jerk_weight = 6.0;  // 全向底盘: jerk 权重提高, 消除曲率跳变
//   for (int i = 0; i < n - 3; i++) {
//     Eigen::Vector2d jerk = q[i + 3] - 3.0 * q[i + 2] + 3.0 * q[i + 1] - q[i];
//     cost += jerk_weight * jerk.squaredNorm();
//     Eigen::Vector2d temp_j = 2.0 * jerk_weight * jerk;

//     gradient[i + 0] += -temp_j;
//     gradient[i + 1] += 3.0 * temp_j;
//     gradient[i + 2] += -3.0 * temp_j;
//     gradient[i + 3] += temp_j;
//   }

//   // ---- Snap 平滑项 (四阶差分): 使 jerk 也连续, 让全向底盘运动如行云流水 ----
//   // 对于全向移动机器人, snap 连续意味着力矩变化平滑, 电机不会有冲击
//   double snap_weight = 0.9;
//   for (int i = 0; i < n - 4; i++) {
//     Eigen::Vector2d snap = q[i + 4] - 4.0 * q[i + 3] + 6.0 * q[i + 2] - 4.0 * q[i + 1] + q[i];
//     cost += snap_weight * snap.squaredNorm();
//     Eigen::Vector2d temp_s = 2.0 * snap_weight * snap;

//     gradient[i + 0] += temp_s;
//     gradient[i + 1] += -4.0 * temp_s;
//     gradient[i + 2] += 6.0 * temp_s;
//     gradient[i + 3] += -4.0 * temp_s;
//     gradient[i + 4] += temp_s;
//   }
// }
void BsplineOptimizer::calcSmoothnessCost(
  const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
{
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int n = static_cast<int>(q.size());
  if (n < 5) return;

  // -------------------------------------------------------------------------
  // 1. 阈值曲率惩罚 (Threshold Curvature Penalty)
  // 仅当转弯半径过小（二阶差分过大）时施加惩罚
  // -------------------------------------------------------------------------
  double curvature_limit_weight = 30.0;
  double min_radius = 0.2;  // 期望的最小曲率半径 (m)

  // 1. 先计算平均间距 delta_s
  double total_dist = 0;
  for (int i = 0; i < n - 1; i++) {
    total_dist += (q[i + 1] - q[i]).norm();
  }
  double delta_s = total_dist / (n - 1);

  // 2. 根据几何关系计算位移阈值
  // 这里的阈值单位是“米”，代表允许的最大折弯程度
  double max_bending_dis = (delta_s * delta_s) / min_radius;

  for (int i = 0; i < n - 2; i++) {
    Eigen::Vector2d acc = q[i + 2] - 2.0 * q[i + 1] + q[i];
    double acc_norm = acc.norm();

    if (acc_norm > max_bending_dis) {
      double diff = acc_norm - max_bending_dis;
      cost += curvature_limit_weight * diff * diff;

      // 梯度计算部分是正确的 (链式法则)
      // g = d(cost)/d(acc_norm) * d(acc_norm)/d(acc)
      // d(acc_norm)/d(acc) = acc / acc_norm (单位向量)
      Eigen::Vector2d g_base = (2.0 * curvature_limit_weight * diff / acc_norm) * acc;

      gradient[i + 0] += g_base;
      gradient[i + 1] += -2.0 * g_base;
      gradient[i + 2] += g_base;
    }
  }
  // -------------------------------------------------------------------------
  // 2. 均匀性惩罚 (Uniformity) - 必须保留
  // 只有点分布均匀，上面的 acc_norm 才能准确代表几何曲率
  // -------------------------------------------------------------------------
  double uniformity_weight = 2.0;
  for (int i = 0; i < n - 2; i++) {
    Eigen::Vector2d diff = q[i + 2] - 2.0 * q[i + 1] + q[i];
    cost += uniformity_weight * diff.squaredNorm();
    Eigen::Vector2d g = 2.0 * uniformity_weight * diff;
    gradient[i + 0] += g;
    gradient[i + 1] += -2.0 * g;
    gradient[i + 2] += g;
  }

  // -------------------------------------------------------------------------
  // 3. Jerk & Snap (保留基础平滑，权重可以适当调低，让位给阈值惩罚)
  // -------------------------------------------------------------------------
  double jerk_weight = 2.0;
  for (int i = 0; i < n - 3; i++) {
    Eigen::Vector2d jerk = q[i + 3] - 3.0 * q[i + 2] + 3.0 * q[i + 1] - q[i];
    cost += jerk_weight * jerk.squaredNorm();
    Eigen::Vector2d g = 2.0 * jerk_weight * jerk;
    gradient[i + 0] += -g;
    gradient[i + 1] += 3.0 * g;
    gradient[i + 2] += -3.0 * g;
    gradient[i + 3] += g;
  }

  // 4. Snap 平滑: 惩罚 Jerk 的变化率 (全向底盘力矩平滑)

  // -------------------------------------------------------------------------

  double snap_weight = 1.0;

  for (int i = 0; i < n - 4; i++) {
    Eigen::Vector2d snap = q[i + 4] - 4.0 * q[i + 3] + 6.0 * q[i + 2] - 4.0 * q[i + 1] + q[i];

    cost += snap_weight * snap.squaredNorm();

    Eigen::Vector2d g = 2.0 * snap_weight * snap;

    gradient[i + 0] += g;

    gradient[i + 1] += -4.0 * g;

    gradient[i + 2] += 6.0 * g;

    gradient[i + 3] += -4.0 * g;

    gradient[i + 4] += g;
  }
}
//TODO
//这里通过得到的nav2占据地图完成sdf的计算并完善相关部分

void BsplineOptimizer::evaluateEDTWithGrad(
  const Eigen::Vector2d & pos, double /*time*/, double & dist, Eigen::Vector2d & grad)
{
  if (!esdf_valid_) {
    buildESDFCache();
  }
  if (!esdf_valid_) {
     dist = 0.0;
    grad = Eigen::Vector2d::Zero();
    return;
  }

  double fx = (pos(0) - esdf_origin_x_) / esdf_resolution_;
  double fy = (pos(1) - esdf_origin_y_) / esdf_resolution_;

  // 二次插值需要 3x3 窗口，中心点周围各需 1 像素，安全起见留 2 像素边界
  if (fx < 2.0 || fx >= esdf_size_x_ - 2.0 || fy < 2.0 || fy >= esdf_size_y_ - 2.0) {
    dist = 0.0;
    grad = Eigen::Vector2d::Zero();
    return;
  }

  // 找到最近的整数网格点作为中心
  int ix = static_cast<int>(std::round(fx));
  int iy = static_cast<int>(std::round(fy));

  // 相对于中心点的偏移 (范围在 [-0.5, 0.5])
  double dx = fx - ix;
  double dy = fy - iy;

  // --- 二次插值基函数 (Lagrange 多项式形式) ---
  // b_m1: x-1 的权重, b_0: x 的权重, b_p1: x+1 的权重
  double bx_m1 = 0.5 * (dx * dx - dx);
  double bx_0 = 1.0 - dx * dx;
  double bx_p1 = 0.5 * (dx * dx + dx);

  double by_m1 = 0.5 * (dy * dy - dy);
  double by_0 = 1.0 - dy * dy;
  double by_p1 = 0.5 * (dy * dy + dy);

  // --- 导数基函数 (用于直接计算梯度) ---
  double dbx_m1 = dx - 0.5;
  double dbx_0 = -2.0 * dx;
  double dbx_p1 = dx + 0.5;

  double dby_m1 = dy - 0.5;
  double dby_0 = -2.0 * dy;
  double dby_p1 = dy + 0.5;

  // --- 3x3 窗口内距离值采样 ---
  double d[3][3];
  for (int j = -1; j <= 1; ++j) {
    for (int i = -1; i <= 1; ++i) {
      d[j + 1][i + 1] = esdf_cache_.at<float>(iy + j, ix + i);
    }
  }

  // --- 计算插值距离 dist ---
  dist = 0.0;
  double row_val[3];  // 暂存每一行的 x 方向插值结果
  for (int j = 0; j < 3; ++j) {
    row_val[j] = bx_m1 * d[j][0] + bx_0 * d[j][1] + bx_p1 * d[j][2];
  }
  dist = by_m1 * row_val[0] + by_0 * row_val[1] + by_p1 * row_val[2];

  // --- 计算梯度 grad (直接对插值多项式求导) ---
  // x 方向梯度: 对 x 求导，y 方向保持插值
  double row_grad_x[3];
  for (int j = 0; j < 3; ++j) {
    row_grad_x[j] = dbx_m1 * d[j][0] + dbx_0 * d[j][1] + dbx_p1 * d[j][2];
  }
  double gx = by_m1 * row_grad_x[0] + by_0 * row_grad_x[1] + by_p1 * row_grad_x[2];

  // y 方向梯度: 对 y 求导，x 方向保持插值
  double row_val_x[3];  // 重用 x 方向插值结果
  for (int j = 0; j < 3; ++j) {
    row_val_x[j] = bx_m1 * d[j][0] + bx_0 * d[j][1] + bx_p1 * d[j][2];
  }
  double gy = dby_m1 * row_val_x[0] + dby_0 * row_val_x[1] + dby_p1 * row_val_x[2];

  // 转换到世界坐标系下的变化率 (除以分辨率)
  grad(0) = gx / esdf_resolution_;
  grad(1) = gy / esdf_resolution_;

  // 归一化梯度
  double grad_norm = grad.norm();
  if (grad_norm > 1e-6) {
    grad /= grad_norm;
  }
}

// calcDistanceCost - 使用 ESDF 缓存的精确距离场

void BsplineOptimizer::calcDistanceCost(
  const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
{
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double dist;
  Eigen::Vector2d dist_grad;

  int end_idx =
    (cost_function_ & ENDPOINT) ? static_cast<int>(q.size()) : static_cast<int>(q.size()) - order_;

  for (int i = start_fixed_; i < end_idx; i++) {
    evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);

    if (dist < dist0_&&dist_grad.squaredNorm()>1e-10) {
      double diff = dist0_ - dist;
      // ---- 连续惩罚: 使用三次函数, C1 连续, 无梯度跳变 ----
      // penalty = (dist0 - dist)^3 / dist0
      // 越接近障碍物惩罚增长越快, 但在 dist=dist0 处平滑过渡到 0
      double penalty = diff * diff * diff / dist0_;
      double d_penalty = -3.0 * diff * diff / dist0_;  // d(penalty)/d(dist)

      cost += penalty;
      gradient[i] += d_penalty * dist_grad;
    }

    // ---- 中点插值检查: 防止控制点之间的曲线穿过障碍物 ----
    if (i < end_idx - 1) {
      Eigen::Vector2d mid = 0.5 * (q[i] + q[i + 1]);
      evaluateEDTWithGrad(mid, -1.0, dist, dist_grad);

      if (dist < dist0_) {
        double diff = dist0_ - dist;
        double penalty = diff * diff * diff / dist0_;
        double d_penalty = -3.0 * diff * diff / dist0_;

        cost += 0.5 * penalty;
        gradient[i] += 0.25 * d_penalty * dist_grad;
        gradient[i + 1] += 0.25 * d_penalty * dist_grad;
      }
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(
  const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
{
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector2d vi = q[i + 1] - q[i];

    for (int j = 0; j < 2; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector2d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 2; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(
  const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
{
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector2d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(
  const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
{
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector2d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector2d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(
  const std::vector<Eigen::Vector2d> & q, double & cost, std::vector<Eigen::Vector2d> & gradient)
{
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx =
    (cost_function_ & ENDPOINT) ? static_cast<int>(q.size()) : static_cast<int>(q.size()) - order_;

  for (int i = start_fixed_; i < end_idx; i++) {
    if (i >= static_cast<int>(guide_pts_.size())) continue;

    Eigen::Vector2d diff = q[i] - guide_pts_[i];

    // 我们只需要微弱约束让它不要偏离太远。如果偏离太远产生抛物线拉回效果
    cost += diff.squaredNorm();
    gradient[i] += 2.0 * diff;
  }
}
void BsplineOptimizer::combineCost(
  const std::vector<double> & x, std::vector<double> & grad, double & f_combine)
{
  /* convert the NLopt format vector to control points. */
  // 2D 版本：dim_ = 2

  // 固定的前 start_fixed_ 个控制点（起点约束，只固定 P0, P1）
  for (int i = 0; i < start_fixed_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i](j) = control_points_(i, j);
    }
  }

  // 优化变量转换为控制点（从 start_fixed_ 开始）
  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + start_fixed_](j) = x[dim_ * i + j];
    }
  }

  // 固定的后 order_ 个控制点（终点约束，如果不优化终点）
  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {
      for (int j = 0; j < dim_; ++j) {
        g_q_[start_fixed_ + variable_num_ / dim_ + i](j) =
          control_points_(control_points_.rows() - order_ + i, j);
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  std::fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness = 0.0, f_distance = 0.0, f_feasibility = 0.0;
  double f_endpoint = 0.0, f_guide = 0.0, f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + start_fixed_](j);
      }
    }
  }

  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda2_ * g_distance_[i + start_fixed_](j);
      }
    }
  }

  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + start_fixed_](j);
      }
    }
  }

  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + start_fixed_](j);
      }
    }
  }

  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda5_ * g_guide_[i + start_fixed_](j);
      }
    }
  }

  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + start_fixed_](j);
      }
    }
  }
}

double BsplineOptimizer::costFunction(
  const std::vector<double> & x, std::vector<double> & grad, void * func_data)
{
  BsplineOptimizer * opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  return cost;
}

std::vector<Eigen::Vector2d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd & ctrl_pts)
{
  std::vector<Eigen::Vector2d> ctrl_q;
  ctrl_q.reserve(ctrl_pts.rows());
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(Eigen::Vector2d(ctrl_pts(i, 0), ctrl_pts(i, 1)));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic()
{
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner

#endif  // FAST_PLANNER__BSPLINE_OPTIMIZER_HPP_
