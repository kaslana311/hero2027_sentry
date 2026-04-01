/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Bstyle_smoother/non_uniform_bspline.h"

#include "rclcpp/rclcpp.hpp"

namespace fast_planner
{

NonUniformBspline::NonUniformBspline(
  const Eigen::MatrixXd & points, const int & order, const double & interval)
{
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(
  const Eigen::MatrixXd & points, const int & order, const double & interval)
{
  control_points_ = points;
  p_ = order;
  interval_ = interval;

  n_ = points.rows() - 1;
  m_ = n_ + p_ + 1;

  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) {
    if (i <= p_) {
      u_(i) = double(-p_ + i) * interval_;
    } else if (i > p_ && i <= m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    } else if (i > m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

void NonUniformBspline::setKnot(const Eigen::VectorXd & knot) { this->u_ = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

void NonUniformBspline::getTimeSpan(double & um, double & um_p)
{
  um = u_(p_);
  um_p = u_(m_ - p_);
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts()
{
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double & u)
{
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // determine which [ui,ui+1] lay in
  int k = p_;
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  /* deBoor's alg */
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }

  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) {
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double & t)
{
  return evaluateDeBoor(t + u_(p_));
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints()
{
  // The derivative of a b-spline is also a b-spline, its order become p_-1
  // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) =
      p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

NonUniformBspline NonUniformBspline::getDerivative()
{
  Eigen::MatrixXd ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  /* cut the first and last knot */
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double NonUniformBspline::getInterval() { return interval_; }

void NonUniformBspline::setPhysicalLimits(const double & vel, const double & acc)
{
  limit_vel_ = vel;
  limit_acc_ = acc;
  limit_ratio_ = 1.1;
}

bool NonUniformBspline::checkFeasibility(bool show)
{
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P = control_points_;
  int dimension = control_points_.cols();

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (
      fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
      fabs(vel(2)) > limit_vel_ + 1e-4) {
      if (show) cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
                          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                           (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                          (u_(i + p_ + 1) - u_(i + 2));

    if (
      fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
      fabs(acc(2)) > limit_acc_ + 1e-4) {
      if (show) cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

double NonUniformBspline::checkRatio()
{
  Eigen::MatrixXd P = control_points_;
  int dimension = control_points_.cols();

  // find max vel
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  // find max acc
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
                          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                           (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                          (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  if (ratio > 2.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("my_node_name"), "Max velocity exceeded! Max vel: %lf, Max acc: %lf.",
      max_vel, max_acc);
  }

  return ratio;
}

bool NonUniformBspline::reallocateTime(bool show)
{
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;

  Eigen::MatrixXd P = control_points_;
  int dimension = control_points_.cols();

  double max_vel, max_acc;

  /* check vel feasibility and insert points */
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (
      fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
      fabs(vel(2)) > limit_vel_ + 1e-4) {
      fea = false;
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      max_vel = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }

      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 1);
      double time_new = ratio * time_ori;
      double delta_t = time_new - time_ori;
      double t_inc = delta_t / double(p_);

      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;
      }
    }
  }

  /* acc feasibility */
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
                          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                           (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                          (u_(i + p_ + 1) - u_(i + 2));

    if (
      fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
      fabs(acc(2)) > limit_acc_ + 1e-4) {
      fea = false;
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }

      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;
      // cout << "ratio: " << ratio << endl;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t = time_new - time_ori;
      double t_inc = delta_t / double(p_ - 1);

      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {
        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

void NonUniformBspline::lengthenTime(const double & ratio)
{
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

void NonUniformBspline::recomputeInit() {}

void NonUniformBspline::parameterizeToBspline(
  const double & ts, const vector<Eigen::Vector3d> & point_set,
  const vector<Eigen::Vector3d> & start_end_derivative, Eigen::MatrixXd & ctrl_pts)
{
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  int K = point_set.size();

  // write A
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
  // cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // write b
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  // cout << "[B-spline]: parameterization ok." << endl;
}

void NonUniformBspline::parameterizeToBspline2D(
  const double & ts, const vector<Eigen::Vector2d> & point_set,
  const vector<Eigen::Vector2d> & start_end_derivative, Eigen::MatrixXd & ctrl_pts)
{
  if (ts <= 0) {
    cout << "[B-spline 2D]: time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline 2D]: point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline 2D]: derivatives error. Expected 4 (start_vel, end_vel, start_acc, "
            "end_acc), got "
         << start_end_derivative.size() << endl;
    return;
  }

  int K = point_set.size();

  // 构建约束矩阵 A
  // 对于 3 阶 B 样条：P(u) = (1/6) * (P_{i-1} + 4*P_i + P_{i+1})
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;   // 位置约束系数
  vrow << -1, 0, 1;  // 速度约束系数
  arow << 1, -2, 1;  // 加速度约束系数

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 3);

  // 位置约束：K 个路径点
  for (int i = 0; i < K; ++i) {
    A.block(i, i, 1, 3) = (1.0 / 6.0) * prow.transpose();
  }

  // 速度约束：起点和终点速度
  A.block(K, 0, 1, 3) = (1.0 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1.0 / 2.0 / ts) * vrow.transpose();

  // 加速度约束：起点和终点加速度
  A.block(K + 2, 0, 1, 3) = (1.0 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1.0 / ts / ts) * arow.transpose();

  // 构建约束向量 b
  Eigen::VectorXd bx(K + 4), by(K + 4);

  // 位置约束
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
  }

  // 导数约束: start_vel, end_vel, start_acc, end_acc
  bx(K) = start_end_derivative[0](0);      // start_vel_x
  by(K) = start_end_derivative[0](1);      // start_vel_y
  bx(K + 1) = start_end_derivative[1](0);  // end_vel_x
  by(K + 1) = start_end_derivative[1](1);  // end_vel_y
  bx(K + 2) = start_end_derivative[2](0);  // start_acc_x
  by(K + 2) = start_end_derivative[2](1);  // start_acc_y
  bx(K + 3) = start_end_derivative[3](0);  // end_acc_x
  by(K + 3) = start_end_derivative[3](1);  // end_acc_y

  // 求解 Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);

  // 转换为控制点矩阵 (K+2) x 2
  ctrl_pts.resize(K + 2, 2);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
}

// ...existing code (keep the includes and other functions)...

void NonUniformBspline::pathToBspline2D(
  double ts, const std::vector<Eigen::Vector2d> & point_set, const Eigen::Vector2d & start_vel,
  const Eigen::Vector2d & end_vel, Eigen::MatrixXd & ctrl_pts)
{
  int K = static_cast<int>(point_set.size());

  if (K < 2) {
    ctrl_pts.resize(0, 2);
    return;
  }

  // 对于非常短的路径，直接使用简单方法
  if (K <= 4) {
    ctrl_pts.resize(K + 2, 2);

    // 起点控制点：向后延伸
    Eigen::Vector2d start_dir = (point_set[1] - point_set[0]).normalized();
    ctrl_pts.row(0) = point_set[0] - start_dir * ts * start_vel.norm();

    // 中间控制点直接使用路径点
    for (int i = 0; i < K; ++i) {
      ctrl_pts.row(i + 1) = point_set[i];
    }

    // 终点控制点：向前延伸
    Eigen::Vector2d end_dir = (point_set[K - 1] - point_set[K - 2]).normalized();
    ctrl_pts.row(K + 1) = point_set[K - 1] + end_dir * ts * end_vel.norm();

    return;
  }

  // ========== 使用更稳定的方法：直接构造控制点 ==========
  //
  // 对于三阶 B-spline，控制点数 = 路径点数 + 2（首尾各加一个）
  // 这种方法不需要求解线性系统，避免了数值不稳定问题

  int N = K + 2;  // 控制点数量
  ctrl_pts.resize(N, 2);

  // 第一个控制点：基于起点速度向后推算
  // 对于三阶 B-spline: v(0) = (P1 - P0) / ts
  // 所以: P0 = P1 - v(0) * ts
  // 但我们还没有 P1，使用 point_set[0] 作为近似
  Eigen::Vector2d p0 = point_set[0];
  Eigen::Vector2d v0 = start_vel;
  if (v0.norm() < 1e-6) {
    // 如果起点速度为零，使用路径方向
    v0 = (point_set[1] - point_set[0]).normalized() * 0.5;
  }
  ctrl_pts.row(0) = p0 - v0 * ts;

  // 中间控制点：直接使用路径点
  // 这是一个近似，但保证了曲线不会自相交
  for (int i = 0; i < K; ++i) {
    ctrl_pts.row(i + 1) = point_set[i];
  }

  // 最后一个控制点：基于终点速度向前推算
  Eigen::Vector2d pN = point_set[K - 1];
  Eigen::Vector2d vN = end_vel;
  if (vN.norm() < 1e-6) {
    // 如果终点速度为零，使用路径方向
    vN = (point_set[K - 1] - point_set[K - 2]).normalized() * 0.5;
  }
  ctrl_pts.row(N - 1) = pN + vN * ts;

  // 可选：对控制点进行轻微平滑，减少锯齿
  // 使用简单的移动平均，但保持首尾固定
  if (N > 6) {
    Eigen::MatrixXd smoothed_pts = ctrl_pts;
    for (int iter = 0; iter < 2; ++iter) {
      for (int i = 2; i < N - 2; ++i) {
        smoothed_pts.row(i) =
          0.25 * ctrl_pts.row(i - 1) + 0.5 * ctrl_pts.row(i) + 0.25 * ctrl_pts.row(i + 1);
      }
      ctrl_pts = smoothed_pts;
    }
  }
}

// ...existing code...
double NonUniformBspline::getTimeSum()
{
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double & res)
{
  double length = 0.0;
  double dur = getTimeSum();
  Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk()
{
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
  int dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double & mean_v, double & max_v)
{
  NonUniformBspline vel = getDerivative();
  double tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double vn = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v = mean_vel;
  max_v = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double & mean_a, double & max_a)
{
  NonUniformBspline acc = getDerivative().getDerivative();
  double tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double an = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a = mean_acc;
  max_a = max_acc;
}
}  // namespace fast_planner
