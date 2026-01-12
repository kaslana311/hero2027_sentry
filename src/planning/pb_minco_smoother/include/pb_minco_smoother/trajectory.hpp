/**
 * @file trajectory.hpp
 * @brief 2D 多项式轨迹表示类
 *
 * 基于 GCOPTER (ZJU-FAST-Lab) 的 trajectory.hpp 修改
 * 原版为 3D 无人机轨迹，本版本改为 2D 地面机器人轨迹
 *
 * 主要修改：
 * 1. 将所有 Eigen::Vector3d 替换为 Eigen::Vector2d
 * 2. 将所有 Eigen::Matrix3Xd 替换为 Eigen::Matrix2Xd
 * 3. 移除 Z 轴相关的计算
 *
 * MIT License
 * Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)
 * Modified for 2D by Lihan Chen, 2025
 */

#ifndef PB_MINCO_SMOOTHER__TRAJECTORY_HPP_
#define PB_MINCO_SMOOTHER__TRAJECTORY_HPP_

#include "pb_minco_smoother/root_finder.hpp"

#include <Eigen/Eigen>
#include <cfloat>
#include <cmath>
#include <vector>

namespace pb_minco {

/**
 * @brief 单段多项式轨迹
 * @tparam D 多项式阶数 (degree)
 *
 * 对于 MINCO s=2 (最小化 Jerk)：D=3 (三次多项式)
 * 对于 MINCO s=3 (最小化 Snap)：D=5 (五次多项式)
 *
 * 多项式形式：p(t) = c_D * t^D + c_{D-1} * t^{D-1} + ... + c_1 * t + c_0
 * 系数矩阵存储：每列是一个系数，从高次到低次
 */
template <int D> class Piece {
public:
  // 2D 系数矩阵：2 行 (x, y)，D+1 列 (系数)
  using CoefficientMat = Eigen::Matrix<double, 2, D + 1>;
  using VelCoefficientMat = Eigen::Matrix<double, 2, D>;
  using AccCoefficientMat = Eigen::Matrix<double, 2, D - 1>;

private:
  double duration_;       ///< 该段轨迹的持续时间
  CoefficientMat coeffs_; ///< 系数矩阵

public:
  Piece() = default;

  /**
   * @brief 构造函数
   * @param dur 持续时间
   * @param cMat 系数矩阵
   */
  Piece(double dur, const CoefficientMat &cMat)
      : duration_(dur), coeffs_(cMat) {}

  /// @brief 获取维度 (固定为 2)
  inline int getDim() const { return 2; }

  /// @brief 获取多项式阶数
  inline int getDegree() const { return D; }

  /// @brief 获取持续时间
  inline double getDuration() const { return duration_; }

  /// @brief 获取系数矩阵引用
  inline const CoefficientMat &getCoeffMat() const { return coeffs_; }

  /**
   * @brief 计算时刻 t 的位置
   * @param t 时间 [0, duration]
   * @return 2D 位置向量
   *
   * 使用 Horner 法则的逆序计算：
   * p(t) = c_D + t*(c_{D-1} + t*(c_{D-2} + ... ))
   * 但这里为了数值稳定性，使用显式求和
   */
  inline Eigen::Vector2d getPos(double t) const {
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double tn = 1.0;
    // 系数从高次到低次存储，所以从 D 到 0 遍历
    for (int i = D; i >= 0; i--) {
      pos += tn * coeffs_.col(i);
      tn *= t;
    }
    return pos;
  }

  /**
   * @brief 计算时刻 t 的速度
   * @param t 时间 [0, duration]
   * @return 2D 速度向量
   *
   * v(t) = dp/dt = D*c_D*t^{D-1} + (D-1)*c_{D-1}*t^{D-2} + ... + c_1
   */
  inline Eigen::Vector2d getVel(double t) const {
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
    double tn = 1.0;
    int n = 1;
    for (int i = D - 1; i >= 0; i--) {
      vel += n * tn * coeffs_.col(i);
      tn *= t;
      n++;
    }
    return vel;
  }

  /**
   * @brief 计算时刻 t 的加速度
   * @param t 时间 [0, duration]
   * @return 2D 加速度向量
   */
  inline Eigen::Vector2d getAcc(double t) const {
    Eigen::Vector2d acc = Eigen::Vector2d::Zero();
    double tn = 1.0;
    int m = 1;
    int n = 2;
    for (int i = D - 2; i >= 0; i--) {
      acc += m * n * tn * coeffs_.col(i);
      tn *= t;
      m++;
      n++;
    }
    return acc;
  }

  /**
   * @brief 计算时刻 t 的加加速度 (Jerk)
   * @param t 时间 [0, duration]
   * @return 2D Jerk 向量
   */
  inline Eigen::Vector2d getJer(double t) const {
    Eigen::Vector2d jer = Eigen::Vector2d::Zero();
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    for (int i = D - 3; i >= 0; i--) {
      jer += l * m * n * tn * coeffs_.col(i);
      tn *= t;
      l++;
      m++;
      n++;
    }
    return jer;
  }

  /**
   * @brief 归一化位置系数矩阵
   *
   * 将时间归一化到 [0, 1]，系数矩阵需要相应缩放
   * 原多项式：p(t) = sum_{i=0}^{D} c_i * t^{D-i}
   * 归一化后：p(s) = sum_{i=0}^{D} c_i * (s*T)^{D-i} = sum_{i=0}^{D} (c_i *
   * T^{D-i}) * s^{D-i}
   */
  inline CoefficientMat normalizePosCoeffMat() const {
    CoefficientMat nPosCoeffsMat;
    double t = 1.0;
    for (int i = D; i >= 0; i--) {
      nPosCoeffsMat.col(i) = coeffs_.col(i) * t;
      t *= duration_;
    }
    return nPosCoeffsMat;
  }

  /// @brief 归一化速度系数矩阵
  inline VelCoefficientMat normalizeVelCoeffMat() const {
    VelCoefficientMat nVelCoeffMat;
    int n = 1;
    double t = duration_;
    for (int i = D - 1; i >= 0; i--) {
      nVelCoeffMat.col(i) = n * coeffs_.col(i) * t;
      t *= duration_;
      n++;
    }
    return nVelCoeffMat;
  }

  /// @brief 归一化加速度系数矩阵
  inline AccCoefficientMat normalizeAccCoeffMat() const {
    AccCoefficientMat nAccCoeffMat;
    int n = 2;
    int m = 1;
    double t = duration_ * duration_;
    for (int i = D - 2; i >= 0; i--) {
      nAccCoeffMat.col(i) = n * m * coeffs_.col(i) * t;
      n++;
      m++;
      t *= duration_;
    }
    return nAccCoeffMat;
  }

  /**
   * @brief 获取该段轨迹的最大速度
   * @return 最大速度模值
   *
   * 通过对速度平方函数求极值点来计算
   * ||v(t)||^2 = vx(t)^2 + vy(t)^2
   */
  inline double getMaxVelRate() const {
    VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
    // 速度平方的多项式系数
    Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                            RootFinder::polySqr(nVelCoeffMat.row(1));
    int N = coeff.size();
    int n = N - 1;
    // 对速度平方求导
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return getVel(0.0).norm();
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates = RootFinder::solvePolynomial(
          coeff.head(N - 1), l, r, FLT_EPSILON / duration_);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxVelRateSqr = -INFINITY;
      double tempNormSqr;
      for (auto it = candidates.begin(); it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr = getVel((*it) * duration_).squaredNorm();
          maxVelRateSqr =
              maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
        }
      }
      return sqrt(maxVelRateSqr);
    }
  }

  /**
   * @brief 获取该段轨迹的最大加速度
   * @return 最大加速度模值
   */
  inline double getMaxAccRate() const {
    AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
    Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                            RootFinder::polySqr(nAccCoeffMat.row(1));
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return getAcc(0.0).norm();
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates = RootFinder::solvePolynomial(
          coeff.head(N - 1), l, r, FLT_EPSILON / duration_);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxAccRateSqr = -INFINITY;
      double tempNormSqr;
      for (auto it = candidates.begin(); it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr = getAcc((*it) * duration_).squaredNorm();
          maxAccRateSqr =
              maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
        }
      }
      return sqrt(maxAccRateSqr);
    }
  }

  /**
   * @brief 检查速度是否超过限制
   * @param maxVelRate 最大速度限制
   * @return true 如果满足约束
   */
  inline bool checkMaxVelRate(double maxVelRate) const {
    double sqrMaxVelRate = maxVelRate * maxVelRate;
    if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
        getVel(duration_).squaredNorm() >= sqrMaxVelRate) {
      return false;
    } else {
      VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
      Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                              RootFinder::polySqr(nVelCoeffMat.row(1));
      double t2 = duration_ * duration_;
      coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
      return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
  }

  /**
   * @brief 检查加速度是否超过限制
   * @param maxAccRate 最大加速度限制
   * @return true 如果满足约束
   */
  inline bool checkMaxAccRate(double maxAccRate) const {
    double sqrMaxAccRate = maxAccRate * maxAccRate;
    if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
        getAcc(duration_).squaredNorm() >= sqrMaxAccRate) {
      return false;
    } else {
      AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
      Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                              RootFinder::polySqr(nAccCoeffMat.row(1));
      double t2 = duration_ * duration_;
      double t4 = t2 * t2;
      coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
      return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
  }
};

/**
 * @brief 多段多项式轨迹
 * @tparam D 多项式阶数
 *
 * 由多个 Piece<D> 组成的完整轨迹
 */
template <int D> class Trajectory {
private:
  using Pieces = std::vector<Piece<D>>;
  Pieces pieces_;

public:
  Trajectory() = default;

  /**
   * @brief 从持续时间和系数矩阵构造轨迹
   * @param durs 每段的持续时间
   * @param cMats 每段的系数矩阵
   */
  Trajectory(const std::vector<double> &durs,
             const std::vector<typename Piece<D>::CoefficientMat> &cMats) {
    int N = std::min(durs.size(), cMats.size());
    pieces_.reserve(N);
    for (int i = 0; i < N; i++) {
      pieces_.emplace_back(durs[i], cMats[i]);
    }
  }

  /// @brief 获取轨迹段数
  inline int getPieceNum() const { return pieces_.size(); }

  /// @brief 获取每段的持续时间向量
  inline Eigen::VectorXd getDurations() const {
    int N = getPieceNum();
    Eigen::VectorXd durations(N);
    for (int i = 0; i < N; i++) {
      durations(i) = pieces_[i].getDuration();
    }
    return durations;
  }

  /// @brief 获取总持续时间
  inline double getTotalDuration() const {
    double totalDuration = 0.0;
    for (const auto &piece : pieces_) {
      totalDuration += piece.getDuration();
    }
    return totalDuration;
  }

  /**
   * @brief 获取所有路点位置
   * @return 2 x (N+1) 矩阵，每列是一个路点
   */
  inline Eigen::Matrix2Xd getPositions() const {
    int N = getPieceNum();
    Eigen::Matrix2Xd positions(2, N + 1);
    for (int i = 0; i < N; i++) {
      positions.col(i) = pieces_[i].getCoeffMat().col(D);
    }
    positions.col(N) = pieces_[N - 1].getPos(pieces_[N - 1].getDuration());
    return positions;
  }

  // 下标访问
  inline const Piece<D> &operator[](int i) const { return pieces_[i]; }
  inline Piece<D> &operator[](int i) { return pieces_[i]; }

  // 容器操作
  inline void clear() { pieces_.clear(); }
  inline void reserve(int n) { pieces_.reserve(n); }

  inline typename Pieces::const_iterator begin() const {
    return pieces_.begin();
  }
  inline typename Pieces::const_iterator end() const { return pieces_.end(); }
  inline typename Pieces::iterator begin() { return pieces_.begin(); }
  inline typename Pieces::iterator end() { return pieces_.end(); }

  inline void emplace_back(const Piece<D> &piece) {
    pieces_.emplace_back(piece);
  }

  inline void emplace_back(double dur,
                           const typename Piece<D>::CoefficientMat &cMat) {
    pieces_.emplace_back(dur, cMat);
  }

  inline void append(const Trajectory<D> &traj) {
    pieces_.insert(pieces_.end(), traj.begin(), traj.end());
  }

  /**
   * @brief 定位时间 t 所在的轨迹段
   * @param t 输入/输出：输入为全局时间，输出为段内局部时间
   * @return 轨迹段索引
   */
  inline int locatePieceIdx(double &t) const {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces_[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces_[idx].getDuration();
    }
    return idx;
  }

  /// @brief 获取时刻 t 的位置
  inline Eigen::Vector2d getPos(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces_[pieceIdx].getPos(t);
  }

  /// @brief 获取时刻 t 的速度
  inline Eigen::Vector2d getVel(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces_[pieceIdx].getVel(t);
  }

  /// @brief 获取时刻 t 的加速度
  inline Eigen::Vector2d getAcc(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces_[pieceIdx].getAcc(t);
  }

  /// @brief 获取时刻 t 的 Jerk
  inline Eigen::Vector2d getJer(double t) const {
    int pieceIdx = locatePieceIdx(t);
    return pieces_[pieceIdx].getJer(t);
  }

  /// @brief 获取第 juncIdx 个路点的位置
  inline Eigen::Vector2d getJuncPos(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces_[juncIdx].getCoeffMat().col(D);
    } else {
      return pieces_[juncIdx - 1].getPos(pieces_[juncIdx - 1].getDuration());
    }
  }

  /// @brief 获取第 juncIdx 个路点的速度
  inline Eigen::Vector2d getJuncVel(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces_[juncIdx].getCoeffMat().col(D - 1);
    } else {
      return pieces_[juncIdx - 1].getVel(pieces_[juncIdx - 1].getDuration());
    }
  }

  /// @brief 获取第 juncIdx 个路点的加速度
  inline Eigen::Vector2d getJuncAcc(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces_[juncIdx].getCoeffMat().col(D - 2) * 2.0;
    } else {
      return pieces_[juncIdx - 1].getAcc(pieces_[juncIdx - 1].getDuration());
    }
  }

  /// @brief 获取整条轨迹的最大速度
  inline double getMaxVelRate() const {
    double maxVelRate = -INFINITY;
    for (const auto &piece : pieces_) {
      double tempNorm = piece.getMaxVelRate();
      maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
    }
    return maxVelRate;
  }

  /// @brief 获取整条轨迹的最大加速度
  inline double getMaxAccRate() const {
    double maxAccRate = -INFINITY;
    for (const auto &piece : pieces_) {
      double tempNorm = piece.getMaxAccRate();
      maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
    }
    return maxAccRate;
  }

  /// @brief 检查速度约束
  inline bool checkMaxVelRate(double maxVelRate) const {
    for (const auto &piece : pieces_) {
      if (!piece.checkMaxVelRate(maxVelRate)) {
        return false;
      }
    }
    return true;
  }

  /// @brief 检查加速度约束
  inline bool checkMaxAccRate(double maxAccRate) const {
    for (const auto &piece : pieces_) {
      if (!piece.checkMaxAccRate(maxAccRate)) {
        return false;
      }
    }
    return true;
  }
};

} // namespace pb_minco

#endif // PB_MINCO_SMOOTHER__TRAJECTORY_HPP_
