/**
 * @file root_finder.hpp
 * @brief 多项式求根工具类
 *
 * 基于 GCOPTER (ZJU-FAST-Lab) 的 root_finder.hpp
 * 用于轨迹的最值查找（如最大速度/加速度）
 *
 * 本文件无需维度相关的修改，因为它是通用的多项式求根工具
 *
 * MIT License
 * Copyright (c) 2020 Zhepei Wang (wangzhepei@live.com)
 * Modified for pb_minco_smoother by Lihan Chen, 2025
 */

#ifndef PB_MINCO_SMOOTHER__ROOT_FINDER_HPP_
#define PB_MINCO_SMOOTHER__ROOT_FINDER_HPP_

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <cstring>
#include <set>

#include <Eigen/Eigen>

namespace pb_minco {

namespace RootFinderParam {
constexpr size_t highestOrder = 64;
}

/**
 * @brief 多项式求根工具类
 *
 * 提供多项式求值、求导、求根等功能
 * 主要用于轨迹的极值查找
 */
class RootFinder {
public:
  /**
   * @brief 计算两个多项式乘积的平方
   * @param coef 多项式系数（行向量）
   * @return 平方多项式的系数
   *
   * 用于计算 ||v(t)||^2 = vx(t)^2 + vy(t)^2 的多项式系数
   */
  template <typename Derived>
  static Eigen::VectorXd polySqr(const Eigen::MatrixBase<Derived> &coef) {
    int len = coef.size();
    Eigen::VectorXd result = Eigen::VectorXd::Zero(2 * len - 1);
    for (int i = 0; i < len; ++i) {
      for (int j = 0; j < len; ++j) {
        result(i + j) += coef(i) * coef(j);
      }
    }
    return result;
  }

  /**
   * @brief 多项式求值（Horner 法则的稳定实现）
   * @param coef 多项式系数向量
   * @param x 求值点
   * @return 多项式在 x 处的值
   */
  static double polyVal(const Eigen::VectorXd &coef, double x) {
    int len = coef.size();
    if (len <= 0)
      return 0.0;
    if (std::fabs(x) < DBL_EPSILON)
      return coef(len - 1);
    if (x == 1.0) {
      double sum = 0.0;
      for (int i = 0; i < len; ++i)
        sum += coef(i);
      return sum;
    }
    double result = 0.0;
    double xn = 1.0;
    for (int i = len - 1; i >= 0; --i) {
      result += coef(i) * xn;
      xn *= x;
    }
    return result;
  }

  /**
   * @brief 统计多项式在区间内的根的数量
   * @param coef 多项式系数向量
   * @param l 区间左端点
   * @param r 区间右端点
   * @return 根的数量
   */
  static int countRoots(const Eigen::VectorXd &coef, double l, double r) {
    int len = coef.size();
    if (len <= 1)
      return 0;

    // 使用 Sturm 序列计数
    std::set<double> roots = solvePolynomial(coef, l, r, DBL_EPSILON * 10);
    return static_cast<int>(roots.size());
  }

  /**
   * @brief 求解多项式的所有实根
   * @param coef 多项式系数向量
   * @param lbound 区间左端点
   * @param ubound 区间右端点
   * @param tol 容差
   * @return 所有实根的集合
   */
  static std::set<double> solvePolynomial(const Eigen::VectorXd &coef,
                                          double lbound, double ubound,
                                          double tol = FLT_EPSILON) {
    std::set<double> roots;
    int order = coef.size() - 1;

    if (order <= 0)
      return roots;
    if (order == 1) {
      // 一次方程
      if (std::fabs(coef(0)) > DBL_EPSILON) {
        double root = -coef(1) / coef(0);
        if (root > lbound && root < ubound)
          roots.insert(root);
      }
      return roots;
    }
    if (order == 2) {
      // 二次方程
      return solveQuadratic(coef(0), coef(1), coef(2), lbound, ubound);
    }
    if (order == 3) {
      // 三次方程
      return solveCubic(coef(0), coef(1), coef(2), coef(3), lbound, ubound);
    }

    // 高次多项式使用伴随矩阵法
    return solveByCompanionMatrix(coef, lbound, ubound, tol);
  }

private:
  /// @brief 求解二次方程
  static std::set<double> solveQuadratic(double a, double b, double c,
                                         double lbound, double ubound) {
    std::set<double> roots;
    if (std::fabs(a) < DBL_EPSILON) {
      // 退化为一次方程
      if (std::fabs(b) > DBL_EPSILON) {
        double root = -c / b;
        if (root > lbound && root < ubound)
          roots.insert(root);
      }
      return roots;
    }
    double disc = b * b - 4.0 * a * c;
    if (disc >= 0) {
      double sqrtDisc = std::sqrt(disc);
      double inv2a = 0.5 / a;
      double r1 = (-b + sqrtDisc) * inv2a;
      double r2 = (-b - sqrtDisc) * inv2a;
      if (r1 > lbound && r1 < ubound)
        roots.insert(r1);
      if (r2 > lbound && r2 < ubound)
        roots.insert(r2);
    }
    return roots;
  }

  /// @brief 求解三次方程（Cardano 公式）
  static std::set<double> solveCubic(double a, double b, double c, double d,
                                     double lbound, double ubound) {
    std::set<double> roots;

    if (std::fabs(a) < DBL_EPSILON) {
      return solveQuadratic(b, c, d, lbound, ubound);
    }

    // 标准化
    double invA = 1.0 / a;
    b *= invA;
    c *= invA;
    d *= invA;

    double b2 = b * b;
    double b3 = b2 * b;
    double p = c - b2 / 3.0;
    double q = 2.0 * b3 / 27.0 - b * c / 3.0 + d;
    double disc = q * q / 4.0 + p * p * p / 27.0;

    double bOver3 = b / 3.0;

    if (disc > DBL_EPSILON) {
      // 一个实根
      double sqrtDisc = std::sqrt(disc);
      double u = std::cbrt(-q / 2.0 + sqrtDisc);
      double v = std::cbrt(-q / 2.0 - sqrtDisc);
      double root = u + v - bOver3;
      if (root > lbound && root < ubound)
        roots.insert(root);
    } else if (disc < -DBL_EPSILON) {
      // 三个实根
      double r = std::sqrt(-p * p * p / 27.0);
      double theta = std::acos(-q / (2.0 * r));
      double cubeRootR = std::cbrt(r);
      double root1 = 2.0 * cubeRootR * std::cos(theta / 3.0) - bOver3;
      double root2 =
          2.0 * cubeRootR * std::cos((theta + 2.0 * M_PI) / 3.0) - bOver3;
      double root3 =
          2.0 * cubeRootR * std::cos((theta + 4.0 * M_PI) / 3.0) - bOver3;
      if (root1 > lbound && root1 < ubound)
        roots.insert(root1);
      if (root2 > lbound && root2 < ubound)
        roots.insert(root2);
      if (root3 > lbound && root3 < ubound)
        roots.insert(root3);
    } else {
      // 两个实根（重根）
      double u = std::cbrt(-q / 2.0);
      double root1 = 2.0 * u - bOver3;
      double root2 = -u - bOver3;
      if (root1 > lbound && root1 < ubound)
        roots.insert(root1);
      if (root2 > lbound && root2 < ubound)
        roots.insert(root2);
    }

    return roots;
  }

  /// @brief 使用伴随矩阵求解高次多项式
  static std::set<double> solveByCompanionMatrix(const Eigen::VectorXd &coef,
                                                 double lbound, double ubound,
                                                 double tol) {
    std::set<double> roots;
    int order = coef.size() - 1;

    if (order <= 0 || std::fabs(coef(0)) < DBL_EPSILON)
      return roots;

    // 构造 monic 多项式系数
    Eigen::VectorXd monicCoeffs(order + 1);
    monicCoeffs(0) = 1.0;
    for (int i = 1; i <= order; ++i) {
      monicCoeffs(i) = coef(i) / coef(0);
    }

    // 构造伴随矩阵
    Eigen::MatrixXd companionMat = Eigen::MatrixXd::Zero(order, order);
    companionMat(0, order - 1) = -monicCoeffs(order);
    for (int i = 1; i < order; ++i) {
      companionMat(i, i - 1) = 1.0;
      companionMat(i, order - 1) = -monicCoeffs(order - i);
    }

    // 计算特征值
    Eigen::VectorXcd eigenvalues = companionMat.eigenvalues();

    // 提取实根
    for (int i = 0; i < eigenvalues.size(); ++i) {
      if (std::fabs(eigenvalues(i).imag()) < tol) {
        double realPart = eigenvalues(i).real();
        if (realPart > lbound && realPart < ubound) {
          roots.insert(realPart);
        }
      }
    }

    return roots;
  }
};

} // namespace pb_minco

#endif // PB_MINCO_SMOOTHER__ROOT_FINDER_HPP_
