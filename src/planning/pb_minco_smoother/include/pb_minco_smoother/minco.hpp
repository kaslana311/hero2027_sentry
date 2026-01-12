/**
 * @file minco.hpp
 * @brief 2D MINCO (Minimum Control) 轨迹优化器
 *
 * 基于 GCOPTER (ZJU-FAST-Lab) 的 minco.hpp 修改
 * 原版为 3D 无人机轨迹，本版本改为 2D 地面机器人轨迹
 *
 * MINCO 是一种闭式轨迹优化方法，通过约束传递将中间路点的约束隐式编码到
 * 多项式系数中，从而实现高效的优化。
 *
 * 主要修改：
 * 1. 将所有 Eigen::Matrix<double, 3, X> 替换为 Eigen::Matrix<double, 2, X>
 * 2. 将所有 Eigen::MatrixX3d 替换为 Eigen::MatrixX2d
 * 3. 将所有 Eigen::Matrix3Xd 替换为 Eigen::Matrix2Xd
 * 4. 仅保留 MINCO_S2NU (s=2, 最小化 Jerk) 用于地面机器人
 *
 * 数学背景：
 * - s=2 意味着最小化轨迹的 Jerk (加加速度) 能量
 * - 对应多项式阶数为 D=3 (三次多项式)
 * - 每段轨迹有 4 个系数，共 4N 个未知数 (N 是段数)
 *
 * MIT License
 * Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)
 * Modified for 2D by Lihan Chen, 2025
 */

#ifndef PB_MINCO_SMOOTHER__MINCO_HPP_
#define PB_MINCO_SMOOTHER__MINCO_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

#include "pb_minco_smoother/trajectory.hpp"

namespace pb_minco {

/**
 * @brief 带状矩阵线性方程组求解器
 *
 * 用于高效求解 MINCO 的线性方程组 Ax = b
 * 利用带状矩阵的稀疏结构，LU 分解复杂度为 O(N)
 *
 * 带状矩阵 A 是 N×N 矩阵，下带宽为 lowerBw，上带宽为 upperBw
 * 对于 MINCO s=2：带宽均为 4
 */
class BandedSystem {
   public:
    /**
     * @brief 创建带状矩阵
     * @param n 矩阵大小
     * @param p 下带宽
     * @param q 上带宽
     */
    inline void create(int n, int p, int q) {
        destroy();
        N_ = n;
        lowerBw_ = p;
        upperBw_ = q;
        int actualSize = N_ * (lowerBw_ + upperBw_ + 1);
        ptrData_ = new double[actualSize];
        std::fill_n(ptrData_, actualSize, 0.0);
    }

    /// @brief 销毁矩阵
    inline void destroy() {
        if (ptrData_ != nullptr) {
            delete[] ptrData_;
            ptrData_ = nullptr;
        }
    }

    /// @brief 重置矩阵为零
    inline void reset() {
        std::fill_n(ptrData_, N_ * (lowerBw_ + upperBw_ + 1), 0.0);
    }

    /// @brief 获取矩阵元素 (常量版本)
    inline const double &operator()(int i, int j) const {
        return ptrData_[(i - j + upperBw_) * N_ + j];
    }

    /// @brief 获取矩阵元素 (可修改版本)
    inline double &operator()(int i, int j) {
        return ptrData_[(i - j + upperBw_) * N_ + j];
    }

    /**
     * @brief 就地 LU 分解
     *
     * 注意：为了效率，不进行主元选取 (pivoting)
     * 对于 MINCO 的正定矩阵，这是安全的
     */
    inline void factorizeLU() {
        int iM, jM;
        double cVl;
        for (int k = 0; k <= N_ - 2; k++) {
            iM = std::min(k + lowerBw_, N_ - 1);
            cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; i++) {
                if (operator()(i, k) != 0.0) {
                    operator()(i, k) /= cVl;
                }
            }
            jM = std::min(k + upperBw_, N_ - 1);
            for (int j = k + 1; j <= jM; j++) {
                cVl = operator()(k, j);
                if (cVl != 0.0) {
                    for (int i = k + 1; i <= iM; i++) {
                        if (operator()(i, k) != 0.0) {
                            operator()(i, j) -= operator()(i, k) * cVl;
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief 求解 Ax = b，结果存储在 b 中
     * @tparam EIGENMAT 矩阵类型（支持行操作）
     * @param b 输入为右侧向量，输出为解
     */
    template <typename EIGENMAT>
    inline void solve(EIGENMAT &b) const {
        int iM;
        // 前向替换
        for (int j = 0; j <= N_ - 1; j++) {
            iM = std::min(j + lowerBw_, N_ - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        // 后向替换
        for (int j = N_ - 1; j >= 0; j--) {
            b.row(j) /= operator()(j, j);
            iM = std::max(0, j - upperBw_);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
    }

    /**
     * @brief 求解 A^T x = b（伴随方程），结果存储在 b 中
     *
     * 用于梯度反向传播
     */
    template <typename EIGENMAT>
    inline void solveAdj(EIGENMAT &b) const {
        int iM;
        for (int j = 0; j <= N_ - 1; j++) {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw_, N_ - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        for (int j = N_ - 1; j >= 0; j--) {
            iM = std::max(0, j - lowerBw_);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
    }

   private:
    int N_ = 0;
    int lowerBw_ = 0;
    int upperBw_ = 0;
    double *ptrData_ = nullptr;
};

/**
 * @brief MINCO s=2 非均匀时间版本 (2D)
 *
 * s=2 意味着最小化 Jerk 能量，对应三次多项式轨迹
 *
 * 输入：
 * - 起始状态 (位置, 速度)
 * - 终止状态 (位置, 速度)
 * - 中间路点 (N-1 个点，其中 N 是轨迹段数)
 * - 每段时间 (N 个时间)
 *
 * 输出：
 * - N 段三次多项式轨迹
 *
 * 特性：
 * - 自动保证位置、速度、加速度连续
 * - 提供能量及其对路点和时间的梯度
 *
 * 性能优化：
 * - 所有临时变量预分配，运行时零动态内存分配
 * - 注意：此类非线程安全，预分配的缓冲区在多线程下会产生竞态条件
 */
class MINCO_S2NU {
   public:
    MINCO_S2NU() = default;
    ~MINCO_S2NU() { A_.destroy(); }

    // 禁止拷贝（因为有原始指针）
    MINCO_S2NU(const MINCO_S2NU &) = delete;
    MINCO_S2NU &operator=(const MINCO_S2NU &) = delete;

    // 允许移动
    MINCO_S2NU(MINCO_S2NU &&other)
    noexcept
        : N_(other.N_),
          headPV_(std::move(other.headPV_)),
          tailPV_(std::move(other.tailPV_)),
          b_(std::move(other.b_)),
          T1_(std::move(other.T1_)),
          T2_(std::move(other.T2_)),
          T3_(std::move(other.T3_)),
          adjGrad_(std::move(other.adjGrad_)) {
        A_.destroy();
        std::swap(A_, other.A_);
    }

    /**
     * @brief 设置边界条件并预分配所有内部缓冲区
     * @param headState 起始状态 [位置, 速度] (2x2 矩阵)
     * @param tailState 终止状态 [位置, 速度] (2x2 矩阵)
     * @param pieceNum 轨迹段数 N
     *
     * 此函数会预分配所有运行时需要的内存，之后的调用将不再触发动态分配
     */
    inline void setConditions(const Eigen::Matrix<double, 2, 2> &headState,
                              const Eigen::Matrix<double, 2, 2> &tailState,
                              int pieceNum) {
        N_ = pieceNum;
        headPV_ = headState;
        tailPV_ = tailState;
        // 每段 4 个系数，共 4N 个未知数
        // 带宽为 4（来自连续性约束的结构）
        A_.create(4 * N_, 4, 4);
        b_.resize(4 * N_, 2);
        T1_.resize(N_);
        T2_.resize(N_);
        T3_.resize(N_);

        // ========== 预分配梯度传播所需的缓冲区 ==========
        adjGrad_.resize(4 * N_, 2);
    }

    /**
     * @brief 设置参数并求解轨迹系数
     * @param inPs 中间路点 (2 x (N-1) 矩阵)
     * @param ts 每段时间向量 (N 维)
     *
     * 这个函数构建并求解 MINCO 的线性方程组：
     * 1. 起始边界条件 (2 个方程)
     * 2. 中间连续性约束 (4(N-1) 个方程)
     * 3. 终止边界条件 (2 个方程)
     *
     * 共 4N 个方程求解 4N 个系数
     */
    inline void setParameters(const Eigen::Matrix2Xd &inPs,
                              const Eigen::VectorXd &ts) {
        // 预计算时间的幂次
        T1_ = ts;
        T2_ = T1_.cwiseProduct(T1_);
        T3_ = T2_.cwiseProduct(T1_);

        A_.reset();
        b_.setZero();

        // ========== 起始边界条件 ==========
        // 位置: c0 = p0
        A_(0, 0) = 1.0;
        // 速度: c1 = v0
        A_(1, 1) = 1.0;
        b_.row(0) = headPV_.col(0).transpose();
        b_.row(1) = headPV_.col(1).transpose();

        // ========== 中间连续性约束 ==========
        for (int i = 0; i < N_ - 1; i++) {
            // 加速度连续: 2*c2_i + 6*T*c3_i = 2*c2_{i+1}
            A_(4 * i + 2, 4 * i + 2) = 2.0;
            A_(4 * i + 2, 4 * i + 3) = 6.0 * T1_(i);
            A_(4 * i + 2, 4 * i + 6) = -2.0;

            // 位置等于路点: p_i(T) = q_i
            A_(4 * i + 3, 4 * i) = 1.0;
            A_(4 * i + 3, 4 * i + 1) = T1_(i);
            A_(4 * i + 3, 4 * i + 2) = T2_(i);
            A_(4 * i + 3, 4 * i + 3) = T3_(i);

            // 位置连续: p_i(T) = p_{i+1}(0)
            A_(4 * i + 4, 4 * i) = 1.0;
            A_(4 * i + 4, 4 * i + 1) = T1_(i);
            A_(4 * i + 4, 4 * i + 2) = T2_(i);
            A_(4 * i + 4, 4 * i + 3) = T3_(i);
            A_(4 * i + 4, 4 * i + 4) = -1.0;

            // 速度连续: v_i(T) = v_{i+1}(0)
            A_(4 * i + 5, 4 * i + 1) = 1.0;
            A_(4 * i + 5, 4 * i + 2) = 2.0 * T1_(i);
            A_(4 * i + 5, 4 * i + 3) = 3.0 * T2_(i);
            A_(4 * i + 5, 4 * i + 5) = -1.0;

            // 右侧向量：路点位置
            b_.row(4 * i + 3) = inPs.col(i).transpose();
        }

        // ========== 终止边界条件 ==========
        // 终止位置: p_{N-1}(T) = pf
        A_(4 * N_ - 2, 4 * N_ - 4) = 1.0;
        A_(4 * N_ - 2, 4 * N_ - 3) = T1_(N_ - 1);
        A_(4 * N_ - 2, 4 * N_ - 2) = T2_(N_ - 1);
        A_(4 * N_ - 2, 4 * N_ - 1) = T3_(N_ - 1);

        // 终止速度: v_{N-1}(T) = vf
        A_(4 * N_ - 1, 4 * N_ - 3) = 1.0;
        A_(4 * N_ - 1, 4 * N_ - 2) = 2.0 * T1_(N_ - 1);
        A_(4 * N_ - 1, 4 * N_ - 1) = 3.0 * T2_(N_ - 1);

        b_.row(4 * N_ - 2) = tailPV_.col(0).transpose();
        b_.row(4 * N_ - 1) = tailPV_.col(1).transpose();

        // ========== LU 分解并求解 ==========
        A_.factorizeLU();
        A_.solve(b_);
    }

    /**
     * @brief 获取求解后的轨迹
     * @param traj 输出轨迹
     */
    inline void getTrajectory(Trajectory<3> &traj) const {
        traj.clear();
        traj.reserve(N_);
        for (int i = 0; i < N_; i++) {
            // 系数矩阵布局转换：
            // b_ 存储为 4Nx2，每 4 行是一段
            // 需要转换为 Piece 需要的 2x4 格式
            // 并反转列顺序（因为多项式系数顺序）
            traj.emplace_back(
                T1_(i),
                b_.block<4, 2>(4 * i, 0).transpose().rowwise().reverse());
        }
    }

    /**
     * @brief 获取 Jerk 能量
     * @param energy 输出能量值
     *
     * 能量公式：E = ∫ ||j(t)||^2 dt
     * 对于三次多项式 p(t) = c0 + c1*t + c2*t^2 + c3*t^3
     * Jerk j(t) = 6*c3
     * 但由于连续性约束，完整公式涉及 c2 和 c3
     */
    inline void getEnergy(double &energy) const {
        energy = 0.0;
        for (int i = 0; i < N_; i++) {
            // Jerk 能量的解析积分
            energy += 4.0 * b_.row(4 * i + 2).squaredNorm() * T1_(i) +
                      12.0 * b_.row(4 * i + 2).dot(b_.row(4 * i + 3)) * T2_(i) +
                      12.0 * b_.row(4 * i + 3).squaredNorm() * T3_(i);
        }
    }

    /// @brief 获取系数矩阵（只读）
    inline const Eigen::MatrixX2d &getCoeffs() const { return b_; }

    /**
     * @brief 获取能量对系数的偏导数
     * @param gdC 输出梯度矩阵 (4N x 2)，必须已预分配正确大小
     *
     * 用于链式法则的中间变量
     * 注意：调用前必须确保 gdC 已经 resize 到 (4*N_, 2)
     */
    inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX2d &gdC) const {
        assert(gdC.rows() == 4 * N_ && gdC.cols() == 2 &&
               "gdC must be pre-allocated to (4*N, 2)");
        for (int i = 0; i < N_; i++) {
            gdC.row(4 * i + 3) = 12.0 * b_.row(4 * i + 2) * T2_(i) +
                                 24.0 * b_.row(4 * i + 3) * T3_(i);
            gdC.row(4 * i + 2) = 8.0 * b_.row(4 * i + 2) * T1_(i) +
                                 12.0 * b_.row(4 * i + 3) * T2_(i);
            gdC.row(4 * i).setZero();
            gdC.row(4 * i + 1).setZero();
        }
    }

    /**
     * @brief 获取能量对时间的偏导数
     * @param gdT 输出梯度向量 (N 维)，必须已预分配正确大小
     *
     * 注意：调用前必须确保 gdT 已经 resize 到 N_
     */
    inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const {
        assert(gdT.size() == N_ && "gdT must be pre-allocated to N");
        for (int i = 0; i < N_; i++) {
            gdT(i) = 4.0 * b_.row(4 * i + 2).squaredNorm() +
                     24.0 * b_.row(4 * i + 2).dot(b_.row(4 * i + 3)) * T1_(i) +
                     36.0 * b_.row(4 * i + 3).squaredNorm() * T2_(i);
        }
    }

    /**
     * @brief 梯度传播：将对系数/时间的偏导数转换为对路点/时间的梯度
     *
     * @param partialGradByCoeffs 对系数的偏导数 (4N x 2)
     * @param partialGradByTimes 对时间的偏导数 (N 维)
     * @param gradByPoints 输出：对路点的梯度 (2 x (N-1))，必须已预分配
     * @param gradByTimes 输出：对时间的梯度 (N 维)，必须已预分配
     *
     * 这是 MINCO 的核心：利用隐函数定理，将系数的梯度转换为路点的梯度
     * 需要求解伴随方程 A^T * adjGrad = partialGradByCoeffs
     */
    inline void propogateGrad(const Eigen::MatrixX2d &partialGradByCoeffs,
                              const Eigen::VectorXd &partialGradByTimes,
                              Eigen::Matrix2Xd &gradByPoints,
                              Eigen::VectorXd &gradByTimes) {
        assert(gradByPoints.cols() == N_ - 1 && gradByPoints.rows() == 2 &&
               "gradByPoints must be pre-allocated to (2, N-1)");
        assert(gradByTimes.size() == N_ &&
               "gradByTimes must be pre-allocated to N");

        // 使用预分配的缓冲区求解伴随方程
        adjGrad_ = partialGradByCoeffs;
        A_.solveAdj(adjGrad_);

        // 提取路点梯度（对应于 b 中路点约束的行）
        for (int i = 0; i < N_ - 1; i++) {
            gradByPoints.col(i) = adjGrad_.row(4 * i + 3).transpose();
        }

        // 计算时间梯度（需要考虑约束矩阵对时间的依赖）
        // 使用固定大小的栈变量，无需动态分配
        Eigen::Matrix<double, 4, 2> B1;
        Eigen::Matrix<double, 2, 2> B2;

        for (int i = 0; i < N_ - 1; i++) {
            // 负 Jerk 贡献
            B1.row(0) = -6.0 * b_.row(i * 4 + 3);

            // 负速度贡献
            B1.row(1) = -(b_.row(i * 4 + 1) + 2.0 * T1_(i) * b_.row(i * 4 + 2) +
                          3.0 * T2_(i) * b_.row(i * 4 + 3));
            B1.row(2) = B1.row(1);

            // 负加速度贡献
            B1.row(3) =
                -(2.0 * b_.row(i * 4 + 2) + 6.0 * T1_(i) * b_.row(i * 4 + 3));

            gradByTimes(i) =
                B1.cwiseProduct(adjGrad_.block<4, 2>(4 * i + 2, 0)).sum();
        }

        // 最后一段的时间梯度
        // 负速度贡献
        B2.row(0) =
            -(b_.row(4 * N_ - 3) + 2.0 * T1_(N_ - 1) * b_.row(4 * N_ - 2) +
              3.0 * T2_(N_ - 1) * b_.row(4 * N_ - 1));

        // 负加速度贡献
        B2.row(1) = -(2.0 * b_.row(4 * N_ - 2) +
                      6.0 * T1_(N_ - 1) * b_.row(4 * N_ - 1));

        gradByTimes(N_ - 1) =
            B2.cwiseProduct(adjGrad_.block<2, 2>(4 * N_ - 2, 0)).sum();

        // 加上直接的时间偏导数
        gradByTimes += partialGradByTimes;
    }

    /// @brief 获取轨迹段数
    inline int getPieceNum() const { return N_; }

    /// @brief 获取时间向量
    inline const Eigen::VectorXd &getTimes() const { return T1_; }

   private:
    int N_ = 0;  ///< 轨迹段数

    /// 边界状态：每列分别是位置和速度
    Eigen::Matrix<double, 2, 2> headPV_;
    Eigen::Matrix<double, 2, 2> tailPV_;

    BandedSystem A_;      ///< 带状系数矩阵
    Eigen::MatrixX2d b_;  ///< 右侧向量 / 求解后的系数矩阵

    /// 时间的幂次，用于避免重复计算
    Eigen::VectorXd T1_;  ///< T
    Eigen::VectorXd T2_;  ///< T^2
    Eigen::VectorXd T3_;  ///< T^3

    // ========== 预分配缓冲区 (Pre-allocated Buffers) ==========
    // 用于 propogateGrad 函数，避免运行时动态内存分配
    // 注意：这些缓冲区使此类非线程安全
    mutable Eigen::MatrixX2d adjGrad_;  ///< 伴随梯度缓冲区 (4N x 2)
};

}  // namespace pb_minco

#endif  // PB_MINCO_SMOOTHER__MINCO_HPP_
