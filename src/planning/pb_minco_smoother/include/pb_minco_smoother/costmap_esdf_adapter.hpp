/**
 * @file costmap_esdf_adapter.hpp
 * @brief Costmap ESDF 适配器 - 基于 costmap 计算局部 ESDF
 *
 * 本文件实现了 ESDFInterface 接口，采用 "Snapshot on Demand" 模式：
 * 在每次轨迹平滑开始时，从 Nav2 的 Master Costmap 中截取路径周围的
 * 局部区域（ROI），使用 OpenCV 计算 ESDF，供 L-BFGS 优化过程使用。
 *
 * 设计理念：
 * 1. 单一真值来源：直接使用 Nav2 的 Master Costmap
 * 2. 按需快照：仅在优化前计算一次，避免锁竞争
 * 3. 高效查询：预计算 ESDF 后，getDistance/getGradient 为纯内存操作
 *
 * MIT License
 * Created by Lihan Chen, 2025
 */

#ifndef PB_MINCO_SMOOTHER__COSTMAP_ESDF_ADAPTER_HPP_
#define PB_MINCO_SMOOTHER__COSTMAP_ESDF_ADAPTER_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "pb_minco_smoother/minco_optimizer.hpp"

namespace pb_minco {

/**
 * @brief 基于 Costmap 的局部 ESDF 适配器
 *
 * 核心功能：
 * 1. updateSnapshot(): 根据路径计算 ROI，生成局部 ESDF
 * 2. getDistance(): 双线性插值查询距离（单位：米）
 * 3. getGradient(): 中心差分计算梯度（梯度指向远离障碍物的方向）
 *
 * 使用流程：
 * 1. 在 smooth() 函数开始时，调用 updateSnapshot(costmap, path_points)
 * 2. 将本对象传给优化器作为 ESDF 接口
 * 3. 优化器内部调用 getDistance/getGradient 进行避障代价计算
 */
class CostmapESDFAdapter : public ESDFInterface {
   public:
    /**
     * @brief 构造函数
     * @param logger 日志记录器
     * @param max_distance ESDF 最大距离（米），超过此距离的值将被截断
     * @param roi_margin ROI 外扩距离（米），确保路径周围有足够的梯度场
     */
    CostmapESDFAdapter(const rclcpp::Logger& logger, double max_distance = 5.0,
                       double roi_margin = 4.0)
        : logger_(logger),
          max_distance_(max_distance),
          roi_margin_(roi_margin) {
        RCLCPP_INFO(logger_,
                    "CostmapESDFAdapter 初始化完成: max_distance=%.2f m, "
                    "roi_margin=%.2f m",
                    max_distance_, roi_margin_);
    }

    /**
     * @brief 【核心函数】从 Costmap 截取局部区域并计算 ESDF
     *
     * 该函数实现了 "Snapshot on Demand" 模式的核心逻辑：
     * 1. 根据路径点计算 Bounding Box (ROI)
     * 2. 加锁并拷贝 ROI 内的 Costmap 数据
     * 3. 解锁后，独立计算 ESDF（不占用 Nav2 资源）
     * 4. 存储结果供后续 getDistance/getGradient 使用
     *
     * @param costmap Nav2 的 Master Costmap 指针
     * @param path_points 待平滑的路径点（世界坐标，用于计算 ROI）
     * @return true 如果成功更新 ESDF，false 如果失败
     */
    bool updateSnapshot(nav2_costmap_2d::Costmap2D* costmap,
                        const std::vector<Eigen::Vector2d>& path_points) {
        if (!costmap) {
            RCLCPP_WARN(logger_, "updateSnapshot 失败: costmap 为空");
            return false;
        }

        if (path_points.empty()) {
            RCLCPP_WARN(logger_, "updateSnapshot 失败: 路径点为空");
            return false;
        }

        // ========== Step 1: 计算路径的 Bounding Box ==========
        // 遍历所有路径点，找到最小和最大坐标
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto& pt : path_points) {
            min_x = std::min(min_x, pt.x());
            max_x = std::max(max_x, pt.x());
            min_y = std::min(min_y, pt.y());
            max_y = std::max(max_y, pt.y());
        }

        // 向外扩展 margin，确保梯度场覆盖优化可能的区域
        min_x -= roi_margin_;
        min_y -= roi_margin_;
        max_x += roi_margin_;
        max_y += roi_margin_;

        // ========== Step 2: 获取 Costmap 参数并计算 ROI 的栅格范围 ==========
        // 【关键】使用 Costmap 的互斥锁，确保读取期间数据一致
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap->getMutex()));

        // 保存全局 Costmap 参数（用于坐标转换）
        global_resolution_ = costmap->getResolution();
        global_origin_x_ = costmap->getOriginX();
        global_origin_y_ = costmap->getOriginY();
        unsigned int global_size_x = costmap->getSizeInCellsX();
        unsigned int global_size_y = costmap->getSizeInCellsY();

        // 将世界坐标的 ROI 边界转换为栅格索引
        unsigned int mx_min, my_min, mx_max, my_max;

        // 使用手动计算而非 worldToMap，因为 worldToMap 可能因边界外返回 false
        // 我们需要将其裁剪到有效范围
        mx_min = static_cast<unsigned int>(
            std::max(0.0, (min_x - global_origin_x_) / global_resolution_));
        my_min = static_cast<unsigned int>(
            std::max(0.0, (min_y - global_origin_y_) / global_resolution_));
        mx_max = static_cast<unsigned int>(
            std::max(0.0, (max_x - global_origin_x_) / global_resolution_));
        my_max = static_cast<unsigned int>(
            std::max(0.0, (max_y - global_origin_y_) / global_resolution_));

        // 裁剪到 Costmap 边界内
        mx_min = std::min(mx_min, global_size_x - 1);
        my_min = std::min(my_min, global_size_y - 1);
        mx_max = std::min(mx_max, global_size_x);
        my_max = std::min(my_max, global_size_y);

        // 确保 ROI 至少有 2x2 的大小
        if (mx_max <= mx_min + 1 || my_max <= my_min + 1) {
            lock.unlock();
            RCLCPP_WARN(logger_, "updateSnapshot 失败: ROI 过小");
            return false;
        }

        // ========== Step 3: 拷贝 ROI 内的 Costmap 数据 ==========
        // 计算局部地图的尺寸
        local_size_x_ = mx_max - mx_min;
        local_size_y_ = my_max - my_min;

        // 记录局部地图在世界坐标系中的原点
        // 这是局部地图 (0,0) 对应的世界坐标
        local_origin_x_ = global_origin_x_ + mx_min * global_resolution_;
        local_origin_y_ = global_origin_y_ + my_min * global_resolution_;
        resolution_ = global_resolution_;

        // 创建二值图像：障碍物=0（前景），自由空间=255（背景）
        // 复用成员变量，避免频繁内存分配
        if (obstacle_img_.rows != static_cast<int>(local_size_y_) ||
            obstacle_img_.cols != static_cast<int>(local_size_x_)) {
            obstacle_img_ = cv::Mat(local_size_y_, local_size_x_, CV_8UC1);
        }

        // 获取 Costmap 的原始数据指针（更高效）
        unsigned char* char_map = costmap->getCharMap();
        unsigned int stride = global_size_x;

        // 遍历 ROI 并拷贝数据
        for (unsigned int y = 0; y < local_size_y_; ++y) {
            for (unsigned int x = 0; x < local_size_x_; ++x) {
                // 计算在全局 Costmap 中的索引
                unsigned int global_idx = (my_min + y) * stride + (mx_min + x);
                unsigned char cost = char_map[global_idx];

                // 【关键决策】只将 LETHAL_OBSTACLE (254) 视为障碍物
                // 不包含 INSCRIBED_INFLATED_OBSTACLE (253)
                // 原因：优化器的 safe_distance 已经包含了机器人半径，
                //       如果这里也包含膨胀层，会导致双重膨胀
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
                    obstacle_img_.at<uchar>(y, x) = 0;  // 障碍物（前景）
                } else {
                    obstacle_img_.at<uchar>(y, x) = 255;  // 自由空间（背景）
                }
            }
        }

        // 【重要】尽早释放锁！后续的 EDT 计算不需要访问 Costmap
        lock.unlock();

        // ========== Step 4: 计算 ESDF (Euclidean Distance Transform)
        // ========== 使用 OpenCV 的距离变换，速度极快（100x100 通常 < 1ms）
        cv::distanceTransform(obstacle_img_, distance_img_, cv::DIST_L2,
                              cv::DIST_MASK_PRECISE);

        // ========== Step 5: 将结果存储到一维数组，便于快速查询 ==========
        // 复用 vector，避免频繁分配
        esdf_map_.resize(local_size_x_ * local_size_y_);

        for (unsigned int y = 0; y < local_size_y_; ++y) {
            for (unsigned int x = 0; x < local_size_x_; ++x) {
                // distance_img_ 存储的是像素距离，需要乘以分辨率转换为米
                float dist_pixels = distance_img_.at<float>(y, x);
                float dist_meters = dist_pixels * resolution_;
                // 截断到最大距离，节省存储空间并避免数值问题
                esdf_map_[y * local_size_x_ + x] =
                    std::min(dist_meters, static_cast<float>(max_distance_));
            }
        }

        has_valid_esdf_ = true;

        RCLCPP_DEBUG(logger_,
                     "ESDF 快照更新完成: 局部地图 %ux%u, 原点 (%.2f, %.2f), "
                     "分辨率 %.3f m",
                     local_size_x_, local_size_y_, local_origin_x_,
                     local_origin_y_, resolution_);

        return true;
    }

    /**
     * @brief 旧版接口：更新整个 Costmap 的 ESDF（保留兼容性）
     * @param costmap Costmap2D 指针
     * @deprecated 建议使用 updateSnapshot 以获得更好的性能
     */
    void updateESDF(const nav2_costmap_2d::Costmap2D* costmap) {
        if (!costmap) {
            RCLCPP_WARN(logger_, "Cannot update ESDF: costmap is null");
            return;
        }

        // 构建覆盖整个 Costmap 的虚拟路径点
        std::vector<Eigen::Vector2d> full_coverage;
        double ox = costmap->getOriginX();
        double oy = costmap->getOriginY();
        double sx = costmap->getSizeInMetersX();
        double sy = costmap->getSizeInMetersY();

        full_coverage.push_back(Eigen::Vector2d(ox, oy));
        full_coverage.push_back(Eigen::Vector2d(ox + sx, oy + sy));

        // 临时设置 margin 为 0，因为已经是全覆盖
        double old_margin = roi_margin_;
        roi_margin_ = 0.0;
        updateSnapshot(const_cast<nav2_costmap_2d::Costmap2D*>(costmap),
                       full_coverage);
        roi_margin_ = old_margin;
    }

    /**
     * @brief 【ESDFInterface】获取点 (x, y) 到最近障碍物的有符号距离
     *
     * 采用二次插值 (Quadratic Interpolation) 替代双线性插值。
     * 解决了在障碍物等距线（Voronoi 边）处梯度消失或震荡的问题。
     *
     * @param x 世界坐标 X
     * @param y 世界坐标 Y
     * @return 距离（米）
     */
    double getDistance(double x, double y) const override {
        if (!has_valid_esdf_) {
            return max_distance_;
        }
        // 使用二次插值获取距离
        double dist, grad_x, grad_y;
        interpolateQuadratic(x, y, dist, grad_x, grad_y,
                             false);  // false 表示不计算梯度
        return dist;
    }

    /**
     * @brief 【ESDFInterface】获取距离场在点 (x, y) 处的梯度
     *
     * 直接计算二次拟合曲面的解析梯度，保证了梯度的连续性和光滑性。
     *
     * @param x 世界坐标 X
     * @param y 世界坐标 Y
     * @return 梯度向量 (∂d/∂x, ∂d/∂y)
     */
    Eigen::Vector2d getGradient(double x, double y) const override {
        if (!has_valid_esdf_) {
            return Eigen::Vector2d::Zero();
        }

        double dist, grad_x, grad_y;
        interpolateQuadratic(x, y, dist, grad_x, grad_y,
                             true);  // true 表示计算梯度
        return Eigen::Vector2d(grad_x, grad_y);
    }

    /**
     * @brief 【ESDFInterface】检查点是否在局部地图范围内
     */
    bool isInside(double x, double y) const override {
        if (!has_valid_esdf_) {
            return false;
        }

        // 转换到局部地图坐标
        double local_x = (x - local_origin_x_) / resolution_;
        double local_y = (y - local_origin_y_) / resolution_;

        // 二次插值需要 3x3 邻域，所以边界需要留 1 个栅格
        return local_x >= 1 && local_x < local_size_x_ - 2 && local_y >= 1 &&
               local_y < local_size_y_ - 2;
    }

    /**
     * @brief 检查 ESDF 是否有效
     */
    bool hasValidESDF() const { return has_valid_esdf_; }

    /**
     * @brief 获取地图分辨率
     */
    double getResolution() const { return resolution_; }

    /**
     * @brief 设置 ROI 外扩距离
     */
    void setRoiMargin(double margin) { roi_margin_ = margin; }

   private:
    /**
     * @brief 二次插值核心算法
     *
     * 在 3x3 邻域内拟合二次曲面，同时计算函数值和解析梯度。
     *
     * @param x, y 世界坐标
     * @param[out] dist 输出距离
     * @param[out] grad_x 输出 X 梯度
     * @param[out] grad_y 输出 Y 梯度
     * @param compute_grad 是否计算梯度（优化性能）
     */
    void interpolateQuadratic(double x, double y, double& dist, double& grad_x,
                              double& grad_y, bool compute_grad) const {
        // 1. 转换到局部地图坐标
        double local_x = (x - local_origin_x_) / resolution_;
        double local_y = (y - local_origin_y_) / resolution_;

        // 2. 找到最近的整数中心点 (round)
        int ix = static_cast<int>(std::round(local_x));
        int iy = static_cast<int>(std::round(local_y));

        // 3. 边界保护：如果太靠近边界，回退到双线性插值或返回边界值
        if (ix < 1 || ix >= static_cast<int>(local_size_x_) - 1 || iy < 1 ||
            iy >= static_cast<int>(local_size_y_) - 1) {
            // 简单回退策略：直接取最近点的值，梯度设为0
            // 更好的做法是回退到双线性，但这里假设 ROI margin
            // 足够大，不会触碰边界
            unsigned int safe_x =
                std::clamp(ix, 0, static_cast<int>(local_size_x_) - 1);
            unsigned int safe_y =
                std::clamp(iy, 0, static_cast<int>(local_size_y_) - 1);
            dist = esdf_map_[safe_y * local_size_x_ + safe_x];
            grad_x = 0;
            grad_y = 0;
            return;
        }

        // 4. 计算相对于中心点的偏移 t_x, t_y ∈ [-0.5, 0.5]
        double tx = local_x - ix;
        double ty = local_y - iy;

        // 5. 获取 3x3 邻域的值
        // 索引：row * width + col
        // rows: iy-1, iy, iy+1
        // cols: ix-1, ix, ix+1

        // 预计算行偏移
        unsigned int row_up = (iy - 1) * local_size_x_;
        unsigned int row_mid = iy * local_size_x_;
        unsigned int row_down = (iy + 1) * local_size_x_;

        // 读取 3x3 矩阵
        double v00 = esdf_map_[row_up + ix - 1];
        double v01 = esdf_map_[row_up + ix];
        double v02 = esdf_map_[row_up + ix + 1];

        double v10 = esdf_map_[row_mid + ix - 1];
        double v11 = esdf_map_[row_mid + ix];
        double v12 = esdf_map_[row_mid + ix + 1];

        double v20 = esdf_map_[row_down + ix - 1];
        double v21 = esdf_map_[row_down + ix];
        double v22 = esdf_map_[row_down + ix + 1];

        // 6. 在 X 方向进行三次 1D 二次插值，得到三行在 tx 处的值
        // 公式：f(t) = 0.5 * (y0 + y2 - 2y1) * t^2 + 0.5 * (y2 - y0) * t + y1
        // 记 a = 0.5 * (y0 + y2 - 2y1), b = 0.5 * (y2 - y0), c = y1

        // Row 0 (up)
        double a0 = 0.5 * (v00 + v02 - 2 * v01);
        double b0 = 0.5 * (v02 - v00);
        double c0 = v01;
        double val_y0 = a0 * tx * tx + b0 * tx + c0;

        // Row 1 (mid)
        double a1 = 0.5 * (v10 + v12 - 2 * v11);
        double b1 = 0.5 * (v12 - v10);
        double c1 = v11;
        double val_y1 = a1 * tx * tx + b1 * tx + c1;

        // Row 2 (down)
        double a2 = 0.5 * (v20 + v22 - 2 * v21);
        double b2 = 0.5 * (v22 - v20);
        double c2 = v21;
        double val_y2 = a2 * tx * tx + b2 * tx + c2;

        // 7. 在 Y 方向进行一次 1D 二次插值，得到最终距离
        double ay = 0.5 * (val_y0 + val_y2 - 2 * val_y1);
        double by = 0.5 * (val_y2 - val_y0);
        double cy = val_y1;

        dist = ay * ty * ty + by * ty + cy;

        // 8. 计算梯度 (如果需要)
        if (compute_grad) {
            // Y 方向导数：∂f/∂y = 2 * ay * ty + by
            // 注意：这是相对于栅格单位的导数，需要除以分辨率得到相对于米的导数
            double d_dy_grid = 2 * ay * ty + by;
            grad_y = d_dy_grid / resolution_;

            // X 方向导数：需要链式法则或重新插值
            // 方法：先计算三行在 X 方向的导数，再在 Y 方向插值这些导数
            double d_dx_row0 = 2 * a0 * tx + b0;
            double d_dx_row1 = 2 * a1 * tx + b1;
            double d_dx_row2 = 2 * a2 * tx + b2;

            // 在 Y 方向插值 X 导数
            // 使用线性插值还是二次插值？为了平滑，建议继续用二次插值
            double ax_grad = 0.5 * (d_dx_row0 + d_dx_row2 - 2 * d_dx_row1);
            double bx_grad = 0.5 * (d_dx_row2 - d_dx_row0);
            double cx_grad = d_dx_row1;

            double d_dx_grid = ax_grad * ty * ty + bx_grad * ty + cx_grad;
            grad_x = d_dx_grid / resolution_;
        }
    }

   private:
    rclcpp::Logger logger_;

    // ========== 配置参数 ==========
    double max_distance_;  ///< ESDF 最大距离（米）
    double roi_margin_;    ///< ROI 外扩距离（米）

    // ========== 全局 Costmap 参数（用于调试） ==========
    double global_resolution_ = 0.0;
    double global_origin_x_ = 0.0;
    double global_origin_y_ = 0.0;

    // ========== 局部 ESDF 数据 ==========
    bool has_valid_esdf_ = false;
    std::vector<float> esdf_map_;  ///< 一维数组存储，索引 = y * size_x + x
    unsigned int local_size_x_ = 0;  ///< 局部地图宽度（栅格数）
    unsigned int local_size_y_ = 0;  ///< 局部地图高度（栅格数）
    double resolution_ = 0.0;        ///< 分辨率（米/栅格）
    double local_origin_x_ = 0.0;    ///< 局部地图原点 X（世界坐标）
    double local_origin_y_ = 0.0;    ///< 局部地图原点 Y（世界坐标）

    // ========== OpenCV 图像缓存（复用以避免频繁分配） ==========
    cv::Mat obstacle_img_;  ///< 二值障碍物图像
    cv::Mat distance_img_;  ///< 距离变换结果
};

}  // namespace pb_minco

#endif  // PB_MINCO_SMOOTHER__COSTMAP_ESDF_ADAPTER_HPP_
