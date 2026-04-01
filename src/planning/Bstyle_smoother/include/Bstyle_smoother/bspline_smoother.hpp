/**
 * @file bspline_smoother.hpp
 * @brief Nav2 B-spline 轨迹平滑器插件
 *
 * 实现 nav2_core::Smoother 接口
 * 使用 B-spline 优化方法平滑路径
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Bstyle_smoother/bspline_optimizer.h"
#include "Bstyle_smoother/non_uniform_bspline.h"
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bspline_smoother {

/**
 * @brief B-spline 平滑器类
 *
 * 使用 L-BFGS 优化算法对 B-spline 控制点进行优化
 * 考虑以下代价项:
 * - 平滑性代价 (最小化加加速度/加速度)
 * - 碰撞代价 (远离障碍物)
 * - 可行性代价 (速度/加速度限制)
 */
class BSplineSmoother : public nav2_core::Smoother {
   public:
    BSplineSmoother() = default;
    ~BSplineSmoother() override = default;

    /**
     * @brief 配置平滑器
     */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
        override;

    /**
     * @brief 清理资源
     */
    void cleanup() override;

    /**
     * @brief 激活平滑器
     */
    void activate() override;

    /**
     * @brief 停用平滑器
     */
    void deactivate() override;

    /**
     * @brief 平滑路径
     *
     * @param path 输入/输出路径
     * @param max_time 最大执行时间
     * @return true 成功
     */
    bool smooth(nav_msgs::msg::Path& path,
                const rclcpp::Duration& max_time) override;

   protected:
    /**
     * @brief 将 Path 转换为 B-spline 控制点
     *
     * @param path 输入路径
     * @param ctrl_pts 输出控制点 (N x 2)
     * @param interval 节点间隔
     */
    void pathToBspline(const nav_msgs::msg::Path& path,
                       Eigen::MatrixXd& ctrl_pts, double& interval);

    /**
     * @brief 将 B-spline 控制点转换回 Path
     *
     * @param bspline B-spline 曲线
     * @param path 输出路径
     * @param original_path 原始路径 (用于获取 header 和采样密度)
     */
    void bsplineToPath(const fast_planner::NonUniformBspline& bspline,
                       nav_msgs::msg::Path& path,
                       const nav_msgs::msg::Path& original_path);

    /**
     * @brief 优化 B-spline 控制点
     */
    bool optimizeBspline(Eigen::MatrixXd& ctrl_pts, double interval);

    // 节点和参数
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string name_;
    rclcpp::Logger logger_{rclcpp::get_logger("BSplineSmoother")};
    rclcpp::Clock::SharedPtr clock_;

    // 优化器
    fast_planner::BsplineOptimizer optimizer_;
    fast_planner::BsplineConfig config_bspline_;

    // TF 和 Costmap
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    nav2_costmap_2d::Costmap2D* costmap_{nullptr};

    // 参数
    int bspline_order_{4};
    double lambda_smooth_{1.0};
    double lambda_collision_{0.5};
    double lambda_feasibility_{0.1};
    double dist_threshold_{0.4};
    double max_vel_{1.0};
    double max_acc_{2.0};
    int max_iterations_{100};
    double min_point_spacing_{0.05};
};

}  // namespace nav2_bspline_smoother
