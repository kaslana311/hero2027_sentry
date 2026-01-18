#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <string>
#include <thread>
#include <vector>

// #include <small_gicp/pcl/pcl_registration.hpp>
using namespace small_gicp;
class RelocalizationNode : public rclcpp::Node {
   public:
    RelocalizationNode();
    ~RelocalizationNode() = default;

   private:
    void GlobalLocalizationTimerCallback();  //定位

    void SaveCurScanCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);  //保存积累点云
    void publishTransform();                                  //发布tf
    void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                           double min_z, double max_z, double min_y,
                           double max_y, double min_x, double max_x);  //滤波
    bool InitGlobalMap(std::string map_path);
    Eigen::Isometry3d SmallIcpRegistration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_scan, Eigen::Matrix4d pre,
        double& fitness);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    pcl::CropBox<pcl::PointXYZ> crop;
    //-------------------------------data---------------------------------//
    std::mutex mtx, mtx1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_ = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;  //累计点云
    std::string pcd_path_;
    // rclcpp::CallbackGroup::SharedPtr timer_group_;
    // rclcpp::CallbackGroup::SharedPtr sub_group_;
    rclcpp::TimerBase::SharedPtr timer_, timer_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ = nullptr;  // global_load

    rclcpp::CallbackGroup::SharedPtr pub_gp;
    rclcpp::CallbackGroup::SharedPtr lc_gp;

    bool has_global_map_ = false;
    bool has_scan_ = false;
    bool registration_success_ = false;

    // Gates and policies (populated from ROS parameters at runtime)
    double gate_yaw_deg_ = 5.0;    // yaw 门限（度）
    double gate_roll_deg_ = 5.0;   // roll 门限（度）
    double gate_pitch_deg_ = 5.0;  // pitch 门限（度）
    double gate_trans_m_ = 0.1;    // 平移门限（米）

    bool enforce_rp_zero_ = false;  // 默认不强制清零RP，改为门限约束
    double yaw_search_step_deg_ = 2.0;         // 粗搜步长（度）
    int max_fail_before_fallback_ = 5;         // 失败次数后触发兜底
    std::string fallback_policy_ = "initial";  // initial | identity
    double fitness_threshold_ = 0.08;  // 接受阈值（与代码内一致）
    int registration_fail_count_ = 0;  // 配准失败计数
    double max_fitness_threshold_ = 0.1;  // 最大允许的fitness分数

    // small_gicp参数------------------
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor,
                                             small_gicp::ParallelReductionOMP>>
        register_;                                       //重定位
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;  //先验地图
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;  //扫描
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
        target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>
        source_tree_;  // kd树
    double smgicp_global_leaf_size_, smgicp_local_leaf_size_;
    int smgicp_max_iterations_;
    int smgicp_num_threads_;
    double smgicp_max_dist_sq_;
    int nebhor_num;
    //当前累计点云帧数目
    int scan_count = 0;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Eigen::Isometry3d result_t_;  // small_gicp result
    Eigen::Matrix4d pose_estimation_;
    std::vector<double> init_pose_;
    bool first_lidar_pos = false;
    geometry_msgs::msg::TransformStamped transform_stampedlidarodom;

    //test 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr test_pub;
    //---------------------inline func----------------------------//
    // Helpers for constrained registration and gating
    static inline double NormalizeAngle(double a) {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
    static inline double YawFromMat(const Eigen::Matrix4d& T) {
        const auto& R = T.block<3, 3>(0, 0);
        return std::atan2(R(1, 0), R(0, 0));
    }
    static inline double PitchFromMat(const Eigen::Matrix4d& T) {
        const auto& R = T.block<3, 3>(0, 0);
        // Eigen ZYX order: yaw(Z), pitch(Y), roll(X)
        Eigen::Vector3d eul = R.eulerAngles(2, 1, 0);
        return eul[1];
    }
    static inline double RollFromMat(const Eigen::Matrix4d& T) {
        const auto& R = T.block<3, 3>(0, 0);
        Eigen::Vector3d eul = R.eulerAngles(2, 1, 0);
        return eul[2];
    }
    static inline Eigen::Matrix4d RotZ(double yaw) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        double c = std::cos(yaw), s = std::sin(yaw);
        T(0, 0) = c;
        T(0, 1) = -s;
        T(1, 0) = s;
        T(1, 1) = c;
        return T;
    }
    static inline void ZeroRollPitchKeepYaw(Eigen::Matrix4d& T) {
        double yaw = YawFromMat(T);
        T.block<3, 3>(0, 0) = RotZ(yaw).block<3, 3>(0, 0);
    }
    static inline double TransDelta(const Eigen::Matrix4d& A,
                                    const Eigen::Matrix4d& B) {
        Eigen::Vector3d da = A.block<3, 1>(0, 3);
        Eigen::Vector3d db = B.block<3, 1>(0, 3);
        return (da - db).norm();
    }
    static inline double YawDelta(const Eigen::Matrix4d& A,
                                  const Eigen::Matrix4d& B) {
        return std::abs(NormalizeAngle(YawFromMat(A) - YawFromMat(B)));
    }
    static inline double PitchDelta(const Eigen::Matrix4d& A,
                                    const Eigen::Matrix4d& B) {
        double deg =
            std::abs(NormalizeAngle(PitchFromMat(A) - PitchFromMat(B)));
        if (deg > 175.0 * M_PI / 180.0) deg = std::abs(M_PI - deg);
        return deg;
    }
    static inline double RollDelta(const Eigen::Matrix4d& A,
                                   const Eigen::Matrix4d& B) {
        double deg = std::abs(NormalizeAngle(RollFromMat(A) - RollFromMat(B)));
        if (deg > 175.0 * M_PI / 180.0) deg = std::abs(M_PI - deg);
        return deg;
    }
};