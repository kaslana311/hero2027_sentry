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
#include <tf2_ros/transform_broadcaster.h>

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
#include <small_gicp/registration/registration_helper.hpp>
#include <string>
#include <thread>
#include <vector>
// #include <small_gicp/pcl/pcl_registration.hpp>

class RelocalizationNode : public rclcpp::Node {
   public:
    RelocalizationNode();
    ~RelocalizationNode() = default;

   private:
    std::mutex mtx;

    void GlobalLocalizationTimerCallback();
    bool GlobalLocalization();

    void SaveCurOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void SaveCurScanCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void InitPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void PublishCropMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &crop_map);
    // 发布接受到的fast_lio的点云，在camera_init系下
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cur_scan_;
    // 参考系为map系，但点云是FOV视角中的
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crop_map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;

    // 订阅odom和fast_lio的点云数据
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
        SharedPtr sub_init_pose_;

    nav_msgs::msg::Odometry cur_odom_;
    // 分配内存需要在构造函数中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_ = nullptr;

    Eigen::Matrix4d pose_estimation_ = Eigen::Matrix4d::Identity();
    double init_x_;
    double init_y_;
    double init_z_;
    double init_yaw_;
    double init_pitch_;
    double init_roll_;
    std::string pcd_path_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool InitGlobalMap(std::string map_path);
    // 裁减FOV中的地图，目前写的是方形采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr CropMapInFov(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map,
        const Eigen::Matrix4d &pose_estimation,
        const nav_msgs::msg::Odometry &cur_odom, double FOV, double FOV_FAR);
    // voxel下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelDownSamping(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double leaf_size);
    std::vector<Eigen::Vector3d> ConvertPointCloudToEigenVector(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    Eigen::Isometry3d SmallIcpRegistration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_scan,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_map,
        const Eigen::Matrix4d &initial, double &fitness);
    Eigen::Matrix4d IcpRegistration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_scan,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_map,
        const Eigen::Matrix4d &initial, double scale, double &fitness);
    Eigen::Matrix4d PoseToMat(const geometry_msgs::msg::Pose &pose);
    geometry_msgs::msg::Pose MatToPose(const Eigen::Matrix4d &mat);
    Eigen::Matrix4d CreateTfMatrix(double x, double y, double z, double yaw,
                                   double pitch, double roll);
    double Deg2Rad(double deg);

    // Helpers for constrained registration and gating
    static inline double NormalizeAngle(double a) {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
    static inline double YawFromMat(const Eigen::Matrix4d &T) {
        const auto &R = T.block<3, 3>(0, 0);
        return std::atan2(R(1, 0), R(0, 0));
    }
    static inline double PitchFromMat(const Eigen::Matrix4d &T) {
        const auto &R = T.block<3, 3>(0, 0);
        // Eigen ZYX order: yaw(Z), pitch(Y), roll(X)
        Eigen::Vector3d eul = R.eulerAngles(2, 1, 0);
        return eul[1];
    }
    static inline double RollFromMat(const Eigen::Matrix4d &T) {
        const auto &R = T.block<3, 3>(0, 0);
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
    static inline void ZeroRollPitchKeepYaw(Eigen::Matrix4d &T) {
        double yaw = YawFromMat(T);
        T.block<3, 3>(0, 0) = RotZ(yaw).block<3, 3>(0, 0);
    }
    static inline double TransDelta(const Eigen::Matrix4d &A,
                                    const Eigen::Matrix4d &B) {
        Eigen::Vector3d da = A.block<3, 1>(0, 3);
        Eigen::Vector3d db = B.block<3, 1>(0, 3);
        return (da - db).norm();
    }
    static inline double YawDelta(const Eigen::Matrix4d &A,
                                  const Eigen::Matrix4d &B) {
        return std::abs(NormalizeAngle(YawFromMat(A) - YawFromMat(B)));
    }
    static inline double PitchDelta(const Eigen::Matrix4d &A,
                                    const Eigen::Matrix4d &B) {
        return std::abs(NormalizeAngle(PitchFromMat(A) - PitchFromMat(B)));
    }
    static inline double RollDelta(const Eigen::Matrix4d &A,
                                   const Eigen::Matrix4d &B) {
        return std::abs(NormalizeAngle(RollFromMat(A) - RollFromMat(B)));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_global_map_ = nullptr;

    bool has_global_map_ = false;
    bool has_odom_ = false;
    bool has_scan_ = false;

    bool first_registration_ = false;
    bool only_one_ = false;
    bool registration_success_ = false;

    // 配准参数
    int num_threads_ = 4;                           // 线程数
    double downsampling_resolution_ = 0.25;         // 降采样分辨率
    int num_neighbors_ = 10;                        // 邻居点数量
    double max_correspondence_distance_ = 1.0;      // 最大对应点距离
    double rotation_epsilon_ = 0.1 * M_PI / 180.0;  // 旋转收敛阈值
    double translation_epsilon_ = 1e-3;             // 平移收敛阈值
    int max_iterations_ = 20;                       // 最大迭代次数
    bool verbose_ = false;                          // 是否输出详细信息
    double max_fitness_threshold_ = 0.1;            // 最大允许的fitness分数

    // Gates and policies (populated from ROS parameters at runtime)
    double gate_yaw_deg_ = 5.0;                // yaw 门限（度）
    double gate_trans_m_ = 0.05;               // 平移门限（米）
    bool enforce_rp_zero_ = false;             // 默认不强制清零RP，改为门限约束
    double yaw_search_step_deg_ = 2.0;         // 粗搜步长（度）
    int max_fail_before_fallback_ = 5;         // 失败次数后触发兜底
    std::string fallback_policy_ = "initial";  // initial | identity
    bool use_crop_map_ = true;                 // 使用裁剪地图
    double crop_radius_ = 30.0;                // 裁剪半径（米）
    double fitness_threshold_ = 0.08;          // 接受阈值（与代码内一致）
    double gate_roll_deg_ = 5.0;               // roll 门限（度）
    double gate_pitch_deg_ = 5.0;              // pitch 门限（度）

    // ...existing code...
    int registration_fail_count_ = 0;  // 配准失败计数
                                       // ...existing code...
};