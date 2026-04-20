#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <QObject>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "tf2_eigen/tf2_eigen.hpp"
#include "util.hpp"

class RelocalizationNode : public QObject, public rclcpp::Node {
    Q_OBJECT
   public:
    explicit RelocalizationNode(QObject* parent = nullptr);
    ~RelocalizationNode() = default;
    bool registration_success_ = false;

   public slots:
    void onSetAlignSignal(align_res alignResult);
    void OnRetrySet(bool isOk);

   signals:
    void alignResultSignal(reloc_res res);
    void setResultSignal(bool isok);

   private:
    std::atomic<bool> qt_control{false};
    void GlobalLocalizationTimerCallback();

    void SaveCurScanCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishTransform();
    bool InitGlobalMap(std::string map_path);

    // 2D BEV matching components
    bool loadContourMapFromPgm();
    void buildMatchGrid();
    Eigen::Matrix4d CorrelativeMatch();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_2d_;

    std::mutex mtx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_ = nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::string pcd_path_;

    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ = nullptr;
    bool has_global_map_ = false;
    bool has_scan_ = false;

    // Small matching config
    int max_scan_count_ = 10;
    int scan_count = 0;

    // Pose limits
    double map_z_min_ = 0.1;
    double map_z_max_ = 0.5;
    double scan_z_min_ = -0.2;
    double scan_z_max_ = 0.2;
    double resolution_ = 0.01;
    bool use_precomputed_contour_map_ = true;
    std::string contour_map_pgm_path_;
    double contour_map_origin_x_ = 0.0;
    double contour_map_origin_y_ = 0.0;
    int match_tolerance_cells_ = 1;
    double search_x_bounds_ = 0.08;
    double search_y_bounds_ = 0.08;
    double search_yaw_bounds_deg_ = 10.0;
    double trans_step_ = 0.01;
    double yaw_step_deg_ = 0.5;

    // Grid data
    std::vector<uint8_t> grid_map_data_;
    std::vector<uint8_t> match_grid_data_;
    double map_x_min_ = 0.0;
    double map_x_max_ = 0.0;
    double map_y_min_ = 0.0;
    double map_y_max_ = 0.0;
    int map_width_ = 0;
    int map_height_ = 0;
    sensor_msgs::msg::PointCloud2 map_cloud_msg_;
    std::vector<Eigen::Vector2d> scan_2d_points_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    Eigen::Isometry3d result_t_;
    Eigen::Matrix4d pose_estimation_;
    std::vector<double> init_pose_;

    // Helpers
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
        Eigen::Vector3d eul = R.eulerAngles(2, 1, 0);
        return eul[1];
    }
    static inline double RollFromMat(const Eigen::Matrix4d& T) {
        const auto& R = T.block<3, 3>(0, 0);
        Eigen::Vector3d eul = R.eulerAngles(2, 1, 0);
        return eul[2];
    }
};
