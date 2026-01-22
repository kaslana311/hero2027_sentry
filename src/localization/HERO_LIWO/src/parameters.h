// #ifndef PARAM_H
// #define PARAM_H
#pragma once
#include <Python.h>
#include <ivox/ivox3d.h>
#include <math.h>
#include <omp.h>
#include <pcl/common/transforms.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "IMU_Processing.h"
#include "preprocess.h"

// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
using IVoxType =
    faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;
extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern state_input state_in;
extern state_output state_out;
extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
// extern double cube_len;
extern float DET_RANGE;
extern bool imu_en;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern bool runtime_pos_log, pcd_save_en, path_en;
extern bool scan_pub_en, scan_body_pub_en, tf_send_en;
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern bool is_first_frame;

extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern double time_diff_lidar_to_imu;

// ======================= 轮速计 (Wheel Odometry) 相关参数
// ======================= 轮速计中心相对于 IMU 的外参平移向量 (单位: 米)
// 物理意义: 从 IMU 坐标系原点指向轮速计/机器人中心的向量，在 IMU 坐标系下表示
extern std::vector<double> extrinT_wheel;
// 轮速计中心相对于 IMU 的外参旋转矩阵 (3x3, 行优先存储)
// 物理意义: 将轮速计坐标系下的向量变换到 IMU 坐标系下
extern std::vector<double> extrinR_wheel;
// 轮速计速度测量噪声协方差 (用于 EKF 测量更新)
// 物理意义: 表示轮速计测量值的不确定性，值越大表示越不信任轮速计
extern double wheel_vel_cov;
// 轮速计话题名称
extern std::string wheel_topic;
// 是否启用轮速计融合
extern bool wheel_en;
// 轮速计数据的最后时间戳 (用于时间同步和乱序检测)
extern double last_timestamp_wheel;
extern double lidar_time_inte, first_imu_time;
extern int cut_frame_num, orig_odom_freq;
extern double online_refine_time;  // unit: s
extern bool cut_frame_init;
extern double time_update_last, time_current, time_predict_last_const, t_last;

extern bool enable_prior_pcd;
extern string prior_pcd_map_path;
extern std::vector<double> init_pose;

extern MeasureGroup Measures;

extern ofstream fout_out, fout_imu_pbp;
void readParameters(std::shared_ptr<rclcpp::Node>& n);
void open_file();
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3& orient);
void reset_cov(Eigen::Matrix<double, 24, 24>& P_init);
void reset_cov_output(Eigen::Matrix<double, 30, 30>& P_init_output);