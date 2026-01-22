#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <so3_math.h>
#include <tf2_ros/transform_broadcaster.h>

#include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <sensor_msgs/msg/imu.hpp>

using namespace std;
using namespace Eigen;

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2;
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_input, ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                                    (vect3, offset_T_L_I))((vect3, vel))((
                                    vect3, bg))((vect3, ba))((vect3, gravity)));

MTK_BUILD_MANIFOLD(state_output,
                   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))(
                       (vect3, offset_T_L_I))((vect3, vel))((vect3, omg))(
                       (vect3, acc))((vect3, gravity))((vect3, bg))((vect3,
                                                                     ba)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

MTK_BUILD_MANIFOLD(process_noise_input,
                   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

MTK_BUILD_MANIFOLD(
    process_noise_output,
    ((vect3, vel))((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

extern esekfom::esekf<state_input, 24, input_ikfom> kf_input;
extern esekfom::esekf<state_output, 30, input_ikfom> kf_output;

#define PBWIDTH 30
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"

#define PI_M (3.14159265358)
// #define G_m_s2 (9.81)         // Gravaty const in GuangDong/China
#define DIM_STATE (24)   // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)  // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (0.0001)
#define NUM_MATCH_POINTS (5)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define VEC_FROM_ARRAY_SIX(v) v[0], v[1], v[2], v[3], v[4], v[5]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)               \
    vector<decltype(mat)::Scalar>(mat.data(), \
                                  mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

const M3D Eye3d(M3D::Identity());
const M3F Eye3f(M3F::Identity());
const V3D Zero3d(0, 0, 0);
const V3F Zero3f(0, 0, 0);

// ======================= 数据打包结构体 =======================
// MeasureGroup: 用于存储一次 EKF 更新所需的所有传感器数据
// 包含: 一帧 LiDAR 点云 + 该时段内的所有 IMU 数据 + 该时段内的所有轮速计数据
// 这是实现紧耦合多源融合的基础数据结构
struct MeasureGroup  // Lidar data and imu dates for the curent process
{
    MeasureGroup() {
        lidar_beg_time = 0.0;
        lidar_last_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;      // LiDAR 帧的起始时间 (秒)
    double lidar_last_time;     // LiDAR 帧的结束时间 (秒)
    PointCloudXYZI::Ptr lidar;  // LiDAR 点云数据
    deque<sensor_msgs::msg::Imu::ConstSharedPtr>
        imu;  // 该时段内的 IMU 数据队列

    // ======================= 轮速计数据 (Wheel Odometry)
    // ======================= 存储与当前 LiDAR 帧时间对齐的轮速计测量值
    // 时间范围: [lidar_beg_time, lidar_last_time]
    // 数据格式: nav_msgs::msg::Odometry，主要使用其中的 twist.twist.linear
    // (线速度) 用途: 在 Estimator 的 h_model_output
    // 中构建速度观测残差，紧耦合优化 注意: 如果时间窗口很短（如 cut_frame
    // 模式下的 10ms），该队列可能为空，这是允许的
    deque<nav_msgs::msg::Odometry> wheel_odom;
};

template <typename T>
T calc_dist(PointType p1, PointType p2) {
    T d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
          (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template <typename T>
T calc_dist(Eigen::Vector3d p1, PointType p2) {
    T d = (p1(0) - p2.x) * (p1(0) - p2.x) + (p1(1) - p2.y) * (p1(1) - p2.y) +
          (p1(2) - p2.z) * (p1(2) - p2.z);
    return d;
}

// ======================= Batch-Wise 时间切分函数 =======================
// 功能：将点云按固定时间窗口（默认 10ms）切分为多个 Batch
//
// 背景：
//   原 Point-LIO 的切分逻辑是"时间戳变化即切分"，对于 Mid-360 等固态雷达，
//   由于点云时间戳极其密集且连续，会导致 Batch 极其细碎（趋近于逐点更新），
//   造成 EKF Update 频率过高（可达数千 Hz），CPU 负载极大。
//
// 优化思路（Batch-LIWO 论文核心）：
//   强制按固定时间窗口（如 10ms = 100Hz）切分，将窗口内所有点作为一个 Batch
//   统一进行 EKF Update。这样：
//   1. 减少 Update 次数（从几千次降到 ~10 次/帧）
//   2. 每个 Batch 包含多方向平面信息，观测矩阵秩更满，提高稳定性
//   3. 输出频率稳定在 100Hz，便于下游控制算法使用
//
// 参数：
//   - point_cloud: 输入点云（已按时间排序），curvature
//   字段存储相对帧起始的时间(ms)
//
// 返回：
//   - time_seq: 每个 Batch 包含的点数列表，例如 [150, 180, 170, ...] 表示
//               第一个 Batch 有 150 个点，第二个有 180 个点，以此类推
//
template <typename T>
std::vector<int> time_compressing(const PointCloudXYZI::Ptr& point_cloud) {
    int points_size = point_cloud->points.size();
    std::vector<int> time_seq;

    // 空点云保护
    if (points_size == 0) {
        return time_seq;
    }

    // ======================= 核心参数 =======================
    // Batch 时间窗口大小 (单位: 毫秒)
    // 10ms 对应 100Hz 的更新频率
    // 可根据需求调整：5ms -> 200Hz, 20ms -> 50Hz
    constexpr double batch_size_ms = 1.0;

    // 预估 Batch 数量，减少 vector 重分配
    // 假设一帧 100ms，则约有 10 个 Batch
    time_seq.reserve(points_size / 100 + 10);

    int count = 0;  // 当前 Batch 内的点计数
    // 当前 Batch 的起始时间 (curvature 单位是 ms)
    double batch_start_time = point_cloud->points[0].curvature;

    for (int i = 0; i < points_size; i++) {
        count++;
        double current_time = point_cloud->points[i].curvature;

        // 切分条件：
        // 1. 当前点与 Batch 起始点的时间差超过阈值 (10ms)
        // 2. 或者已经是最后一个点
        bool time_exceeded = (current_time - batch_start_time >= batch_size_ms);
        bool is_last_point = (i == points_size - 1);

        if (time_exceeded || is_last_point) {
            // 保存当前 Batch 的点数
            time_seq.emplace_back(count);

            // 重置计数器，准备下一个 Batch
            count = 0;

            // 更新下一个 Batch 的起始时间
            // 注意：如果不是最后一个点，下一个点将作为新 Batch 的第一个点
            if (i < points_size - 1) {
                batch_start_time = point_cloud->points[i + 1].curvature;
            }
        }
    }

    return time_seq;
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template <typename T>
bool esti_normvector(Matrix<T, 3, 1>& normvec, const PointVector& point,
                     const T& threshold, const int& point_num) {
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++) {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++) {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y +
                 normvec(2) * point[j].z + 1.0f) > threshold) {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

template <typename T>
bool esti_plane(Matrix<T, 4, 1>& pca_result, const PointVector& point,
                const T& threshold) {
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y +
                 pca_result(2) * point[j].z + pca_result(3)) > threshold) {
            return false;
        }
    }
    return true;
}

inline double get_time_sec(const builtin_interfaces::msg::Time& time) {
    return rclcpp::Time(time).seconds();
}

inline rclcpp::Time get_ros_time(double timestamp) {
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}

#endif