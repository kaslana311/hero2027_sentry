#include "hero_lidar_scan/hero_lidar_scan_node.hpp"

#include <omp.h>

#include <algorithm>

namespace hero_lidar_scan {

HeroLidarScanNode::HeroLidarScanNode(const rclcpp::NodeOptions& options)
    : Node("hero_lidar_scan_node", options), max_odom_buffer_size_(1000) {
    // 声明参数
    this->declare_parameter<std::string>("lidar1_topic", "/livox/lidar1");
    this->declare_parameter<std::string>("lidar2_topic", "/livox/lidar2");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("fused_cloud_topic", "/fused_cloud");

    // 雷达1外参
    this->declare_parameter<double>("lidar1_x", 0.0);
    this->declare_parameter<double>("lidar1_y", 0.0);
    this->declare_parameter<double>("lidar1_z", 0.0);
    this->declare_parameter<double>("lidar1_roll", 0.0);
    this->declare_parameter<double>("lidar1_pitch", 0.0);
    this->declare_parameter<double>("lidar1_yaw", 0.0);

    // 雷达2外参
    this->declare_parameter<double>("lidar2_x", 0.0);
    this->declare_parameter<double>("lidar2_y", 0.0);
    this->declare_parameter<double>("lidar2_z", 0.0);
    this->declare_parameter<double>("lidar2_roll", 0.0);
    this->declare_parameter<double>("lidar2_pitch", 0.0);
    this->declare_parameter<double>("lidar2_yaw", 0.0);

    // 读取参数
    auto lidar1_topic = this->get_parameter("lidar1_topic").as_string();
    auto lidar2_topic = this->get_parameter("lidar2_topic").as_string();
    auto odom_topic = this->get_parameter("odom_topic").as_string();
    auto fused_cloud_topic =
        this->get_parameter("fused_cloud_topic").as_string();

    // 构建雷达1外参矩阵
    double l1_x = this->get_parameter("lidar1_x").as_double();
    double l1_y = this->get_parameter("lidar1_y").as_double();
    double l1_z = this->get_parameter("lidar1_z").as_double();
    double l1_roll = this->get_parameter("lidar1_roll").as_double();
    double l1_pitch = this->get_parameter("lidar1_pitch").as_double();
    double l1_yaw = this->get_parameter("lidar1_yaw").as_double();

    Eigen::AngleAxisd rollAngle1(l1_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle1(l1_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle1(l1_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q1 = yawAngle1 * pitchAngle1 * rollAngle1;
    T_lidar1_to_base_ = Eigen::Matrix4d::Identity();
    T_lidar1_to_base_.block<3, 3>(0, 0) = q1.toRotationMatrix();
    T_lidar1_to_base_(0, 3) = l1_x;
    T_lidar1_to_base_(1, 3) = l1_y;
    T_lidar1_to_base_(2, 3) = l1_z;

    // 构建雷达2外参矩阵
    double l2_x = this->get_parameter("lidar2_x").as_double();
    double l2_y = this->get_parameter("lidar2_y").as_double();
    double l2_z = this->get_parameter("lidar2_z").as_double();
    double l2_roll = this->get_parameter("lidar2_roll").as_double();
    double l2_pitch = this->get_parameter("lidar2_pitch").as_double();
    double l2_yaw = this->get_parameter("lidar2_yaw").as_double();

    Eigen::AngleAxisd rollAngle2(l2_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle2(l2_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle2(l2_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q2 = yawAngle2 * pitchAngle2 * rollAngle2;
    T_lidar2_to_base_ = Eigen::Matrix4d::Identity();
    T_lidar2_to_base_.block<3, 3>(0, 0) = q2.toRotationMatrix();
    T_lidar2_to_base_(0, 3) = l2_x;
    T_lidar2_to_base_(1, 3) = l2_y;
    T_lidar2_to_base_(2, 3) = l2_z;

    // 创建订阅器和发布器
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 500,
        std::bind(&HeroLidarScanNode::odomCallback, this,
                  std::placeholders::_1));

    lidar1_sub_.subscribe(this, lidar1_topic);
    lidar2_sub_.subscribe(this, lidar2_topic);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), lidar1_sub_, lidar2_sub_);
    sync_->registerCallback(std::bind(&HeroLidarScanNode::lidarSyncCallback,
                                      this, std::placeholders::_1,
                                      std::placeholders::_2));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        fused_cloud_topic, 10);

    RCLCPP_INFO(this->get_logger(), "HeroLidarScan节点已启动");
}

// 里程计回调：维护时间有序队列
void HeroLidarScanNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push_back(*msg);
    if (odom_buffer_.size() > max_odom_buffer_size_) {
        odom_buffer_.pop_front();
    }
}

// 位姿插值查询：基于传入的只读本地里程计队列。线性插值平移，SLERP插值旋转
bool HeroLidarScanNode::getPoseAtTimeLocal(
    const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
    const double target_time, Eigen::Matrix4d& pose) {
    if (local_odom_buffer.size() < 2) return false;

    // 查找前后两帧
    size_t idx_after = 0;
    for (size_t i = 0; i < local_odom_buffer.size(); ++i) {
        if (rclcpp::Time(local_odom_buffer[i].header.stamp).seconds() >=
            target_time) {
            idx_after = i;
            break;
        }
    }

    if (idx_after == 0) {
        // 目标时间早于队列最早时间，使用最早帧
        auto& odom = local_odom_buffer[0];
        Eigen::Quaterniond q(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        pose(0, 3) = odom.pose.pose.position.x;
        pose(1, 3) = odom.pose.pose.position.y;
        pose(2, 3) = odom.pose.pose.position.z;
        return true;
    }

    size_t idx_before = idx_after - 1;
    auto& odom_before = local_odom_buffer[idx_before];
    auto& odom_after = local_odom_buffer[idx_after];

    double t_before = rclcpp::Time(odom_before.header.stamp).seconds();
    double t_after = rclcpp::Time(odom_after.header.stamp).seconds();
    double ratio = (target_time - t_before) / (t_after - t_before + 1e-9);
    ratio = std::clamp(ratio, 0.0, 1.0);

    // 平移线性插值
    Eigen::Vector3d pos_before(odom_before.pose.pose.position.x,
                               odom_before.pose.pose.position.y,
                               odom_before.pose.pose.position.z);
    Eigen::Vector3d pos_after(odom_after.pose.pose.position.x,
                              odom_after.pose.pose.position.y,
                              odom_after.pose.pose.position.z);
    Eigen::Vector3d pos_interp = pos_before + ratio * (pos_after - pos_before);

    // 旋转SLERP插值
    Eigen::Quaterniond q_before(odom_before.pose.pose.orientation.w,
                                odom_before.pose.pose.orientation.x,
                                odom_before.pose.pose.orientation.y,
                                odom_before.pose.pose.orientation.z);
    Eigen::Quaterniond q_after(
        odom_after.pose.pose.orientation.w, odom_after.pose.pose.orientation.x,
        odom_after.pose.pose.orientation.y, odom_after.pose.pose.orientation.z);
    Eigen::Quaterniond q_interp = q_before.slerp(ratio, q_after);

    pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
    pose.block<3, 1>(0, 3) = pos_interp;
    return true;
}

// 处理单个雷达点云去畸变：引入降频桶优化与无锁本地化查询
void HeroLidarScanNode::processLidar(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
    const Eigen::Matrix4d& T_ext, const Eigen::Matrix4d& pose_end,
    const std::deque<nav_msgs::msg::Odometry>& local_odom_buffer,
    pcl::PointCloud<pcl::PointXYZI>& cloud_out) {
    double timebase = msg->timebase * 1e-9;  // 纳秒转秒
    Eigen::Matrix4d pose_end_inv = pose_end.inverse();

    size_t point_num = msg->point_num;
    cloud_out.points.resize(point_num);

    // 动态桶策略 (分段预积分)：假设每 5 毫秒 (0.005 秒)
    // 视作一个固定位姿桶，大幅减少插值次数
    const double BUCKET_TIME_RESOLUTION = 0.005;

    // 计算每个桶的位姿并缓存
    double total_duration = 0.0;
    if (point_num > 0) {
        total_duration = msg->points[point_num - 1].offset_time * 1e-9;
    }

    size_t num_buckets =
        static_cast<size_t>(total_duration / BUCKET_TIME_RESOLUTION) + 1;
    std::vector<Eigen::Matrix4d> bucket_transforms(num_buckets);
    std::vector<bool> bucket_valid(num_buckets, false);

// 这里的桶大小大概也就 10-20 个，顺带并行处理
#pragma omp parallel for
    for (size_t b = 0; b < num_buckets; ++b) {
        double bucket_time = timebase + b * BUCKET_TIME_RESOLUTION;
        Eigen::Matrix4d pose_i;
        if (getPoseAtTimeLocal(local_odom_buffer, bucket_time, pose_i)) {
            // 预计算出当前桶的转换矩阵: pose_end^-1 * pose_i * T_ext
            bucket_transforms[b] = pose_end_inv * pose_i * T_ext;
            bucket_valid[b] = true;
        }
    }

// 遍历点云并应用桶矩阵变换
#pragma omp parallel for
    for (size_t i = 0; i < point_num; ++i) {
        const auto& pt = msg->points[i];
        double pt_offset_time = pt.offset_time * 1e-9;

        // 找出该点对应的桶索引
        size_t bucket_idx =
            static_cast<size_t>(pt_offset_time / BUCKET_TIME_RESOLUTION);
        if (bucket_idx >= num_buckets) bucket_idx = num_buckets - 1;

        // 如果该桶未能拿到位姿（时间越界等），原样保留点，仅作雷达外参变换兜底
        Eigen::Matrix4d transform;
        if (bucket_valid[bucket_idx]) {
            transform = bucket_transforms[bucket_idx];
        } else {
            transform = T_ext;
        }

        Eigen::Vector4d p_raw(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d p_undist = transform * p_raw;

        cloud_out.points[i].x = p_undist(0);
        cloud_out.points[i].y = p_undist(1);
        cloud_out.points[i].z = p_undist(2);
        cloud_out.points[i].intensity = pt.reflectivity;
    }
}

// 双雷达同步回调
void HeroLidarScanNode::lidarSyncCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar1_msg,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar2_msg) {
    // 1. 计算时间范围
    double t1_start = lidar1_msg->timebase * 1e-9;
    double t2_start = lidar2_msg->timebase * 1e-9;
    double t_start = std::min(t1_start, t2_start);

    double t1_end = t1_start;
    double t2_end = t2_start;
    if (!lidar1_msg->points.empty()) {
        t1_end += lidar1_msg->points.back().offset_time * 1e-9;
    }
    if (!lidar2_msg->points.empty()) {
        t2_end += lidar2_msg->points.back().offset_time * 1e-9;
    }
    double t_end = std::max(t1_end, t2_end);
    rclcpp::Time stamp_end(static_cast<int64_t>(t_end * 1e9));

    // 2. 将此时间段的里程计拷贝到本地无锁队列，立刻放锁
    std::deque<nav_msgs::msg::Odometry> local_odom_buffer;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "里程计队列为空");
            return;
        }

        // 为了插值，我们需要至少拿出一个早于 start_time，和一个晚于 end_time
        // 的缓冲数据
        local_odom_buffer = odom_buffer_;
    }

    // 3. 利用本地无锁队列查询基准时刻位姿
    Eigen::Matrix4d pose_end;
    if (!getPoseAtTimeLocal(local_odom_buffer, t_end, pose_end)) {
        RCLCPP_WARN(this->get_logger(), "无法获取基准时刻位姿");
        return;
    }

    // 4. 处理两个雷达点云，传入本地缓存队列
    pcl::PointCloud<pcl::PointXYZI> cloud1, cloud2;
    processLidar(lidar1_msg, T_lidar1_to_base_, pose_end, local_odom_buffer,
                 cloud1);
    processLidar(lidar2_msg, T_lidar2_to_base_, pose_end, local_odom_buffer,
                 cloud2);

    // 融合点云
    pcl::PointCloud<pcl::PointXYZI> fused_cloud = cloud1 + cloud2;

    // 转换并发布
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(fused_cloud, output_msg);
    output_msg.header.stamp = stamp_end;
    output_msg.header.frame_id = "base_link";
    cloud_pub_->publish(output_msg);
}

}  // namespace hero_lidar_scan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hero_lidar_scan::HeroLidarScanNode)
