#include "relocalization_node.hpp"

#include <pcl/filters/passthrough.h>
// Limit namespace leakage to this translation unit only
using namespace small_gicp;

RelocalizationNode::RelocalizationNode()
    : Node("relocalization_node"),
      global_map_(new pcl::PointCloud<pcl::PointXYZ>),
      cur_scan_(new pcl::PointCloud<pcl::PointXYZ>),
      accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    RCLCPP_INFO(this->get_logger(), "RelocalizationNode init");
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_z", 0.0);
    this->declare_parameter("init_yaw", 0.0);
    this->declare_parameter("init_pitch", 0.0);
    this->declare_parameter("init_roll", 0.0);

    this->declare_parameter(
        "pcd_path",
        "/home/fyk/Projects/ros2_ws/src/relocalization/PCD/voxel2025RMUC.pcd");

    // New registration gating and policy parameters
    this->declare_parameter("gate_yaw_deg", gate_yaw_deg_);
    this->declare_parameter("gate_roll_deg", gate_roll_deg_);
    this->declare_parameter("gate_pitch_deg", gate_pitch_deg_);
    this->declare_parameter("gate_trans_m", gate_trans_m_);

    this->declare_parameter("enforce_rp_zero", enforce_rp_zero_);
    this->declare_parameter("yaw_search_step_deg", yaw_search_step_deg_);
    this->declare_parameter("max_fail_before_fallback",
                            max_fail_before_fallback_);
    this->declare_parameter("fallback_policy", fallback_policy_);
    //------------
    this->declare_parameter("fitness_threshold", fitness_threshold_);
    this->declare_parameter("num_neighbors", 20);
    this->declare_parameter("smgicp_global_leaf_size", 0.01);
    this->declare_parameter("smgicp_local_leaf_size", 0.05);
    this->declare_parameter("smgicp_max_iterations", 50);
    this->declare_parameter("smgicp_num_threads", 5);
    this->declare_parameter("smgicp_max_dist_sq", 3.0);
    //初始位姿设定
    init_x_ = this->get_parameter("init_x").as_double();
    init_y_ = this->get_parameter("init_y").as_double();
    init_z_ = this->get_parameter("init_z").as_double();
    init_yaw_ = this->get_parameter("init_yaw").as_double();
    init_pitch_ = this->get_parameter("init_pitch").as_double();
    init_roll_ = this->get_parameter("init_roll").as_double();
    // this->get_parameter("init_pose", init_poses);
    //先验地图路径
    pcd_path_ = this->get_parameter("pcd_path").as_string();

    // Load runtime-configurable params into member variables
    gate_yaw_deg_ = this->get_parameter("gate_yaw_deg").as_double();
    gate_trans_m_ = this->get_parameter("gate_trans_m").as_double();
    enforce_rp_zero_ = this->get_parameter("enforce_rp_zero").as_bool();
    yaw_search_step_deg_ =
        this->get_parameter("yaw_search_step_deg").as_double();
    max_fail_before_fallback_ =
        this->get_parameter("max_fail_before_fallback").as_int();
    fallback_policy_ = this->get_parameter("fallback_policy").as_string();
    fitness_threshold_ = this->get_parameter("fitness_threshold").as_double();
    gate_roll_deg_ = this->get_parameter("gate_roll_deg").as_double();
    gate_pitch_deg_ = this->get_parameter("gate_pitch_deg").as_double();
    // small-gicp配置
    smgicp_max_dist_sq_ = this->get_parameter("smgicp_max_dist_sq").as_double();
    smgicp_max_iterations_ =
        this->get_parameter("smgicp_max_iterations").as_int();
    smgicp_num_threads_ = this->get_parameter("smgicp_num_threads").as_int();
    smgicp_global_leaf_size_ =
        this->get_parameter("smgicp_global_leaf_size").as_double();
    smgicp_local_leaf_size_ =
        this->get_parameter("smgicp_local_leaf_size").as_double();
    nebhor_num = this->get_parameter("num_neighbors").as_int();
    //-----------------
    pose_estimation_ = CreateTfMatrix(init_x_, init_y_, init_z_, init_yaw_,
                                      init_pitch_, init_roll_);
    std::cout << "pose_estimation:\n" << pose_estimation_ << std::endl;
    std::cout << "pcd_path:" << pcd_path_ << std::endl;
    InitGlobalMap(pcd_path_);  //加载地图
    //先验地图处理
    target_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *global_map_, smgicp_global_leaf_size_);
    // Estimate covariances of points

    small_gicp::estimate_covariances_omp(*target_, nebhor_num,
                                         smgicp_num_threads_);
    // Create KdTree for target
    target_tree_ = std::make_shared<
        small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(smgicp_num_threads_));
    //匹配器初始化
    register_ = std::make_shared<small_gicp::Registration<
        small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
    register_->reduction.num_threads = smgicp_num_threads_;
    register_->rejector.max_dist_sq = smgicp_max_dist_sq_;
    register_->optimizer.max_iterations = smgicp_max_iterations_;

    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1,
        std::bind(&RelocalizationNode::SaveCurScanCallback, this,
                  std::placeholders::_1));

    sub_init_pose_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&RelocalizationNode::InitPoseCallback, this,
                  std::placeholders::_1));

    pub_map_to_odom_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
    pub_cur_scan_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan", 1);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RelocalizationNode::GlobalLocalizationTimerCallback, this));

    std::cout << "new" << std::endl;
}
void RelocalizationNode::SaveCurScanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //累计10帧
    if (this->scan_count >= 10) return;
    std::lock_guard<std::mutex> lock(mtx);
    pcl::fromROSMsg(*msg, *cur_scan_);
    PassThroughFilter(cur_scan_, cur_scan_, 0.0, 3.5, -9.2, 5.3, -3.2, 24.7);
    *accumulated_cloud += *cur_scan_;
    this->scan_count++;
    //至少积累10帧才进行匹配
    if (this->scan_count == 10) has_scan_ = true;
}
void RelocalizationNode::PassThroughFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, double min_z, double max_z,
    double min_y, double max_y, double min_x, double max_x) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_out);

    pass.setInputCloud(cloud_out);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_out);

    pass.setInputCloud(cloud_out);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*cloud_out);
}
void RelocalizationNode::InitPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    geometry_msgs::msg::Pose pose = msg->pose.pose;
    pose_estimation_ = PoseToMat(pose);
    // pose_estimation_ = PoseToMat();
    std::cout << "pose_estimation:\n" << pose_estimation_ << std::endl;

    // 重置配准状态，重新开始配准
    registration_success_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Received new initial pose, restarting registration...");
}

bool RelocalizationNode::GlobalLocalization() {
    if (!has_scan_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "no scan msg");
        return false;
    }
    rclcpp::Time begin_icp_time = this->now();

    // 小范围多 yaw 初始值搜索
    const double yaw_gate = gate_yaw_deg_ * M_PI / 180.0;
    const double yaw_step = std::max(0.5, yaw_search_step_deg_) * M_PI / 180.0;
    std::vector<double> yaw_seeds;
    yaw_seeds.push_back(0.0);
    for (double a = yaw_step; a <= yaw_gate + 1e-6; a += yaw_step) {
        yaw_seeds.push_back(+a);
        yaw_seeds.push_back(-a);
    }

    double best_fitness = std::numeric_limits<double>::infinity();
    Eigen::Matrix4d best_T = pose_estimation_;
    for (double dyaw : yaw_seeds) {
        Eigen::Matrix4d seed = RotZ(dyaw) * pose_estimation_;
        double f = 1e9;
        Eigen::Isometry3d res =
            SmallIcpRegistration(accumulated_cloud, pose_estimation_, f);
        if (f < best_fitness) {
            best_fitness = f;
            best_T = res.matrix();
        }
    }
    //此时得到了最好的变幻关系
    rclcpp::Time end_icp_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Registration time: %f, best fitness: %f",
                (end_icp_time - begin_icp_time).seconds(), best_fitness);
    //评价可行性
    const bool fitness_ok = best_fitness < fitness_threshold_;
    const bool trans_ok = TransDelta(best_T, pose_estimation_) <= gate_trans_m_;
    const bool yaw_ok = YawDelta(best_T, pose_estimation_) <= yaw_gate;
    const double roll_gate = gate_roll_deg_ * M_PI / 180.0;
    const double pitch_gate = gate_pitch_deg_ * M_PI / 180.0;
    const bool roll_ok = RollDelta(best_T, pose_estimation_) <= roll_gate;
    const bool pitch_ok = PitchDelta(best_T, pose_estimation_) <= pitch_gate;

    if (fitness_ok && trans_ok && yaw_ok && pitch_ok && roll_ok) {
        RCLCPP_INFO(this->get_logger(),
                    "registration can be trusted (fitness=%.5f, dtrans=%.3f, "
                    "droll=%.2fdeg, dpitch=%.2fdeg, dyaw=%.2fdeg)",
                    best_fitness, TransDelta(best_T, pose_estimation_),
                    RollDelta(best_T, pose_estimation_) * 180.0 / M_PI,
                    PitchDelta(best_T, pose_estimation_) * 180.0 / M_PI,
                    YawDelta(best_T, pose_estimation_) * 180.0 / M_PI);
        nav_msgs::msg::Odometry map_to_odom;
        Eigen::Matrix4d T_pub = best_T;
        if (enforce_rp_zero_) {
            ZeroRollPitchKeepYaw(T_pub);
        }
        map_to_odom.pose.pose = MatToPose(T_pub);
        map_to_odom.header.stamp = this->now();
        map_to_odom.header.frame_id = "map";

        pub_map_to_odom_->publish(map_to_odom);
        pose_estimation_ = T_pub;
        registration_success_ = true;  // 设置配准成功标志
        registration_fail_count_ = 0;  // 成功后清零
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "registration rejected (fitness=%.5f, dtrans=%.3f, "
                    "droll=%.2fdeg, dpitch=%.2fdeg, dyaw=%.2fdeg)",
                    best_fitness, TransDelta(best_T, pose_estimation_),
                    RollDelta(best_T, pose_estimation_) * 180.0 / M_PI,
                    PitchDelta(best_T, pose_estimation_) * 180.0 / M_PI,
                    YawDelta(best_T, pose_estimation_) * 180.0 / M_PI);
        registration_fail_count_++;

        if (registration_fail_count_ >= max_fail_before_fallback_) {
            RCLCPP_WARN(this->get_logger(),
                        "Registration failed over %d times, publishing "
                        "fallback (%s)!",
                        max_fail_before_fallback_, fallback_policy_.c_str());
            nav_msgs::msg::Odometry map_to_odom;
            if (fallback_policy_ == "identity") {
                map_to_odom.pose.pose = MatToPose(Eigen::Matrix4d::Identity());
            } else {
                map_to_odom.pose.pose = MatToPose(
                    CreateTfMatrix(init_x_, init_y_, init_z_, init_yaw_,
                                   init_pitch_, init_roll_));
            }
            map_to_odom.header.stamp = this->now();
            map_to_odom.header.frame_id = "map";
            pub_map_to_odom_->publish(map_to_odom);
            registration_success_ = true;
            registration_fail_count_ = 0;  // 重置计数
        }
        return false;
    }
}

void RelocalizationNode::GlobalLocalizationTimerCallback() {
    if (!registration_success_) {  // 如果配准还未成功，继续尝试
        GlobalLocalization();
    }
}

// std::vector<Eigen::Vector3d>
// RelocalizationNode::ConvertPointCloudToEigenVector(
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//     std::vector<Eigen::Vector3d> eigen_vectors;
//     eigen_vectors.reserve(cloud->size());  // 预留空间以提高性能

//     for (const auto& point : *cloud) {
//         eigen_vectors.emplace_back(point.x, point.y, point.z);
//     }

//     return eigen_vectors;
// }

Eigen::Isometry3d RelocalizationNode::SmallIcpRegistration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_scan,
    const Eigen::Matrix4d& initial, double& fitness) {
    // 扫描点云处理
    this->source_ = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *pc_scan, this->smgicp_local_leaf_size_);
    small_gicp::estimate_covariances_omp(*source_, nebhor_num,
                                         this->smgicp_num_threads_);
    this->source_tree_ = std::make_shared<
        small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        this->source_, small_gicp::KdTreeBuilderOMP(this->smgicp_num_threads_));
    //初始位姿估计

    Eigen::Isometry3d init_T_target_source;
    init_T_target_source.matrix() = initial;
    if (!source_ || !source_tree_) {
        return init_T_target_source;
    }
    //匹配
    auto result = register_->align(*target_, *source_, *target_tree_,
                                   init_T_target_source);
    size_t num_inliers = result.num_inliers;  // Number of inlier source points
    fitness = result.error / num_inliers;     // Fitness score
    if (result.converged) {
        Eigen::Isometry3d T = result.T_target_source;
        std::cout << "Gicp registration converged" << std::endl;
        std::cout << "Transformation matrix:\n"
                  << T.matrix() << std::endl;  // Estimated transformation
        std::cout << "fitness" << fitness << std::endl;
        std::cout << "Number of inlier source points:" << num_inliers
                  << std::endl;

        // Eigen::Matrix<double, 6, 6> H = result.H;  // Final Hessian matrix
        // (6x6)
        return T;
    } else {
        std::cout << "Gicp registration not converged" << std::endl;
        return init_T_target_source;
    }
}

// //
// 配准的输入源是当前的scan，camera_init系下的，配准对象为，参考系为map系，但点云是FOV视角中的
// Eigen::Matrix4d RelocalizationNode::IcpRegistration(
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_scan,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_map,
//     const Eigen::Matrix4d& initial, double scale, double& fitness) {
//     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//     // icp.setInputSource(VoxelDownSamping(pc_scan, 0.1 * scale));
//     // icp.setInputTarget(VoxelDownSamping(pc_map, 0.4 * scale));
//     icp.setInputSource(pc_scan);
//     icp.setInputTarget(pc_map);
//     pcl::PointCloud<pcl::PointXYZ> Final;
//     icp.setMaximumIterations(50);          // 最大迭代次数
//     icp.setTransformationEpsilon(1e-8);    // 变换矩阵的容差
//     icp.setEuclideanFitnessEpsilon(1e-6);  // 欧几里得适应度容差
//     icp.align(Final, initial.cast<float>());
//     // icp.align(Final);

//     if (icp.hasConverged()) {
//         std::cout << "ICP has converged!" << std::endl;
//         std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
//         std::cout << "Transformation matrix:\n"
//                   << icp.getFinalTransformation() << std::endl;
//         fitness = icp.getFitnessScore();
//         return icp.getFinalTransformation().cast<double>();
//     } else {
//         std::cout << "ICP did not converge." << std::endl;
//         return initial;
//     }
// }

bool RelocalizationNode::InitGlobalMap(std::string map_path) {
    // 耗时
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *global_map_) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return false;
    }
    return true;
}

Eigen::Matrix4d RelocalizationNode::PoseToMat(
    const geometry_msgs::msg::Pose& pose) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat(0, 3) = pose.position.x;
    mat(1, 3) = pose.position.y;
    mat(2, 3) = pose.position.z;

    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = m[i][j];
        }
    }
    return mat;
}

geometry_msgs::msg::Pose RelocalizationNode::MatToPose(
    const Eigen::Matrix4d& mat) {
    geometry_msgs::msg::Pose pose;

    // 提取位置信息
    pose.position.x = mat(0, 3);
    pose.position.y = mat(1, 3);
    pose.position.z = mat(2, 3);

    // 提取旋转矩阵
    tf2::Matrix3x3 m(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1),
                     mat(1, 2), mat(2, 0), mat(2, 1), mat(2, 2));

    // 将旋转矩阵转换为四元数
    tf2::Quaternion q;
    m.getRotation(q);

    // 设置四元数到pose中
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

double RelocalizationNode::Deg2Rad(double deg) { return deg * M_PI / 180.0; }
// 欧拉角转旋转矩阵
Eigen::Matrix4d RelocalizationNode::CreateTfMatrix(double x, double y, double z,
                                                   double yaw, double pitch,
                                                   double roll) {
    yaw = Deg2Rad(yaw);
    pitch = Deg2Rad(pitch);
    roll = Deg2Rad(roll);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return T;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizationNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
