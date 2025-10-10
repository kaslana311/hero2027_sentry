#include "relocalization_node.hpp"

// Limit namespace leakage to this translation unit only
using namespace small_gicp;

RelocalizationNode::RelocalizationNode()
    : Node("relocalization_node"),
      global_map_(new pcl::PointCloud<pcl::PointXYZ>),
      cur_scan_(new pcl::PointCloud<pcl::PointXYZ>) {
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
    this->declare_parameter("use_crop_map", use_crop_map_);
    this->declare_parameter("crop_radius", crop_radius_);
    this->declare_parameter("fitness_threshold", fitness_threshold_);

    init_x_ = this->get_parameter("init_x").as_double();
    init_y_ = this->get_parameter("init_y").as_double();
    init_z_ = this->get_parameter("init_z").as_double();
    init_yaw_ = this->get_parameter("init_yaw").as_double();
    init_pitch_ = this->get_parameter("init_pitch").as_double();
    init_roll_ = this->get_parameter("init_roll").as_double();
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
    use_crop_map_ = this->get_parameter("use_crop_map").as_bool();
    crop_radius_ = this->get_parameter("crop_radius").as_double();
    fitness_threshold_ = this->get_parameter("fitness_threshold").as_double();
    gate_roll_deg_ = this->get_parameter("gate_roll_deg").as_double();
    gate_pitch_deg_ = this->get_parameter("gate_pitch_deg").as_double();

    pose_estimation_ = CreateTfMatrix(init_x_, init_y_, init_z_, init_yaw_,
                                      init_pitch_, init_roll_);

    std::cout << "pose_estimation:\n" << pose_estimation_ << std::endl;
    std::cout << "pcd_path:" << pcd_path_ << std::endl;

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 1,
        std::bind(&RelocalizationNode::SaveCurOdomCallback, this,
                  std::placeholders::_1));

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
    pub_crop_map_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/crop_map", 1);
    pub_cur_scan_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan", 1);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&RelocalizationNode::GlobalLocalizationTimerCallback, this));

    std::cout << "new" << std::endl;
}
void RelocalizationNode::SaveCurOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx);
    cur_odom_ = *msg;
    has_odom_ = true;
}

void RelocalizationNode::SaveCurScanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //
    std::lock_guard<std::mutex> lock(mtx);
    pcl::fromROSMsg(*msg, *cur_scan_);
    // msg->header.frame_id = "odom";
    // msg->header.stamp = this->now();
    // pub_cur_scan_->publish(*msg);
    has_scan_ = true;
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
    // 加载全局地图
    // 没有地图，记载地图
    if (!has_global_map_) {
        rclcpp::Time begin_load_map_time = this->now();
        if (InitGlobalMap(pcd_path_)) {
            has_global_map_ = true;
            RCLCPP_INFO(this->get_logger(), "Successfully init global map");
            rclcpp::Time end_load_map_time = this->now();
            RCLCPP_INFO(this->get_logger(), "Load map time: %f",
                        (end_load_map_time - begin_load_map_time).seconds());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to init global map");
            return false;
        }
    }
    if (has_odom_) {
        if (!has_scan_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "no scan msg");
            return false;
        }
        // 返回一个智能指针，指向crop后的点云地图
        rclcpp::Time begin_crop_map_time = this->now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_for_reg = global_map_;
        if (use_crop_map_) {
            crop_global_map_ = CropMapInFov(global_map_, pose_estimation_,
                                            cur_odom_, 6.28, crop_radius_);
            map_for_reg = crop_global_map_;
        } else {
            PublishCropMap(global_map_);
        }

        // 赋值给
        std::lock_guard<std::mutex> lock(mtx);
        // 拷贝构造函数
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_tobe_mapped(
            new pcl::PointCloud<pcl::PointXYZ>(*cur_scan_));

        rclcpp::Time begin_icp_time = this->now();

        // 小范围多 yaw 初始值搜索
        const double yaw_gate = gate_yaw_deg_ * M_PI / 180.0;
        const double yaw_step =
            std::max(0.5, yaw_search_step_deg_) * M_PI / 180.0;
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
                SmallIcpRegistration(scan_tobe_mapped, map_for_reg, seed, f);
            if (f < best_fitness) {
                best_fitness = f;
                best_T = res.matrix();
            }
        }

        rclcpp::Time end_icp_time = this->now();
        RCLCPP_INFO(this->get_logger(),
                    "Registration time: %f, best fitness: %f",
                    (end_icp_time - begin_icp_time).seconds(), best_fitness);

        const bool fitness_ok = best_fitness < fitness_threshold_;
        const bool trans_ok =
            TransDelta(best_T, pose_estimation_) <= gate_trans_m_;
        const bool yaw_ok = YawDelta(best_T, pose_estimation_) <= yaw_gate;
        const double roll_gate = gate_roll_deg_ * M_PI / 180.0;
        const double pitch_gate = gate_pitch_deg_ * M_PI / 180.0;
        const bool roll_ok = RollDelta(best_T, pose_estimation_) <= roll_gate;
        const bool pitch_ok =
            PitchDelta(best_T, pose_estimation_) <= pitch_gate;

        if (fitness_ok && trans_ok && yaw_ok && roll_ok && pitch_ok) {
            RCLCPP_INFO(this->get_logger(), "registration can be trusted");
            RCLCPP_INFO(this->get_logger(), "Fitness:%f", best_fitness);

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
                            max_fail_before_fallback_,
                            fallback_policy_.c_str());
                nav_msgs::msg::Odometry map_to_odom;
                if (fallback_policy_ == "identity") {
                    map_to_odom.pose.pose =
                        MatToPose(Eigen::Matrix4d::Identity());
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
    } else {
        RCLCPP_WARN(this->get_logger(), "no odom msg");
        return false;
    }
}

void RelocalizationNode::GlobalLocalizationTimerCallback() {
    if (!registration_success_) {  // 如果配准还未成功，继续尝试
        GlobalLocalization();
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RelocalizationNode::VoxelDownSamping(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double leaf_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}

std::vector<Eigen::Vector3d> RelocalizationNode::ConvertPointCloudToEigenVector(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::vector<Eigen::Vector3d> eigen_vectors;
    eigen_vectors.reserve(cloud->size());  // 预留空间以提高性能

    for (const auto &point : *cloud) {
        eigen_vectors.emplace_back(point.x, point.y, point.z);
    }

    return eigen_vectors;
}

Eigen::Isometry3d RelocalizationNode::SmallIcpRegistration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_scan,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_map,
    const Eigen::Matrix4d &initial, double &fitness) {
    std::vector<Eigen::Vector3d> source_points =
        ConvertPointCloudToEigenVector(pc_scan);
    std::vector<Eigen::Vector3d> target_points =
        ConvertPointCloudToEigenVector(pc_map);
    int num_threads = 4;                    // Number of threads to be used
    double downsampling_resolution = 0.25;  // Downsampling resolution
    int num_neighbors = 10;
    auto [source, source_tree] = preprocess_points(
        source_points, downsampling_resolution, num_neighbors, num_threads);
    auto [target, target_tree] = preprocess_points(
        target_points, downsampling_resolution, num_neighbors, num_threads);

    RegistrationSetting setting;
    setting.num_threads = num_threads;
    setting.max_correspondence_distance = 1.0;
    Eigen::Isometry3d init_T_target_source;
    init_T_target_source.matrix() = initial;

    RegistrationResult result =
        align(*target, *source, *target_tree, init_T_target_source, setting);
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

        Eigen::Matrix<double, 6, 6> H = result.H;  // Final Hessian matrix (6x6)
        return T;
    } else {
        std::cout << "Gicp registration not converged" << std::endl;
        return init_T_target_source;
    }
}

// 配准的输入源是当前的scan，camera_init系下的，配准对象为，参考系为map系，但点云是FOV视角中的
Eigen::Matrix4d RelocalizationNode::IcpRegistration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_scan,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_map,
    const Eigen::Matrix4d &initial, double scale, double &fitness) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(VoxelDownSamping(pc_scan, 0.1 * scale));
    // icp.setInputTarget(VoxelDownSamping(pc_map, 0.4 * scale));
    icp.setInputSource(pc_scan);
    icp.setInputTarget(pc_map);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.setMaximumIterations(50);          // 最大迭代次数
    icp.setTransformationEpsilon(1e-8);    // 变换矩阵的容差
    icp.setEuclideanFitnessEpsilon(1e-6);  // 欧几里得适应度容差
    icp.align(Final, initial.cast<float>());
    // icp.align(Final);

    if (icp.hasConverged()) {
        std::cout << "ICP has converged!" << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:\n"
                  << icp.getFinalTransformation() << std::endl;
        fitness = icp.getFitnessScore();
        return icp.getFinalTransformation().cast<double>();
    } else {
        std::cout << "ICP did not converge." << std::endl;
        return initial;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RelocalizationNode::CropMapInFov(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map,
    const Eigen::Matrix4d &pose_estimation,
    const nav_msgs::msg::Odometry &cur_odom, double FOV, double FOV_FAR) {
    // pose_estimation是map to odom
    Eigen::Matrix4d T_odom_to_base_link = PoseToMat(cur_odom.pose.pose);
    Eigen::Matrix4d T_map_to_base_link = pose_estimation * T_odom_to_base_link;

    Eigen::Matrix4d T_base_link_to_map = T_map_to_base_link.inverse();

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_in_base_link(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::transformPointCloud(*global_map, *global_map_in_base_link,
                             T_base_link_to_map);

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(
        new pcl::ConditionAnd<pcl::PointXYZ>());
    // float min_radius = 5.0f;
    float max_radius = FOV_FAR;

    // 添加条件：点到原点的距离在min_radius和max_radius之间
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT,
                                                -max_radius)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT,
                                                max_radius)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT,
                                                -max_radius)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT,
                                                max_radius)));

    // 创建条件滤波器
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_removal;
    cond_removal.setCondition(range_cond);
    cond_removal.setInputCloud(global_map_in_base_link);
    cond_removal.setKeepOrganized(false);  // 如果设置为true，保留点云结构

    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_map(
        new pcl::PointCloud<pcl::PointXYZ>);
    cond_removal.filter(*crop_map);
    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_map_in_map_frame(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*crop_map, *crop_map_in_map_frame,
                             T_map_to_base_link);

    PublishCropMap(crop_map_in_map_frame);

    return crop_map_in_map_frame;
}

void RelocalizationNode::PublishCropMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &crop_map) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*crop_map, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    pub_crop_map_->publish(msg);
}

bool RelocalizationNode::InitGlobalMap(std::string map_path) {
    // 耗时
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *global_map_) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return false;
    }
    return true;
}

Eigen::Matrix4d RelocalizationNode::PoseToMat(
    const geometry_msgs::msg::Pose &pose) {
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
    const Eigen::Matrix4d &mat) {
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizationNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
