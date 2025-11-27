#include <pcl/filters/passthrough.h>

#include "relocalization_node.hpp"
using namespace small_gicp;
RelocalizationNode::RelocalizationNode()
    : Node("relocalization_node"),
      global_map_(new pcl::PointCloud<pcl::PointXYZ>),
      result_t_(Eigen::Isometry3d::Identity()),
      cur_scan_(new pcl::PointCloud<pcl::PointXYZ>),
      accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    RCLCPP_INFO(this->get_logger(), "RelocalizationNode init");
    this->declare_parameter("init_pose",
                            std::vector<double>{0., 0., 0., 0., 0., 0.});

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
    this->get_parameter("init_pose", init_pose_);
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

    if (!init_pose_.empty() && init_pose_.size() >= 6) {
        result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
        result_t_.linear() =
            Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX())
                .toRotationMatrix();
    }
    this->pose_estimation_ = result_t_.matrix();  // pose_est yuce ,res,pub
    std::cout << "pose_estimation:\n" << pose_estimation_ << std::endl;
    std::cout << "pcd_path:" << pcd_path_ << std::endl;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
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
        "registered_scan", 1,
        std::bind(&RelocalizationNode::SaveCurScanCallback, this,
                  std::placeholders::_1));
    std::cout << "running relocazation" << std::endl;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RelocalizationNode::GlobalLocalizationTimerCallback, this));
    timer_pub = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RelocalizationNode::publishTransform, this));
}
void RelocalizationNode::publishTransform() {
    if (!this->registration_success_) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp =
        this->now() + rclcpp::Duration::from_seconds(0.1);
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    Eigen::Isometry3d temp;
    {
        std::lock_guard<std::mutex> lock(mtx1);
        temp = this->result_t_;
    }
    const Eigen::Vector3d translation = temp.translation();
    const Eigen::Quaterniond rotation(temp.rotation());

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
    // if (!first_lidar_pos) {
    //   try {
    //     transform_stampedlidarodom = tf_buffer_->lookupTransform(
    //       "odom", "front_mid360", this->now(),
    //       rclcpp::Duration::from_seconds(0.5));
    //     this->transform_stampedlidarodom.child_frame_id = "lidar_odom";
    //     first_lidar_pos = true;
    //   } catch (tf2::TransformException & ex) {
    //     RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning
    //     identity.", ex.what());
    //   }
    // }
    // if (first_lidar_pos) {
    //   transform_stampedlidarodom.header.stamp = this->now() +
    //   rclcpp::Duration::from_seconds(0.1);
    //   tf_broadcaster_->sendTransform(transform_stampedlidarodom);
    // }
}
void RelocalizationNode::SaveCurScanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //累计10帧
    std::lock_guard<std::mutex> lock(mtx);
    pcl::fromROSMsg(*msg, *cur_scan_);
    //滤波
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

void RelocalizationNode::GlobalLocalizationTimerCallback() {
    if (!has_scan_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "no scan msg");
        return;
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
    std::lock_guard<std::mutex> lock(mtx);
    double best_fitness = 0.03;
    Eigen::Matrix4d best_T = result_t_.matrix();
    if (this->registration_success_)
        this->register_->optimizer.max_iterations = 5;
    for (double dyaw : yaw_seeds) {
        Eigen::Matrix4d seed = RotZ(dyaw) * result_t_.matrix();
        double f = 1e9;
        Eigen::Isometry3d res =
            SmallIcpRegistration(accumulated_cloud, seed, f);
        if (f < best_fitness) {
            best_fitness = f;
            best_T = res.matrix();
            break;
        }
    }
    //此时得到了最好的变幻关系
    rclcpp::Time end_icp_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Registration time: %f, best fitness: %f",
                (end_icp_time - begin_icp_time).seconds(), best_fitness);
    //评价可行性
    const bool fitness_ok = best_fitness < fitness_threshold_;
    const bool trans_ok =
        TransDelta(best_T, result_t_.matrix()) <= gate_trans_m_;
    const bool yaw_ok = YawDelta(best_T, result_t_.matrix()) <= yaw_gate;
    const double roll_gate = gate_roll_deg_ * M_PI / 180.0;
    const double pitch_gate = gate_pitch_deg_ * M_PI / 180.0;
    const bool roll_ok = RollDelta(best_T, result_t_.matrix()) <= roll_gate;
    const bool pitch_ok = PitchDelta(best_T, result_t_.matrix()) <= pitch_gate;

    if (fitness_ok && trans_ok && yaw_ok && pitch_ok && roll_ok) {
        RCLCPP_INFO(this->get_logger(),
                    "registration can be trusted (fitness=%.5f, dtrans=%.3f, "
                    "droll=%.2fdeg, dpitch=%.2fdeg, dyaw=%.2fdeg)",
                    best_fitness, TransDelta(best_T, pose_estimation_),
                    RollDelta(best_T, pose_estimation_) * 180.0 / M_PI,
                    PitchDelta(best_T, pose_estimation_) * 180.0 / M_PI,
                    YawDelta(best_T, pose_estimation_) * 180.0 / M_PI);

        Eigen::Matrix4d T_pub = best_T;
        if (enforce_rp_zero_) {
            ZeroRollPitchKeepYaw(T_pub);
        }
        {
            std::lock_guard<std::mutex> lock(mtx1);
            result_t_.matrix() = T_pub;
        }
        registration_success_ = true;  // 设置配准成功标志
        registration_fail_count_ = 0;  // 成功后清零
    }
    // else {
    //   RCLCPP_WARN(
    //     this->get_logger(),
    //     "registration rejected (fitness=%.5f, dtrans=%.3f, "
    //     "droll=%.2fdeg, dpitch=%.2fdeg, dyaw=%.2fdeg)",
    //     best_fitness, TransDelta(best_T, pose_estimation_),
    //     RollDelta(best_T, pose_estimation_) * 180.0 / M_PI,
    //     PitchDelta(best_T, pose_estimation_) * 180.0 / M_PI,
    //     YawDelta(best_T, pose_estimation_) * 180.0 / M_PI);
    //   registration_fail_count_++;

    //   if (registration_fail_count_ >= max_fail_before_fallback_) {
    //     RCLCPP_WARN(
    //       this->get_logger(),
    //       "Registration failed over %d times, publishing "
    //       "fallback (%s)!",
    //       max_fail_before_fallback_, fallback_policy_.c_str());
    //     // if (fallback_policy_ == "identity") {
    //     //   //todo
    //     // } else {
    //     //   //todo
    //     // }
    //     // publishTransform();
    //     registration_success_ = true;
    //     registration_fail_count_ = 0;  // 重置计数
    //   }
    // }
    has_scan_ = false;
    this->scan_count = 0;
    this->accumulated_cloud->clear();
}
// converged return res,false return pre
Eigen::Isometry3d RelocalizationNode::SmallIcpRegistration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_scan, Eigen::Matrix4d pre,
    double& fitness) {
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
    init_T_target_source.matrix() = pre;  //复制预测转换
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
        return T;
    } else {
        std::cout << "Gicp registration not converged" << std::endl;
        return init_T_target_source;
    }
}

bool RelocalizationNode::InitGlobalMap(std::string map_path) {
    // 耗时
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *global_map_) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return false;
    }
    Eigen::Affine3d odom_to_lidar_odom;
    while (true) {
        try {
            auto tf_stamped = tf_buffer_->lookupTransform(
                "base_footprint", "front_mid360", this->now(),
                rclcpp::Duration::from_seconds(1.0));
            odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "odom_to_lidar_odom: translation = "
                    << odom_to_lidar_odom.translation().transpose()
                    << ", rpy = "
                    << odom_to_lidar_odom.rotation()
                           .eulerAngles(0, 1, 2)
                           .transpose());
            break;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...",
                        ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
    pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // 开始执行循环
    rclcpp::shutdown();
    return 0;
}
