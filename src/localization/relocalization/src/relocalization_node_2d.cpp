#include "relocalization_node_2d.hpp"

#include <pcl/io/pcd_io.h>

RelocalizationNode::RelocalizationNode(QObject* parent)
    : QObject(parent),
      Node("relocalization_node"),
      global_map_(new pcl::PointCloud<pcl::PointXYZ>),
      result_t_(Eigen::Isometry3d::Identity()),
      cur_scan_(new pcl::PointCloud<pcl::PointXYZ>),
      accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
    RCLCPP_INFO(this->get_logger(),
                "RelocalizationNode (2D Correlative Matching) init");

    this->declare_parameter("init_pose",
                            std::vector<double>{0., 0., 0., 0., 0., 0.});
    this->declare_parameter(
        "pcd_path",
        "/home/fyk/Projects/ros2_ws/src/relocalization/PCD/voxel2025RMUC.pcd");

    // 2D BEV Match Config
    this->declare_parameter("map_z_min", 0.1);
    this->declare_parameter("map_z_max", 0.5);
    this->declare_parameter("scan_z_min", -0.2);
    this->declare_parameter("scan_z_max", 0.2);

    this->declare_parameter("resolution", 0.01);       // 1cm 精度
    this->declare_parameter("search_x_bounds", 0.08);  // X搜索 +-8cm
    this->declare_parameter("search_y_bounds", 0.08);  // Y搜索 +-8cm
    this->declare_parameter("search_yaw_bounds_deg",
                            10.0);                 // 偏航角搜索 +-10度
    this->declare_parameter("trans_step", 0.01);   // 平移搜索步长 1cm
    this->declare_parameter("yaw_step_deg", 0.5);  // 角度搜索步长 0.5度

    this->declare_parameter("max_scan_count", 10);  // 累积多少帧雷达数据

    this->get_parameter("init_pose", init_pose_);

    pcd_path_ = this->get_parameter("pcd_path").as_string();
    map_z_min_ = this->get_parameter("map_z_min").as_double();
    map_z_max_ = this->get_parameter("map_z_max").as_double();
    scan_z_min_ = this->get_parameter("scan_z_min").as_double();
    scan_z_max_ = this->get_parameter("scan_z_max").as_double();
    resolution_ = this->get_parameter("resolution").as_double();
    search_x_bounds_ = this->get_parameter("search_x_bounds").as_double();
    search_y_bounds_ = this->get_parameter("search_y_bounds").as_double();
    search_yaw_bounds_deg_ =
        this->get_parameter("search_yaw_bounds_deg").as_double();
    trans_step_ = this->get_parameter("trans_step").as_double();
    yaw_step_deg_ = this->get_parameter("yaw_step_deg").as_double();
    max_scan_count_ = this->get_parameter("max_scan_count").as_int();

    if (!init_pose_.empty() && init_pose_.size() >= 6) {
        result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
        result_t_.linear() =
            Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX())
                .toRotationMatrix();
    }
    pose_estimation_ = result_t_.matrix();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    // Publishers
    pub_map_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "pcd_init_map_grid", 1);
    pub_scan_2d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "cloud_2d_matched", 1);

    if (InitGlobalMap(pcd_path_)) {
        createGridMap();
    }

    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 1,
        std::bind(&RelocalizationNode::SaveCurScanCallback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RelocalizationNode::GlobalLocalizationTimerCallback, this));
}

bool RelocalizationNode::InitGlobalMap(std::string map_path) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s",
                     map_path.c_str());
        return false;
    }
    return true;
}

void RelocalizationNode::createGridMap() {
    if (global_map_->empty()) return;

    // Z轴截取先验场地障碍物点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(global_map_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(map_z_min_, map_z_max_);
    pass.filter(*cloud_filtered);

    if (cloud_filtered->empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Map PC is empty after PassThrough (z:[%f, %f])!",
                    map_z_min_, map_z_max_);
        return;
    }

    map_x_min_ = std::numeric_limits<double>::max();
    map_x_max_ = std::numeric_limits<double>::lowest();
    map_y_min_ = std::numeric_limits<double>::max();
    map_y_max_ = std::numeric_limits<double>::lowest();

    for (const auto& pt : cloud_filtered->points) {
        map_x_min_ = std::min(map_x_min_, (double)pt.x);
        map_x_max_ = std::max(map_x_max_, (double)pt.x);
        map_y_min_ = std::min(map_y_min_, (double)pt.y);
        map_y_max_ = std::max(map_y_max_, (double)pt.y);
    }

    // 给栅格边缘增加Padding避免越界
    map_x_min_ -= 1.0;
    map_x_max_ += 1.0;
    map_y_min_ -= 1.0;
    map_y_max_ += 1.0;

    map_width_ = std::ceil((map_x_max_ - map_x_min_) / resolution_);
    map_height_ = std::ceil((map_y_max_ - map_y_min_) / resolution_);
    grid_map_data_.assign(map_width_ * map_height_, 0);

    for (const auto& pt : cloud_filtered->points) {
        int cx = std::floor((pt.x - map_x_min_) / resolution_);
        int cy = std::floor((pt.y - map_y_min_) / resolution_);
        if (cx >= 0 && cx < map_width_ && cy >= 0 && cy < map_height_) {
            // 给像素上色(占据)
            grid_map_data_[cy * map_width_ + cx] = 1;
        }
    }

    map_grid_msg_.header.frame_id = "map";
    map_grid_msg_.info.resolution = resolution_;
    map_grid_msg_.info.width = map_width_;
    map_grid_msg_.info.height = map_height_;
    map_grid_msg_.info.origin.position.x = map_x_min_;
    map_grid_msg_.info.origin.position.y = map_y_min_;
    map_grid_msg_.info.origin.position.z = 0.0;
    map_grid_msg_.info.origin.orientation.w = 1.0;

    map_grid_msg_.data.assign(map_width_ * map_height_, 0);
    for (size_t i = 0; i < grid_map_data_.size(); ++i) {
        map_grid_msg_.data[i] = grid_map_data_[i] > 0 ? 100 : 0;
    }
    pub_map_grid_->publish(map_grid_msg_);
    has_global_map_ = true;
    RCLCPP_INFO(this->get_logger(),
                "2D Map Generated: Width %d, Height %d, Resolution %f",
                map_width_, map_height_, resolution_);
}

void RelocalizationNode::SaveCurScanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (registration_success_ || scan_count >= max_scan_count_) return;
    std::lock_guard<std::mutex> lock(mtx);
    pcl::fromROSMsg(*msg, *cur_scan_);
    *accumulated_cloud_ += *cur_scan_;
    scan_count++;
    if (scan_count == max_scan_count_) {
        has_scan_ = true;
    }
}

void RelocalizationNode::GlobalLocalizationTimerCallback() {
    if (registration_success_) return;
    if (!has_global_map_) return;
    if (!has_scan_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Accumulating lidar scan, wait...");
        return;
    }

    rclcpp::Time begin_time = this->now();

    // 根据 Sensor 高度截断扫描轮廓
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(accumulated_cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(scan_z_min_, scan_z_max_);
    pass.filter(*cloud_filtered);

    scan_2d_points_.clear();
    for (const auto& pt : cloud_filtered->points) {
        scan_2d_points_.emplace_back(pt.x, pt.y);
    }

    if (scan_2d_points_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "No scan points retained after Z-filter [%f, %f]!",
                    scan_z_min_, scan_z_max_);
        scan_count = 0;
        has_scan_ = false;
        accumulated_cloud_->clear();
        return;
    }

    // 核心匹配算法
    Eigen::Matrix4d best_T = CorrelativeMatch();
    rclcpp::Time end_time = this->now();
    RCLCPP_INFO(this->get_logger(),
                "Correlative Matching Done! Time cost: %f ms",
                (end_time - begin_time).seconds() * 1000.0);

    // 回设配准参数并发布TF
    result_t_.matrix() = best_T;
    if (!qt_control.load()) {
        publishTransform();
        registration_success_ = true;

        // --- 可视化: 将最后匹配出来的扫描点标为蓝色 ---
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_scan(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& pt2 : scan_2d_points_) {
            // 注意: 这里为了核对是否贴合，我们将点本身变换到匹配好的 map
            // 坐标系下发布
            Eigen::Matrix4d T = best_T;
            double final_x = T(0, 0) * pt2.x() + T(0, 1) * pt2.y() + T(0, 3);
            double final_y = T(1, 0) * pt2.x() + T(1, 1) * pt2.y() + T(1, 3);

            pcl::PointXYZRGB p;
            p.x = final_x;
            p.y = final_y;
            p.z = 0.0;
            p.r = 0;
            p.g = 150;
            p.b = 255;  // 扫描线浅蓝色
            colored_scan->points.push_back(p);
        }
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*colored_scan, out_msg);
        out_msg.header.frame_id = "map";
        out_msg.header.stamp = this->now();
        pub_scan_2d_->publish(out_msg);

        // 刷新发布一次红色地图框以便于与蓝色点云对比
        map_grid_msg_.header.stamp = this->now();
        pub_map_grid_->publish(map_grid_msg_);
    }

    reloc_res relocResult;
    relocResult.isOk = true;
    emit alignResultSignal(relocResult);

    accumulated_cloud_->clear();
}

Eigen::Matrix4d RelocalizationNode::CorrelativeMatch() {
    double init_x = pose_estimation_(0, 3);
    double init_y = pose_estimation_(1, 3);
    double init_yaw = YawFromMat(pose_estimation_);

    int best_score = -1;
    double best_x = init_x, best_y = init_y, best_yaw = init_yaw;

    double yaw_rad_limit = search_yaw_bounds_deg_ * M_PI / 180.0;
    double yaw_step_rad = yaw_step_deg_ * M_PI / 180.0;

    // 滑动窗口搜索
    for (double dyaw = -yaw_rad_limit; dyaw <= yaw_rad_limit + 1e-6;
         dyaw += yaw_step_rad) {
        double c_yaw = std::cos(init_yaw + dyaw);
        double s_yaw = std::sin(init_yaw + dyaw);

        // 每转一个大角度，生成一批被旋转的点，省去后续循环重复计算
        std::vector<Eigen::Vector2d> rot_pts;
        rot_pts.reserve(scan_2d_points_.size());
        for (const auto& pt : scan_2d_points_) {
            rot_pts.emplace_back(c_yaw * pt.x() - s_yaw * pt.y(),
                                 s_yaw * pt.x() + c_yaw * pt.y());
        }

        for (double dx = -search_x_bounds_; dx <= search_x_bounds_ + 1e-6;
             dx += trans_step_) {
            for (double dy = -search_y_bounds_; dy <= search_y_bounds_ + 1e-6;
                 dy += trans_step_) {
                double tx = init_x + dx;
                double ty = init_y + dy;
                int score = 0;

                // 计算重合度得分
                for (const auto& rpt : rot_pts) {
                    double final_x = rpt.x() + tx;
                    double final_y = rpt.y() + ty;

                    int cx = std::floor((final_x - map_x_min_) / resolution_);
                    int cy = std::floor((final_y - map_y_min_) / resolution_);

                    // 如果坐标合法，且正中地图轮廓黑线
                    if (cx >= 0 && cx < map_width_ && cy >= 0 &&
                        cy < map_height_) {
                        if (grid_map_data_[cy * map_width_ + cx] > 0) {
                            score++;
                        }
                    }
                }

                if (score > best_score) {
                    best_score = score;
                    best_x = tx;
                    best_y = ty;
                    best_yaw = init_yaw + dyaw;
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Best Hit Score: %d / %zu", best_score,
                scan_2d_points_.size());

    Eigen::Matrix4d best_T = pose_estimation_;
    // 覆盖新的 x, y, yaw，Roll与Pitch维持初始估算（保证三维不倾斜）
    best_T(0, 3) = best_x;
    best_T(1, 3) = best_y;
    Eigen::Matrix3d r_mat;
    r_mat = Eigen::AngleAxisd(best_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(PitchFromMat(pose_estimation_),
                              Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(RollFromMat(pose_estimation_),
                              Eigen::Vector3d::UnitX());
    best_T.block<3, 3>(0, 0) = r_mat;

    return best_T;
}

void RelocalizationNode::publishTransform() {
    if (result_t_.matrix().isZero()) return;
    geometry_msgs::msg::TransformStamped t_stamped;
    t_stamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
    t_stamped.header.frame_id = "map";
    t_stamped.child_frame_id = "odom";

    t_stamped.transform.translation.x = result_t_.translation().x();
    t_stamped.transform.translation.y = result_t_.translation().y();
    t_stamped.transform.translation.z = result_t_.translation().z();

    Eigen::Quaterniond q(result_t_.rotation());
    t_stamped.transform.rotation.x = q.x();
    t_stamped.transform.rotation.y = q.y();
    t_stamped.transform.rotation.z = q.z();
    t_stamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t_stamped);
}

void RelocalizationNode::onSetAlignSignal(align_res alignResult) {
    qt_control.store(true);
    result_t_.translation() << alignResult.x, alignResult.y, alignResult.z;
    result_t_.linear() =
        Eigen::AngleAxisd(alignResult.roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(alignResult.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(alignResult.yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    publishTransform();
    registration_success_ = true;
    std::cout << "Manual align set to:
                 " << result_t_.matrix() << std::endl;
        emit setResultSignal(true);
}

void RelocalizationNode::OnRetrySet(bool isOk) {
    std::cout << "Retry relocalization requested." << std::endl;
    if (isOk) {
        qt_control.store(false);
        if (!registration_success_) return;
        registration_success_ = false;
        scan_count = 0;
        has_scan_ = false;
    }
}
