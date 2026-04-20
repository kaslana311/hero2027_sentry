#include "pcd2pgm/pcd2pgm.hpp"

#include <algorithm>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include <fstream>
#include <rclcpp/qos.hpp>

namespace pcd2pgm {
PCLFiltersNode::PCLFiltersNode(const rclcpp::NodeOptions& options)
    : Node("pcd2pgm", options) {
    declareParameters();
    getParameters();

    rclcpp::QoS map_qos(10);  // initialize to default
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);

    pcd_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_name_, map_qos);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu",
                pcd_cloud->points.size());

    passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
    radiusOutlierFilter(cloud_after_PassThrough_, thre_radius_,
                        thres_point_count_);
    setMapTopicMsg(cloud_after_Radius_, map_topic_msg_);
    if (generate_contour_map_) {
        std::vector<uint8_t> contour_grid;
        int contour_width = 0;
        int contour_height = 0;
        double contour_x_min = 0.0;
        double contour_y_min = 0.0;
        if (buildContourGrid(pcd_cloud, contour_grid, contour_width,
                             contour_height, contour_x_min, contour_y_min)) {
            saveContourMapFiles(contour_grid, contour_width, contour_height,
                                contour_x_min, contour_y_min);
        }
    }

    time_ = create_wall_timer(std::chrono::seconds(1),
                              std::bind(&PCLFiltersNode::publishMap, this));
}

PCLFiltersNode::~PCLFiltersNode() {}

void PCLFiltersNode::publishMap() { map_publisher_->publish(map_topic_msg_); }

void PCLFiltersNode::declareParameters() {
    declare_parameter("file_directory", "/home/lihanchen/Downloads/pcd2pgm/");
    declare_parameter("file_name", "RMUC");
    declare_parameter("thre_z_min", 0.5);
    declare_parameter("thre_z_max", 2.0);
    declare_parameter("flag_pass_through", false);
    declare_parameter("thre_radius", 0.5);
    declare_parameter("map_resolution", 0.05);
    declare_parameter("thres_point_count", 10);
    declare_parameter("map_topic_name", "map");
    declare_parameter("generate_contour_map", true);
    declare_parameter("contour_file_name", "fast_lio_latest_map_contour");
    declare_parameter("contour_resolution", 0.01);
    declare_parameter("contour_z_min", 0.1);
    declare_parameter("contour_z_max", 0.5);
    declare_parameter("contour_normal_radius", 0.15);
    declare_parameter("contour_normal_z_abs_max", 0.5);
    declare_parameter("contour_padding", 1.0);
}

void PCLFiltersNode::getParameters() {
    get_parameter("file_directory", file_directory_);
    get_parameter("file_name", file_name_);
    get_parameter("thre_z_min", thre_z_min_);
    get_parameter("thre_z_max", thre_z_max_);
    get_parameter("flag_pass_through", flag_pass_through_);
    get_parameter("thre_radius", thre_radius_);
    get_parameter("map_resolution", map_resolution_);
    get_parameter("thres_point_count", thres_point_count_);
    get_parameter("map_topic_name", map_topic_name_);
    get_parameter("generate_contour_map", generate_contour_map_);
    get_parameter("contour_file_name", contour_file_name_);
    get_parameter("contour_resolution", contour_resolution_);
    get_parameter("contour_z_min", contour_z_min_);
    get_parameter("contour_z_max", contour_z_max_);
    get_parameter("contour_normal_radius", contour_normal_radius_);
    get_parameter("contour_normal_z_abs_max", contour_normal_z_abs_max_);
    get_parameter("contour_padding", contour_padding_);
    pcd_file_ = file_directory_ + file_name_ + ".pcd";
}

void PCLFiltersNode::passThroughFilter(const double& thre_low,
                                       const double& thre_high,
                                       const bool& flag_in) {
    // 将 std::shared_ptr 转换为 boost::shared_ptr，并将其转换为常量指针
    // boost::shared_ptr <const pcl::PointCloud<pcl::PointXYZ>> ptr(new
    // pcl::PointCloud<pcl::PointXYZ>(*pcd_cloud)); 将 boost::shared_ptr 转换为
    // ConstPtr const pcl::PointCloud<pcl::PointXYZ>::ConstPtr const_ptr = ptr;
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr const_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*pcd_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr capt(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud_after_PassThrough_ = capt;

    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(const_ptr);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(thre_low, thre_high);
    passthrough.setNegative(flag_in);
    passthrough.filter(*cloud_after_PassThrough_);

    RCLCPP_INFO(get_logger(),
                "After PassThrough filtering, point cloud size: %lu",
                cloud_after_PassThrough_->points.size());
}

void PCLFiltersNode::radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_cloud0, const double& radius,
    const int& thre_count) {
    // 将 std::shared_ptr 转换为 boost::shared_ptr，并将其转换为常量指针
    // boost::shared_ptr <const pcl::PointCloud<pcl::PointXYZ>> ptr0(new
    // pcl::PointCloud<pcl::PointXYZ>(*pcd_cloud0)); 将 boost::shared_ptr 转换为
    // ConstPtr const pcl::PointCloud<pcl::PointXYZ>::ConstPtr const_ptr0 =
    // ptr0;
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr const_ptr0 =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*pcd_cloud0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr car(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_after_Radius_ = car;

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
    radiusoutlier.setInputCloud(const_ptr0);
    radiusoutlier.setRadiusSearch(radius);
    radiusoutlier.setMinNeighborsInRadius(thre_count);
    radiusoutlier.filter(*cloud_after_Radius_);

    RCLCPP_INFO(get_logger(),
                "After RadiusOutlier filtering, point cloud size: %lu",
                cloud_after_Radius_->points.size());
}

void PCLFiltersNode::setMapTopicMsg(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    nav_msgs::msg::OccupancyGrid& msg) {
    msg.header.stamp = now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = now();
    msg.info.resolution = map_resolution_;

    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    if (cloud->points.empty()) {
        RCLCPP_WARN(get_logger(), "Point cloud is empty!");
        return;
    }

    for (const auto& point : cloud->points) {
        x_min = std::min(x_min, (double)point.x);
        x_max = std::max(x_max, (double)point.x);
        y_min = std::min(y_min, (double)point.y);
        y_max = std::max(y_max, (double)point.y);
    }

    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
    msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    for (const auto& point : cloud->points) {
        int i = std::floor((point.x - x_min) / map_resolution_);
        int j = std::floor((point.y - y_min) / map_resolution_);

        if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
            msg.data[i + j * msg.info.width] = 100;
        }
    }

    RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

bool PCLFiltersNode::buildContourGrid(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    std::vector<uint8_t>& contour_grid, int& width, int& height, double& x_min,
    double& y_min) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(contour_z_min_, contour_z_max_);
    pass.filter(*z_filtered);

    if (z_filtered->empty()) {
        RCLCPP_WARN(get_logger(),
                    "Contour map: empty after z filter [%f, %f].",
                    contour_z_min_, contour_z_max_);
        return false;
    }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(z_filtered);
    ne.setSearchSurface(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(contour_normal_radius_);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr contour_pts(
        new pcl::PointCloud<pcl::PointXYZ>);
    contour_pts->reserve(z_filtered->size());
    for (size_t i = 0; i < z_filtered->size(); ++i) {
        if (std::abs(normals->points[i].normal_z) <= contour_normal_z_abs_max_) {
            contour_pts->push_back(z_filtered->points[i]);
        }
    }

    if (contour_pts->empty()) {
        RCLCPP_WARN(get_logger(), "Contour map: no points after normal filter.");
        return false;
    }

    double x_max = std::numeric_limits<double>::lowest();
    double y_max = std::numeric_limits<double>::lowest();
    x_min = std::numeric_limits<double>::max();
    y_min = std::numeric_limits<double>::max();
    for (const auto& pt : contour_pts->points) {
        x_min = std::min(x_min, static_cast<double>(pt.x));
        x_max = std::max(x_max, static_cast<double>(pt.x));
        y_min = std::min(y_min, static_cast<double>(pt.y));
        y_max = std::max(y_max, static_cast<double>(pt.y));
    }

    x_min -= contour_padding_;
    y_min -= contour_padding_;
    x_max += contour_padding_;
    y_max += contour_padding_;

    width = std::max(1, static_cast<int>(std::ceil((x_max - x_min) / contour_resolution_)));
    height =
        std::max(1, static_cast<int>(std::ceil((y_max - y_min) / contour_resolution_)));
    contour_grid.assign(width * height, 0);

    for (const auto& pt : contour_pts->points) {
        const int cx =
            static_cast<int>(std::floor((pt.x - x_min) / contour_resolution_));
        const int cy =
            static_cast<int>(std::floor((pt.y - y_min) / contour_resolution_));
        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
            contour_grid[cy * width + cx] = 1;
        }
    }

    RCLCPP_INFO(get_logger(),
                "Contour map generated: %zu contour points, %dx%d @ %.3fm.",
                contour_pts->size(), width, height, contour_resolution_);
    return true;
}

bool PCLFiltersNode::saveContourMapFiles(const std::vector<uint8_t>& contour_grid,
                                         int width, int height, double x_min,
                                         double y_min) const {
    if (contour_grid.empty() || width <= 0 || height <= 0) {
        return false;
    }

    const std::string base = "/home/dji/hero2026_-sentry/src/planning/map_server/contour_map/" + contour_file_name_;
    const std::string pgm_path = base + ".pgm";
    const std::string yaml_path = base + ".yaml";

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open contour pgm for write: %s",
                     pgm_path.c_str());
        return false;
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const uint8_t occupied = contour_grid[y * width + x] > 0 ? 0 : 255;
            pgm.write(reinterpret_cast<const char*>(&occupied), 1);
        }
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open contour yaml for write: %s",
                     yaml_path.c_str());
        return false;
    }
    yaml << "image: " << contour_file_name_ << ".pgm\n";
    yaml << "resolution: " << contour_resolution_ << "\n";
    yaml << "origin: [" << x_min << ", " << y_min << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml.close();

    RCLCPP_INFO(get_logger(), "Contour map saved: %s and %s", pgm_path.c_str(),
                yaml_path.c_str());
    return true;
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::PCLFiltersNode)
