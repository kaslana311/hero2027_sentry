// #pragma once

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cloud_analys/eigen_alias.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using namespace analy_utils;
class Handler : public rclcpp::Node
{
  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    int unfinished_frame_cnt{0};
    Pose pc_pose;
    pcl::PointCloud<pcl::PointXYZ> pc;
    rclcpp::TimerBase::SharedPtr update_timer;
    std::mutex update_lock;
  } rc_;
  struct CostPub
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_pc_pub;
  } cost_pub_;
  struct RobotState
  {
    Vec3f p, v, a, j;
    double yaw;
    double rcv_time;
    bool rcv{false};
    Quatf q;
  } robot_state_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    auto pose = std::make_pair(
      Vec3f(
        odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z),
      Quatf(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z));
    robot_state_.p = pose.first;
    robot_state_.q = pose.second;
    // robot_state_.v = Vec3f(
    //   odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y,
    //   odom_msg->twist.twist.linear.z);
    robot_state_.rcv_time = this->get_clock()->now().seconds();
    robot_state_.rcv = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (!robot_state_.rcv) {
      std::cout << " -- [ROS] No odom received, skip cloud callback." << std::endl;
      return;
    }
    double cbk_t = this->get_clock()->now().seconds();
    if (cbk_t - robot_state_.rcv_time > odom_timeout) {
      std::cout << " -- [ROS] Odom timeout, skip cloud callback." << std::endl;
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> temp_pc;
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    rc_.update_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.update_lock.unlock();
  }

  void updateCallback()
  {
    auto start_time = std::chrono::steady_clock::now();
    if (map_empty_) {
      static double last_print_t = this->get_clock()->now().seconds();
      double cur_t = this->get_clock()->now().seconds();
      if ((cur_t - last_print_t > 1.0)) {
        std::cout << " -- [ROG WARN] No point cloud input, check the topic name." << std::endl;
        last_print_t = cur_t;
      }
      return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
      return;
    }

    if (rc_.unfinished_frame_cnt > 1) {
      std::cout << " -- [ROG WARN] Unfinished frame cnt > 1, the map may not "
                   "work in real-time"
                << std::endl;
    }
    static pcl::PointCloud<pcl::PointXYZ> temp_pc;
    static Pose temp_pose;

    rc_.update_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    rc_.unfinished_frame_cnt = 0;
    rc_.update_lock.unlock();

    CloudAnalys(temp_pc, temp_pose);

    /************* */
    auto end_time = std::chrono::steady_clock::now();
    double time_consuming =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << " -- [ROG INFO] Map updated in " << time_consuming << " ms." << std::endl;
  }

  void CloudAnalys(pcl::PointCloud<pcl::PointXYZ> & cloud, Pose & pose)
  {
    double min_x = pose.first[0] - half_map_size;
    double max_x = pose.first[0] + half_map_size;
    double min_y = pose.first[1] - half_map_size;
    double max_y = pose.first[1] + half_map_size;
    auto cptr = cloud.makeShared();
    pass_filter_.setInputCloud(cptr);
    pass_filter_.setFilterFieldName("x");
    pass_filter_.setFilterLimits(min_x, max_x);
    pass_filter_.filter(*cptr);
    pass_filter_.setInputCloud(cptr);
    pass_filter_.setFilterFieldName("y");
    pass_filter_.setFilterLimits(min_y, max_y);
    pass_filter_.filter(*cptr);
    memset(min_hmap, 100.0, sizeof(double) * Map_SIZE_D * Map_SIZE_D);

    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    for (const auto & point : cloud) {
      if (point.z < pose.first[2] - 0.35) continue;
      if (point.z > pose.first[2] + higher_than_car_threshold) continue;
      int idx = static_cast<int>((point.x - min_x) / resolution);
      int idy = static_cast<int>((point.y - min_y) / resolution);
      if (idx < 0 || idx >= Map_SIZE_D || idy < 0 || idy >= Map_SIZE_D) {
        continue;
      }
      //   points[idx + idy * Map_SIZE_D].push_back(point);
      if (point.z < min_hmap[idx + idy * Map_SIZE_D]) {
        min_hmap[idx + idy * Map_SIZE_D] = point.z;
      }
    }
    unsigned int visit[Map_SIZE_D * Map_SIZE_D] = {0};
    for (auto & p_c : cloud) {
      int idx = static_cast<int>((p_c.x - min_x) / resolution);
      int idy = static_cast<int>((p_c.y - min_y) / resolution);
      if (idx < 0 || idx >= Map_SIZE_D || idy < 0 || idy >= Map_SIZE_D) {
        continue;
      }
      if (visit[idx + idy * Map_SIZE_D] > 3) continue;
      if (p_c.z - min_hmap[idx + idy * Map_SIZE_D] > heighter_than_ground_threshold) {
        temp_cloud.push_back(p_c);
        visit[idx + idy * Map_SIZE_D]++;
      }
    }
    if (temp_cloud.size() == 0) {
      //   std::cout << " -- [ROG WARN] No valid points in cloud analys." << std::endl;
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(temp_cloud, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = "odom";
    cost_pub_.cost_pc_pub->publish(cloud_msg);
  }

public:
  Handler() : Node("cloud_analys_node")
  {
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    Map_SIZE_D = static_cast<int>(half_map_size * 2 / resolution);
    min_hmap = new double[Map_SIZE_D * Map_SIZE_D];
    max_hmap = new double[Map_SIZE_D * Map_SIZE_D];
    this->declare_parameter<std::string>("odom_topic", "lidar_odometry");
    this->declare_parameter<std::string>("cloud_topic", "registered_scan");
    this->declare_parameter<double>("half_map_size", 10.0);
    this->declare_parameter<double>("resolution", 0.05);
    this->declare_parameter<double>("heighter_than_ground_threshold", 0.02);
    this->declare_parameter<double>("higher_than_car_threshold", 0.5);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("cloud_topic", cloud_topic);
    this->get_parameter("half_map_size", half_map_size);
    this->get_parameter("resolution", resolution);
    this->get_parameter("heighter_than_ground_threshold", heighter_than_ground_threshold);
    this->get_parameter("higher_than_car_threshold", higher_than_car_threshold);
    cost_pub_.cost_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", qos);
    {
      rc_.odom_me_cbk_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos, std::bind(&Handler::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      rc_.cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, qos, std::bind(&Handler::cloudCallback, this, std::placeholders::_1), so);
      rc_.update_cbk_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = this->create_wall_timer(
        std::chrono::milliseconds(1),  // 0.001秒，即1毫秒
        std::bind(&Handler::updateCallback, this), rc_.update_cbk_group);
    }
  }

private:
  //   pcl::VoxelGrid<pcl::PointXYZI> vox_flt;
  double half_map_size{10.0};
  bool map_empty_{true};
  double odom_timeout{0.1};
  double *min_hmap, *max_hmap;
  int Map_SIZE_D;
  std::string odom_topic = "lidar_odometry", cloud_topic = "registered_scan";
  double resolution{0.05};
  double heighter_than_ground_threshold{0.02};
  //   double lower_than_car_threshold{0.35};
  double higher_than_car_threshold{0.5};
  struct op
  {
    bool operator()(const Vec2i & a, const Vec2i & b) const
    {
      if (a[0] != b[0]) return a[0] < b[0];
      return a[1] < b[1];
    }
  };
  std::map<Vec2i, int, op> time_stamp_lists;
  pcl::PassThrough<pcl::PointXYZ> pass_filter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Handler>();
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}