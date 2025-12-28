// #pragma once

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl_ros/transforms.h>
// #include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cloud_analys/eigen_alias.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_ros/transforms.hpp"

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
    if (!tf_set) return;
    //TODO odometry在odom下，利用tf转为map下
    // tf2::doTransform(odom_msg->pose.pose, odom_msg->pose.pose, tf_stamped);
    odom_msg->pose.pose.position.x += tf_t.getOrigin().x();
    odom_msg->pose.pose.position.y += tf_t.getOrigin().y();
    odom_msg->pose.pose.position.z += tf_t.getOrigin().z();
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
    sensor_msgs::msg::PointCloud2 out;
    if (!tf_set) {
      try {
        tf_stamped = this->tf_buffer_->lookupTransform(
          "map", cloud_msg->header.frame_id, rclcpp::Time(cloud_msg->header.stamp),
          rclcpp::Duration::from_seconds(0.5));

        // tf2::fromMsg(tf_stamped.transform, tf_t);
        tf_t = tf2::Transform(
          tf2::Quaternion(
            tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y,
            tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w),
          tf2::Vector3(
            tf_stamped.transform.translation.x, tf_stamped.transform.translation.y,
            tf_stamped.transform.translation.z));

        tf_set = true;
      } catch (...) {
        std::cout << " -- [ROS] Cloud callback pcl::fromROSMsg error, skip this frame."
                  << std::endl;
        return;
      }
    }
    pcl_ros::transformPointCloud("map", tf_t, *cloud_msg, out);
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
    pcl::fromROSMsg(out, temp_pc);
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
    std::cout << "cloud size " << cloud.size() << std::endl;
    std::cout << "robot pose " << pose.first.transpose() << std::endl;
    double min_x = pose.first[0] - half_map_size;
    min_x = std::max(min_x, 0.0);
    double max_x = pose.first[0] + half_map_size;
    max_x = std::min(max_x, global_map_size_x);
    double min_y = pose.first[1] - half_map_size;
    min_y = std::max(min_y, 0.0);
    double max_y = pose.first[1] + half_map_size;
    max_y = std::min(max_y, global_map_size_y);
    auto cptr = cloud.makeShared();
    pass_filter_.setInputCloud(cptr);
    pass_filter_.setFilterFieldName("x");
    pass_filter_.setFilterLimits(min_x, max_x);
    pass_filter_.filter(*cptr);
    pass_filter_.setInputCloud(cptr);
    pass_filter_.setFilterFieldName("y");
    pass_filter_.setFilterLimits(min_y, max_y);
    pass_filter_.filter(*cptr);
    double max_z = pose.first[2] + higher_than_car_threshold;
    pass_filter_.setInputCloud(cptr);
    pass_filter_.setFilterFieldName("z");
    pass_filter_.setFilterLimits(-0.3, max_z);
    pass_filter_.filter(*cptr);

    //遗忘若干帧没有观测到的障碍物
    for (auto it = pub_list.begin(); it != pub_list.end();) {
      (it->second)++;
      if (it->second > 5) {
        global_map_[it->first] = 0;
        it = pub_list.erase(it);
        continue;
      }
      ++it;
    }
    std::fill_n(min_hmap, Map_SIZE_D * Map_SIZE_D, 1e6);
    //得到最低高度图
    for (auto & point : cptr->points) {
      int xi = static_cast<int>((point.x - min_x) / resolution);
      int idy = static_cast<int>((point.y - min_y) / resolution);
      int index_l = xi + idy * Map_SIZE_D;
      if (point.z < min_hmap[index_l]) {
        min_hmap[index_l] = point.z;
      }
    }
    // 填充障碍地图
    obs_list_.clear();
    for (auto & point : cptr->points) {
      int xi = static_cast<int>((point.x - min_x) / resolution);
      int idy = static_cast<int>((point.y - min_y) / resolution);
      int index_l = xi + idy * Map_SIZE_D;
      if (point.z > min_hmap[index_l] + heighter_than_ground_threshold) {
        int idx_g = static_cast<int>((point.x) / resolution);
        int idy_g = static_cast<int>((point.y) / resolution);
        if (idx_g < 0 || idx_g >= global_map_xi || idy_g < 0 || idy_g >= global_map_yi) {
          continue;
        }
        int index_g = idx_g + idy_g * global_map_xi;
        if (global_map_[index_g] != 250) {
          global_map_[index_g] = 250;
          auto res = Vec2i{idx_g, idy_g};
          obs_list_.push_back(res);
          pub_list[index_g] = 0;
        }
      }
    }
    int px = static_cast<int>((pose.first[0]) / resolution);
    int py = static_cast<int>((pose.first[1]) / resolution);
    Vec2i start_idx = Vec2i{px, py};
    //射线处理，移除移动障碍物之前的轨迹
    RayHandle(start_idx, obs_list_);

    PubRes();
  }
  void PubRes()
  {
    // std::map<int, bool> visit;
    pcl::PointCloud<pcl::PointXYZ> pub_pc;
    pub_pc.reserve(pub_list.size());
    for (auto & item : pub_list) {
      // if (visit[item.first]) continue;

      int idx = item.first % global_map_xi;
      int idy = item.first / global_map_xi;
      pcl::PointXYZ pt;
      pt.x = idx * resolution;
      pt.y = idy * resolution;
      pt.z = 0.2f;

      // visit[item.first] = true;
      pub_pc.points.push_back(pt);
    }
    if (pub_pc.points.size() == 0) {
      return;
    }
    sensor_msgs::msg::PointCloud2 pub_msg;
    pcl::toROSMsg(pub_pc, pub_msg);
    pub_msg.header.frame_id = "map";
    pub_msg.header.stamp = this->get_clock()->now();
    cost_pub_.cost_pc_pub->publish(pub_msg);
  }
  void RayHandle(Vec2i & start_idx, std::vector<Vec2i> & temp_list)
  {
    // Bresenham算法实现2D网格的射线处理
    for (const auto & end_idx : temp_list) {
      int x0 = start_idx[0];
      int y0 = start_idx[1];
      int x1 = end_idx[0];
      int y1 = end_idx[1];

      int dx = std::abs(x1 - x0);
      int dy = std::abs(y1 - y0);
      int sx = (x0 < x1) ? 1 : -1;
      int sy = (y0 < y1) ? 1 : -1;
      int err = (dx > dy ? dx : -dy) / 2;
      int e2;

      while (true) {
        // 到达终点（障碍物本身）前停止，不清理障碍物格点
        if (x0 == x1 && y0 == y1) break;

        // 越界检查
        if (x0 < 0 || x0 >= global_map_xi || y0 < 0 || y0 >= global_map_yi) break;

        int index = x0 + y0 * global_map_xi;
        if (global_map_[index] != 0) {
          global_map_[index] = 0;
          pub_list.erase(index);
        }

        e2 = err;
        if (e2 > -dx) {
          err -= dy;
          x0 += sx;
        }
        if (e2 < dy) {
          err += dx;
          y0 += sy;
        }
      }
    }
  }

public:
  Handler() : Node("cloud_analys_node")
  {
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    Map_SIZE_D = static_cast<int>(half_map_size * 2 / resolution);
    min_hmap = new double[Map_SIZE_D * Map_SIZE_D];
    std::fill_n(min_hmap, Map_SIZE_D * Map_SIZE_D, 100.0);
    // max_hmap = new double[Map_SIZE_D * Map_SIZE_D];

    this->declare_parameter<std::string>("odom_topic", "lidar_odometry");
    this->declare_parameter<std::string>("cloud_topic", "registered_scan");
    this->declare_parameter<double>("half_map_size", 10.0);
    this->declare_parameter<double>("resolution", 0.05);
    this->declare_parameter<double>("heighter_than_ground_threshold", 0.02);
    this->declare_parameter<double>("higher_than_car_threshold", 0.5);
    this->declare_parameter<double>("global_map_size_x", 30.0);
    this->declare_parameter<double>("global_map_size_y", 30.0);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("cloud_topic", cloud_topic);
    this->get_parameter("half_map_size", half_map_size);
    this->get_parameter("resolution", resolution);
    this->get_parameter("heighter_than_ground_threshold", heighter_than_ground_threshold);
    this->get_parameter("higher_than_car_threshold", higher_than_car_threshold);
    this->get_parameter("global_map_size_x", global_map_size_x);
    this->get_parameter("global_map_size_y", global_map_size_y);
    global_map_xi = static_cast<int>(global_map_size_x / resolution);
    global_map_yi = static_cast<int>(global_map_size_y / resolution);
    global_map_ = new unsigned char[static_cast<int>(global_map_xi * global_map_yi)];
    cost_pub_.cost_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", qos);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
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
  double global_map_size_x, global_map_size_y;
  unsigned char * global_map_;
  double *min_hmap, *max_hmap;
  tf2::Transform tf_t;
  int global_map_xi, global_map_yi;
  bool tf_set = false;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  int Map_SIZE_D;
  std::string odom_topic = "lidar_odometry", cloud_topic = "registered_scan";
  double resolution{0.05};
  double heighter_than_ground_threshold{0.02};
  //   double lower_than_car_threshold{0.35};
  double higher_than_car_threshold{0.5};
  geometry_msgs::msg::TransformStamped tf_stamped;
  std::vector<Vec2i> obs_list_;
  std::map<int, int> pub_list;
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