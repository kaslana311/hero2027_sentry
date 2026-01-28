
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "dog_map/occ_map.hpp"
#include "pcl_ros/transforms.hpp"

using namespace analy_utils;
class DogMapNode : public rclcpp::Node
{
public:
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pc_pub;
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

    auto pose = std::make_pair(
      Vec3f(
        odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z),
      Quatf(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z));
    robot_state_.p = pose.first;
    robot_state_.q = pose.second;
    robot_state_.rcv_time = this->get_clock()->now().seconds();
    robot_state_.rcv = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    sensor_msgs::msg::PointCloud2 out;

    try {
      tf_stamped =
        this->tf_buffer_->lookupTransform("map", cloud_msg->header.frame_id, tf2::TimePointZero);
      tf_t = tf2::Transform(
        tf2::Quaternion(
          tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y,
          tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w),
        tf2::Vector3(
          tf_stamped.transform.translation.x, tf_stamped.transform.translation.y,
          tf_stamped.transform.translation.z+0.28));
      if (!tf_set) tf_set = true;
    } catch (...) {
      std::cout << " -- [DOG] Cloud callback pcl::fromROSMsg error, "
                   "skip this frame."
                << std::endl;
      return;
    }
    // tf
    pcl_ros::transformPointCloud("map", tf_t, *cloud_msg, out);
    if (!robot_state_.rcv) {
      std::cout << " -- [DOG] No odom received, skip cloud callback." << std::endl;
      return;
    }
    double cbk_t = this->get_clock()->now().seconds();
    if (cbk_t - robot_state_.rcv_time > odom_timeout) {
      std::cout << " -- [DOG] Odom timeout, skip cloud callback." << std::endl;
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> temp_pc;
    pcl::fromROSMsg(out, temp_pc);
    rc_.update_lock.lock();
    rc_.pc = temp_pc;
    {
      geometry_msgs::msg::Pose ps_in;
      ps_in.position.x = robot_state_.p.x();
      ps_in.position.y = robot_state_.p.y();
      ps_in.position.z = robot_state_.p.z();
      ps_in.orientation.w = robot_state_.q.w();
      ps_in.orientation.x = robot_state_.q.x();
      ps_in.orientation.y = robot_state_.q.y();
      ps_in.orientation.z = robot_state_.q.z();
      // std::cout << " -- [ROS] Cloud callback robot pose before tf inv: " << ps_in.position.x << ", "
      //           << ps_in.position.y << ", " << ps_in.position.z << std::endl;
      geometry_msgs::msg::Pose ps_out;

      try {
        tf2::doTransform(ps_in, ps_out, tf_stamped);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "tf2 doTransform failed: %s", e.what());
      }
      // std::cout << " -- [ROS] Cloud callback robot pose after tf inv: " << ps_out.position.x << ", "
      //           << ps_out.position.y << ", " << ps_out.position.z << std::endl;
      robot_state_.p.x() = ps_out.position.x;
      robot_state_.p.y() = ps_out.position.y;
      robot_state_.p.z() = ps_out.position.z;
      robot_state_.q = Quatf(
        ps_out.orientation.w, ps_out.orientation.x, ps_out.orientation.y, ps_out.orientation.z);
    }
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
        std::cout << " -- [DOG WARN] No point cloud input, check the "
                     "topic name."
                  << std::endl;
        last_print_t = cur_t;
      }
      return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
      return;
    }

    if (rc_.unfinished_frame_cnt > 1) {
      std::cout << " -- [DOG WARN] Unfinished frame cnt > 1, the map may not "
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
    occ_map_->Update(temp_pc.makeShared(), temp_pose);
    pcl::PointCloud<pcl::PointXYZI> occ_pc;
    occ_map_->PubRes(occ_pc);
    if (!occ_pc.empty()) {
      auto msg = sensor_msgs::msg::PointCloud2();
      pcl::toROSMsg(occ_pc, msg);

      msg.header.frame_id = "map";
      msg.header.stamp = this->get_clock()->now();
      cost_pub_.cost_pc_pub->publish(msg);
    }
    /************* */
    auto end_time = std::chrono::steady_clock::now();
    double time_consuming =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << " -- [DOG INFO] Map updated in " << time_consuming << " ms." << std::endl;
  }

  DogMapNode() : Node("dog_map_node")
  {
    this->declare_parameter<std::string>("odom_topic", odom_topic);
    this->declare_parameter<std::string>("cloud_topic", cloud_topic);
    this->declare_parameter<std::string>("map_yaml_path", "");
    this->declare_parameter<double>("heighter_than_ground_threshold", 0.05);
    this->declare_parameter<double>("higher_than_car_threshold", 0.5);
    this->declare_parameter<int>("frame_save", 10);
    this->declare_parameter<double>("half_map_size", 10.0);
    this->declare_parameter<double>("resolution_z", 0.03);
    this->get_parameter("frame_save", frame_save);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("resolution_z", resolution_z);
    this->get_parameter("cloud_topic", cloud_topic);
    this->get_parameter("map_yaml_path", map_yaml_path);
    this->get_parameter("heighter_than_ground_threshold", heighter_than_ground_threshold);
    this->get_parameter("higher_than_car_threshold", higher_than_car_threshold);
    this->get_parameter("half_map_size", half_map_size);

    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    occ_map_ = new OccMap(
      heighter_than_ground_threshold, higher_than_car_threshold, half_map_size, resolution_z,
      frame_save, map_yaml_path);
    cost_pub_.cost_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", qos);
    cost_pub_.ground_pc_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/ground", qos);
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
        odom_topic, qos, std::bind(&DogMapNode::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      rc_.cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, qos, std::bind(&DogMapNode::cloudCallback, this, std::placeholders::_1), so);
      rc_.update_cbk_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = this->create_wall_timer(
        std::chrono::milliseconds(1),  // 0.001秒，即1毫秒
        std::bind(&DogMapNode::updateCallback, this), rc_.update_cbk_group);
    }
  }
  ~DogMapNode()
  {
    if (occ_map_) delete occ_map_;
  }

private:
  OccMap * occ_map_{nullptr};
  bool map_empty_{true};
  double odom_timeout{0.1};
  tf2::Transform tf_t;
  bool tf_set = false;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string odom_topic = "lidar_odometry", cloud_topic = "registered_scan";

  geometry_msgs::msg::TransformStamped tf_stamped;

private:
  double half_map_size{10.0};
  double heighter_than_ground_threshold{0.05};
  double higher_than_car_threshold{0.5};
  double resolution_z{0.03};
  int frame_save{10};
  std::string map_yaml_path{""};
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DogMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}