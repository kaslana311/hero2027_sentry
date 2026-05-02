
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_box.h>
#include "dog_map/occ_map.hpp"
#include "pcl_ros/transforms.hpp"
const float crop_min_x = -0.8f;
const float crop_min_y = -0.8f;
const float crop_min_z = -10.5f;
const float crop_max_x = 0.8f;
const float crop_max_y = 0.8f;
const float crop_max_z = 10.0f;
using namespace analy_utils;
class DogMapNode : public rclcpp::Node {
   public:
    struct ROSCallback {
        rclcpp::CallbackGroup::SharedPtr  cloud_me_cbk_group,
            update_cbk_group;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
            cloud_sub;
        int unfinished_frame_cnt{0};
        Pose pc_pose;
        pcl::PointCloud<pcl::PointXYZ> pc;
        rclcpp::TimerBase::SharedPtr update_timer;
        std::mutex update_lock;
    } rc_;
    struct CostPub {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_pc_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
            ground_pc_pub;
#ifdef FIX_MAP
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fix_pc_pub;
#endif
    } cost_pub_;
    struct RobotState {
        Vec3f p, v, a, j;
        double yaw;
        double rcv_time;
        bool rcv{false};
        Quatf q;
    } robot_state_;

  

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        sensor_msgs::msg::PointCloud2 out;

        pcl::fromROSMsg(*cloud_msg, *cloud_in_);
        crop_box_filter_.setInputCloud(cloud_in_);
        crop_box_filter_.filter(*cloud_in_);
        Eigen::Matrix4f tf_mat = Eigen::Matrix4f::Identity();
        try {
            tf_stamped = this->tf_buffer_->lookupTransform(
                "map", cloud_msg->header.frame_id, tf2::TimePointZero);
            tf_stamped.transform.translation.z += 0.3;  //添加平移,防止越界
            const auto& tr = tf_stamped.transform.translation;
            const auto& rr = tf_stamped.transform.rotation;

            Eigen::Quaternionf q(
                static_cast<float>(rr.w), static_cast<float>(rr.x),
                static_cast<float>(rr.y), static_cast<float>(rr.z));
            q.normalize();

            tf_mat.setIdentity();
            tf_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
            tf_mat(0, 3) = static_cast<float>(tr.x);
            tf_mat(1, 3) = static_cast<float>(tr.y);
            tf_mat(2, 3) = static_cast<float>(tr.z);
            tf_stamped = this->tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);

        } catch (...) {
            std::cout << " -- [DOG] Cloud callback pcl::fromROSMsg error, "
                         "skip this frame."
                      << std::endl;
            return;
        }
        // tf

        // pcl::PointCloud<pcl::PointXYZ> temp_pc;
        pcl::transformPointCloud(*cloud_in_, *cloud_in_, tf_mat);

        rc_.update_lock.lock();
        rc_.pc = *cloud_in_;
        {
             rc_.pc_pose.first[0] = tf_stamped.transform.translation.x;
            rc_.pc_pose.first[1] = tf_stamped.transform.translation.y;
            rc_.pc_pose.first[2] = tf_stamped.transform.translation.z;
           
        }
        rc_.unfinished_frame_cnt++;
        map_empty_ = false;
        rc_.update_lock.unlock();
    }

    void updateCallback() {
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
            std::cout
                << " -- [DOG WARN] Unfinished frame cnt > 1, the map may not "
                   "work in real-time"
                << std::endl;
        }
            static pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        static Pose temp_pose;
        //从rc_中取出当前帧点云和位姿，进行地图更新
        rc_.update_lock.lock();
        temp_pc->clear();
        temp_pc->swap(rc_.pc);
        temp_pose = rc_.pc_pose;
        rc_.unfinished_frame_cnt = 0;
        rc_.update_lock.unlock();
        occ_map_->Update(temp_pc, temp_pose);
      
        pcl::PointCloud<pcl::PointXYZI> occ_pc;
        occ_map_->PubRes(occ_pc);
        
        if (!occ_pc.empty()) {
            auto msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(occ_pc, msg);

            msg.header.frame_id = "map";
            msg.header.stamp = this->get_clock()->now();
            cost_pub_.cost_pc_pub->publish(std::move(msg));
        }
       
#ifdef FIX_MAP
      static   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_fix=
            std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_in_fix->clear();
        cloud_in_fix->swap(occ_pc);
        pcl::PointCloud<pcl::PointXYZ>::Ptr fix_pc =
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        occ_map_->GetStaticFixMap()->AdjustAndFixByRacying2d(
            temp_pose.first[0], temp_pose.first[1], cloud_in_fix, fix_pc);
        /************* */
        if (!fix_pc->empty()) {
            auto fix_msg = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*fix_pc, fix_msg);
            fix_msg.header.frame_id = "map";
            fix_msg.header.stamp = this->get_clock()->now();
            cost_pub_.fix_pc_pub->publish(fix_msg);
        } else {
            std::cout << " -- [DOG INFO] No fix point to publish." << std::endl;
        }
#endif
        /************* */
        // auto end_time = std::chrono::steady_clock::now();
        // double time_consuming =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
        //                                                           start_time)
        //         .count();
        //  std::cout << " -- [DOG INFO] Map updated in " << time_consuming <<
        //  "ms." << std::endl;
    }

    DogMapNode() : Node("dog_map_node") {
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
        this->get_parameter("heighter_than_ground_threshold",
                            heighter_than_ground_threshold);
        this->get_parameter("higher_than_car_threshold",
                            higher_than_car_threshold);
        this->get_parameter("half_map_size", half_map_size);
        crop_box_filter_.setMin(Eigen::Vector4f(crop_min_x, crop_min_y,
                                                crop_min_z,
                                                1.0f));  // 设置最小点坐标
        crop_box_filter_.setMax(Eigen::Vector4f(crop_max_x, crop_max_y,
                                                crop_max_z,
                                                1.0f));  // 设置最大点坐标
        crop_box_filter_.setNegative(
            true);  // 设置为true表示保留框外的点，去除框内的点
        cloud_in_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        const rclcpp::QoS qos(
            rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
        occ_map_ = new OccMap(heighter_than_ground_threshold,
                              higher_than_car_threshold, half_map_size,
                              resolution_z, frame_save, map_yaml_path);
        cost_pub_.cost_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "rog_map/inf_occ", qos);
        cost_pub_.ground_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "rog_map/ground", qos);
#ifdef FIX_MAP
        cost_pub_.fix_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/fix",
                                                                  qos);
#endif
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        {
                
            rc_.cloud_me_cbk_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions so;
            
           
            so.callback_group = rc_.cloud_me_cbk_group;
            rc_.cloud_sub =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic, qos,
                    std::bind(&DogMapNode::cloudCallback, this,
                              std::placeholders::_1),
                    so);
            rc_.update_cbk_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            rc_.update_timer = this->create_wall_timer(
                std::chrono::milliseconds(4),  // 0.001秒，即1毫秒
                std::bind(&DogMapNode::updateCallback, this),
                rc_.update_cbk_group);
        }
    }
    ~DogMapNode() {
        if (occ_map_) delete occ_map_;
    }

   private:
    OccMap* occ_map_{nullptr};
    bool map_empty_{true};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string odom_topic = "lidar_odometry", cloud_topic = "registered_scan";

    geometry_msgs::msg::TransformStamped tf_stamped;
    pcl::CropBox<pcl::PointXYZ> crop_box_filter_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_;

   private:
    double half_map_size{10.0};
    double heighter_than_ground_threshold{0.05};
    double higher_than_car_threshold{0.5};
    double resolution_z{0.03};
    int frame_save{10};
    std::string map_yaml_path{""};
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DogMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}