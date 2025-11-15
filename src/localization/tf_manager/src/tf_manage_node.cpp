#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class TfManager : public rclcpp::Node {
   public:
    TfManager() : Node("tf_manage_node") {
        // 参数配置
        declare_parameter("odom_frame_id", "odom");
        declare_parameter("lidar_frame_id", "livox_frame");
        declare_parameter("base_frame_id", "base_link");
        declare_parameter("publish_frequency", 100.0);

        // 新增：静态TF参数（雷达到底盘的固定偏移）
        declare_parameter("base_to_lidar.translation.xyz",
                          std::vector<double>{0.0, 0.0, 0.0});  // xyz
        declare_parameter(
            "base_to_lidar.rotation.rpy",
            std::vector<double>{0.0, 0.0, 0.0});  // 旋转角度（弧度）

        // 初始化三个独立的TF广播器
        odom_base_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(this);
        map_fake_base_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(
                this);  // 新增fake_base_link广播器
        static_tf_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // 创建QoS配置（根据实际需求调整）
        PublishStaticBaseToLidarTransform();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

        // 订阅高频odom->base_link变换
        // odom_base_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/Odometry", qos, [this](const nav_msgs::msg::Odometry::SharedPtr
        //     msg) {
        //       HandleOdomBase(msg);
        //     });

        // 初始化TF2缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock(), tf2::Duration(std::chrono::seconds(5)));
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建Odometry发布器
        base_in_map_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/base_link_in_map_odom", 1);

        // 设置定时器（100Hz）
        base_in_map_odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TfManager::PubBaseInMapcbk, this));

        // 设置定时器（100Hz）用于发布map到fake_base_link的变换
        map_fake_base_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TfManager::PublishMapToFakeBaseTransform, this));

        RCLCPP_INFO(this->get_logger(), "tf manage node has launched!");
    }

   private:
    void PublishStaticBaseToLidarTransform() {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = now();
        static_transform.header.frame_id =
            get_parameter("lidar_frame_id")
                .as_string();  // 父坐标系：livox_frame
        static_transform.child_frame_id =
            get_parameter("base_frame_id").as_string();  // 子坐标系：base_link

        // 从参数读取平移量
        auto xyz =
            get_parameter("base_to_lidar.translation.xyz").as_double_array();

        static_transform.transform.translation.x = xyz[0];
        static_transform.transform.translation.y = xyz[1];
        static_transform.transform.translation.z = xyz[2];

        // 从参数读取RPY角度并转换为四元数
        // auto rpy =
        // get_parameter("base_to_lidar.rotation.rpy").as_double_array();
        // tf2::Quaternion q;
        // q.setRPY(rpy[0], rpy[1], rpy[2]);
        // static_transform.transform.rotation = tf2::toMsg(q);
        // 从参数读取旋转角度
        auto rpy =
            get_parameter("base_to_lidar.rotation.rpy").as_double_array();
        // 先绕Y轴旋转(pitch)，再绕新的X轴旋转(roll)
        tf2::Quaternion q_roll, q_pitch, q_yaw, q_final;
        // 第二步：绕世界坐标系的X轴旋转roll角度
        q_roll.setRPY(rpy[0], 0, 0);
        // 第一步：绕世界坐标系的Y轴旋转pitch角度
        q_pitch.setRPY(0, rpy[1], 0);
        // 第一步：绕世界坐标系的Y轴旋转pitch角度
        q_yaw.setRPY(0, 0, rpy[2]);
        // 组合旋转：先pitch后roll（注意四元数乘法的顺序）
        // 这相当于：先绕Y轴旋转，然后绕旋转后的X轴旋转
        q_final = q_roll * q_pitch * q_yaw;

        static_transform.transform.rotation = tf2::toMsg(q_final);

        // 发布静态TF
        static_tf_broadcaster_->sendTransform(static_transform);
        RCLCPP_INFO(this->get_logger(), "static tf pub!");
    }

    // void HandleMapOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //     geometry_msgs::msg::TransformStamped transform;

    //     // 填充头信息
    //     transform.header.stamp = msg->header.stamp;  // 使用消息自带时间戳
    //     transform.header.frame_id = "map";
    //     transform.child_frame_id = "odom";

    //     // 位置数据
    //     transform.transform.translation.x = msg->pose.pose.position.x;
    //     transform.transform.translation.y = msg->pose.pose.position.y;
    //     transform.transform.translation.z = msg->pose.pose.position.z;

    //     // 方向数据（自动归一化）
    //     tf2::Quaternion q(
    //         msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    //         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //     q.normalize();

    //     transform.transform.rotation.x = q.x();
    //     transform.transform.rotation.y = q.y();
    //     transform.transform.rotation.z = q.z();
    //     transform.transform.rotation.w = q.w();

    //     // 发布变换
    //     map_odom_broadcaster_->sendTransform(transform);
    //     // RCLCPP_INFO(this->get_logger(), "static tf pub!");
    // }

    void HandleOdomBase(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;

        // 使用当前最新时间戳
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "livox_frame";

        // 位置数据
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // 方向数据（自动归一化）
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        q.normalize();

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // 发布变换
        odom_base_broadcaster_->sendTransform(transform);

        RCLCPP_INFO(this->get_logger(), "odom and base_link tf pub!");
    }

    void PubBaseInMapcbk() {
        try {
            // 获取最新的坐标变换
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(
                    "map",                // 目标坐标系
                    "base_link",          // 源坐标系
                    tf2::TimePointZero);  // 获取最新可用变换

            // 创建Odometry消息
            auto odom_msg = nav_msgs::msg::Odometry();

            // 设置消息头
            odom_msg.header.stamp = this->get_clock()->now();
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_link";

            // 填充位姿信息
            odom_msg.pose.pose.position.x = transform.transform.translation.x;
            odom_msg.pose.pose.position.y = transform.transform.translation.y;
            odom_msg.pose.pose.position.z = transform.transform.translation.z;
            odom_msg.pose.pose.orientation = transform.transform.rotation;

            // 发布消息
            base_in_map_odom_pub_->publish(odom_msg);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
        }
    }

    void PublishMapToFakeBaseTransform() {
        try {
            // 获取map到base_link的变换
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("map", "base_link",
                                            tf2::TimePointZero);

            // 创建新的变换，保持位置不变，但将姿态置零
            geometry_msgs::msg::TransformStamped fake_transform;
            fake_transform.header.stamp = this->now();
            fake_transform.header.frame_id = "map";
            fake_transform.child_frame_id = "fake_base_link";

            // 保持位置不变
            fake_transform.transform.translation =
                transform.transform.translation;

            // 将姿态置零（使用单位四元数）
            fake_transform.transform.rotation.x = 0.0;
            fake_transform.transform.rotation.y = 0.0;
            fake_transform.transform.rotation.z = 0.0;
            fake_transform.transform.rotation.w = 1.0;

            // 发布变换
            map_fake_base_broadcaster_->sendTransform(fake_transform);

            RCLCPP_INFO(this->get_logger(),
                        "Published map to fake_base_link transform");

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                        ex.what());
        }
    }

    // 成员变量
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_base_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>
        map_fake_base_broadcaster_;  // 新增
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_base_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_in_map_odom_pub_;
    rclcpp::TimerBase::SharedPtr base_in_map_odom_timer_;
    rclcpp::TimerBase::SharedPtr map_fake_base_timer_;  // 新增

    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}