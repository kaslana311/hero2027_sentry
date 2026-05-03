#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomToTfNode : public rclcpp::Node {
   public:
    OdomToTfNode() : Node("odom_to_tf_node"), count_(0) {
        // 声明参数
        this->declare_parameter("odom_topic", "/odin1/odometry_highfreq");
        this->declare_parameter("downsample_rate", 4);
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("child_frame_id", "base_link");

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        downsample_rate_ = this->get_parameter("downsample_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 使用 Best Effort QoS 订阅高频话题，防止堵塞
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos,
            std::bind(&OdomToTfNode::odom_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "OdomToTfNode started. Subscribing to '%s' (Best Effort) "
                    "and publishing TF '%s' -> '%s' at 1/%d rate.",
                    odom_topic.c_str(), frame_id_.c_str(),
                    child_frame_id_.c_str(), downsample_rate_);
    }

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 累加计算，降采样
        if (count_++ % downsample_rate_ == 0) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = msg->header.stamp;
            // 覆盖或使用原始frame_id
            tf_msg.header.frame_id =
                frame_id_.empty() ? msg->header.frame_id : frame_id_;
            tf_msg.child_frame_id =
                child_frame_id_.empty() ? msg->child_frame_id : child_frame_id_;

            tf_msg.transform.translation.x = msg->pose.pose.position.x;
            tf_msg.transform.translation.y = msg->pose.pose.position.y;
            tf_msg.transform.translation.z = msg->pose.pose.position.z;
            tf_msg.transform.rotation = msg->pose.pose.orientation;

            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    int count_;
    int downsample_rate_;
    std::string frame_id_;
    std::string child_frame_id_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTfNode>());
    rclcpp::shutdown();
    return 0;
}
