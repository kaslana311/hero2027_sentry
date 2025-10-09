
#include "spin_control/spin_control.hpp"
// static double spin_control::get_yaw(){return point_base_link.yaw;}
double spin_control::distance(double x1,double y1,double x2,double y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
double clamp(double value,double min,double max)
{
    if (value<min) return min;
    if (value>max) return max;
    return value;
}
void spin_control::init()
{
    // this->declare_parameter<double>("Kp",0.0001);
    // this->declare_parameter<double>("Ki",0.2);
    // this->declare_parameter<double>("Kd",0.3);
    // this->declare_parameter<double>("expect_vel",0.5);
    // this->declare_parameter<double>("actual_vel",0.0);
    // this->get_parameter("Kp",pid.Kp);
    // this->get_parameter("Ki",pid.Ki);
    // this->get_parameter("Kd",pid.Kd);
    // this->get_parameter("expect_vel",pid.expect_vel);
    // this->get_parameter("actual_vel",pid.actual_vel);
    this->declare_parameter<double>("speed",1.2);
    this->declare_parameter<double>("min_speed",0.5);
    this->declare_parameter<double>("tolerance",0.2);
    this->declare_parameter<double>("lookahead_dist",0.5);
    this->declare_parameter<double>("lookahead_time",1.5);
    this->declare_parameter<double>("min_lookahead_dist",0.15);
    this->declare_parameter<double>("max_lookahead_dist",0.2);
    this->declare_parameter<bool>("use_velocity_scaled_lookahead_dist",false);
    this->declare_parameter<double>("fit_count",10);
    this->declare_parameter<double>("curvature_k",0.5);


    this->get_parameter("speed", speed_);
    this->get_parameter("min_speed", min_speed_);
    this->get_parameter("tolerance", tolerance_);
    this->get_parameter("lookahead_dist", lookahead_dist_);
    this->get_parameter("lookahead_time", lookahead_time_);
    this->get_parameter("min_lookahead_dist", min_lookahead_dist_);
    this->get_parameter("max_lookahead_dist", max_lookahead_dist_);
    this->get_parameter("speed", speed_);
    this->get_parameter("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
    this->get_parameter("fit_count",fit_count_);
    this->get_parameter("curvature_k",curvature_k_);



}
void spin_control::base_link_callback()
{
    try
    {
    //获取base_link坐标
    geometry_msgs::msg::TransformStamped msg;
    msg=tf_buffer->lookupTransform("map","base_link",tf2::TimePointZero);
    // transform_pub->publish(msg);
    base_link_map.x=msg.transform.translation.x;
    base_link_map.y=msg.transform.translation.y;
    base_link_map.z=msg.transform.translation.z;
    geometry_msgs::msg::PointStamped base_link_point;
    base_link_point.header.frame_id = "map";
    auto clock = std::make_shared<rclcpp::Clock>();
    base_link_point.header.stamp = this->now(); 
    // pursuit_point.header=pursuit_pose.header;
    base_link_point.point.x=msg.transform.translation.x;
    base_link_point.point.y=msg.transform.translation.y;
    base_link_point.point.z=msg.transform.translation.z;
    transform_pub->publish(base_link_point);
    // 使用四元数转换为欧拉角
    tf2::Quaternion q(msg.transform.rotation.x,
                        msg.transform.rotation.y,
                        msg.transform.rotation.z,
                        msg.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    shaobing.robot_x=base_link_map.x;
    shaobing.robot_y=base_link_map.y;
    shaobing.robot_yaw=yaw;

    // RCLCPP_INFO(this->get_logger(),"base_link:\nx:%f,y:%f,z:%f,roll:%f,pitch:%f,yaw:%f",base_link_map.x,base_link_map.y,base_link_map.z,base_link_map.roll,base_link_map.pitch,base_link_map.yaw);
    //消息类型转换
    tf2::Transform transform;
    tf2::fromMsg(msg.transform, transform);
    
    //计算map相对于base的平移矩阵,求逆变换
    tf2::Transform map_to_base_link=transform.inverse();

    //给雷达用的
    base_link_to_map_translation=transform.getOrigin();
    base_link_to_map_rotation.setRotation(transform.getRotation());
    // RCLCPP_INFO(this->get_logger(),"base_link_to_map_translation.x=%.2f",base_link_to_map_translation.x());
    // RCLCPP_INFO(this->get_logger(),"base_link_to_map_translation.y=%.2f",base_link_to_map_translation.y());

    tf2::Vector3 every_car_point(1.0,1.0,0);
    // 计算车在导航地图下坐标
    car_in_base_position=base_link_to_map_rotation*every_car_point+base_link_to_map_translation;
    pose_tf p;
    p.x = car_in_base_position.getX();
    p.y = car_in_base_position.getY();
    // RCLCPP_INFO(this->get_logger(),"p.x=%.2f",p.x);
    // RCLCPP_INFO(this->get_logger(),"p.y=%.2f",p.y);

    geometry_msgs::msg::PointStamped car_pursuit_point;
    car_pursuit_point.header.frame_id = "map";
    // auto clock = std::make_shared<rclcpp::Clock>();
    car_pursuit_point.header.stamp = this->now(); 
    // pursuit_point.header=pursuit_pose.header;
    car_pursuit_point.point.x=p.x;
    car_pursuit_point.point.y=p.y;
    car_pursuit_point.point.z=p.z;
    // pursuit_point_pub->publish(car_pursuit_point);


    //获取平移矩阵和旋转矩阵
    map_to_base_link_translation =map_to_base_link.getOrigin();
    tf2::Quaternion rotation = map_to_base_link.getRotation();
    map_to_base_link_rotation.setRotation(rotation);

    }
    catch(tf2::TransformException & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform vehicle pose: %s", e.what());
    }

}
//获取全局路径,对每个路径点做新坐标系的变换
void spin_control::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    path_=msg;
    point_i=0;

}
double spin_control::optimize_speed(const double curvature)
{
    //弦长
    // double chord=pow(distance(path_in_base_position.getX(),path_in_base_position.getY(),base_link_map.x,base_link_map.y),2);
    // //横向误差 追踪路径点在地图中的横坐标减base_link在map中的横坐标
    // double lateral_err=6;
    // //曲率
    // double curvature=2*lateral_err/chord;
    // double radius=1/curvature;
    // //根据曲率来约束速度
    // RCLCPP_INFO(rclcpp::get_logger("spin_control"),"curvature=%f",curvature);
    double speed = speed_;
    speed = speed/(1+curvature*curvature_k_);
    speed = std::max(speed,min_speed_);
    return speed;
    

}
void spin_control::calculate_cmd_vel()
{
    if (!path_)
    {
        // RCLCPP_WARN(this->get_logger(), "No path received yet.");
        return;
    }
    // RCLCPP_WARN(this->get_logger(), "path size :%d",path_->poses.size());

    geometry_msgs::msg::PointStamped pursuit_point;
    const double lookahead_dist =getLookAheadDistance(speed_);
    // RCLCPP_INFO(this->get_logger(),"LOOKAHEAD_DISTANCE : %f",lookahead_dist);
    auto pursuit_pose =getLookAheadPoint(lookahead_dist,path_);
    // RCLCPP_INFO(this->get_logger(), "x: %f,y: %f",pursuit_pose.pose.position.x,pursuit_pose.pose.position.y);
    pursuit_point.header.frame_id = "map";
    auto clock = std::make_shared<rclcpp::Clock>();
    pursuit_point.header.stamp = this->now(); 
    // pursuit_point.header=pursuit_pose.header;
    pursuit_point.point.x=pursuit_pose.pose.position.x;
    pursuit_point.point.y=pursuit_pose.pose.position.y;
    pursuit_point.point.z=pursuit_pose.pose.position.z;
    pursuit_point_pub->publish(pursuit_point);
    tf2::Vector3 every_point(pursuit_pose.pose.position.x,pursuit_pose.pose.position.y,pursuit_pose.pose.position.z);
    // 计算路径点在base_link下的坐标
    path_in_base_position=map_to_base_link_rotation*every_point+map_to_base_link_translation;
    //计算雷达与路径点的夹角
    point_base_link.yaw=atan2(path_in_base_position.getY(),path_in_base_position.getX());
    double yaw=point_base_link.yaw/M_PI*180;

    // RCLCPP_INFO(this->get_logger(), "Yaw: %f",point_base_link.yaw/M_PI*180);
    double dis=distance(base_link_map.x,base_link_map.y,path_->poses[path_->poses.size()-1].pose.position.x, path_->poses[path_->poses.size()-1].pose.position.y);
    // RCLCPP_WARN(this->get_logger(), "dis:%f",dis);
    double kappa = fit_circle(circle_points);
    double optimized_speed =optimize_speed(kappa);
    
    // pursuit_pose==path_->poses[path_->poses.size()-1]&&
    if(dis<tolerance_)
    {
        shaobing.is_arrived=true;
        RCLCPP_INFO(this->get_logger(), "到达目标点");
        cmd_vel.linear.x=0;
        cmd_vel.linear.y=0;
    }
    else
    {
        shaobing.is_arrived=false;
        cmd_vel.linear.x=optimized_speed*cos(point_base_link.yaw);
        cmd_vel.linear.y=optimized_speed*sin(point_base_link.yaw);
    }
    cmd_vel.linear.z=atan2(path_in_base_position.getX(),path_in_base_position.getY());
    // RCLCPP_INFO(this->get_logger(), "expected yaw = %f",cmd_vel.linear.z);
    //0-90 90-x
    //90-180 90-x
    //-180~-90 270-x
    //-90~0 90-x
    if(yaw>-180&&yaw<-90)
    {shaobing.expect_spin_degree=-270-yaw;}
    else
    {shaobing.expect_spin_degree=90-yaw;}

    cmd_pub->publish(cmd_vel);
    robot_pub->publish(shaobing);

}

double spin_control::getLookAheadDistance(double speed)
{
    // 如果使用基于速度缩放的前瞻距离，则计算并限制距离值
    // 否则，使用静态的前瞻距离
    double lookahead_distance = lookahead_dist_;
    if (use_velocity_scaled_lookahead_dist_)
    {
        lookahead_distance = speed*lookahead_time_;
        lookahead_distance = clamp(lookahead_distance,min_lookahead_dist_,max_lookahead_dist_);
    }
    return lookahead_distance;
    
}
geometry_msgs::msg::PoseStamped spin_control::getLookAheadPoint(const double & lookahead_dist,
const nav_msgs::msg::Path::SharedPtr & path)
{
    // 找到第一个距离大于前瞻距离的姿态
    for(;point_i<path->poses.size();point_i++)
    {
        if(distance(path->poses[point_i].pose.position.x,path->poses[point_i].pose.position.y,base_link_map.x,base_link_map.y)>= lookahead_dist)
        {
        first_pose=path->poses[point_i];
        for(int i = 0;i<=point_i+fit_count_;i++)
        {
            circle_points.push_back(path->poses[i]);
        }

        break;

        }
    }
    // RCLCPP_WARN(this->get_logger(), "point_i:%d",point_i);

    // RCLCPP_INFO(this->get_logger(), "first_posex: %f,first_posey: %f",path->poses[point_i].pose.position.x,path->poses[point_i].pose.position.y);

    // auto first_pose = std::find_if(
    //     path->poses[i],path->poses.end(),[&](const auto & p){
    //         return distance(p.pose.position.x,p.pose.position.y,base_link_map.x,base_link_map.y)>= lookahead_dist;
    //     });

    // 如果没有足够远的姿态，则取最后一个姿态
    // if (first_pose == path->poses.end())
    // {
    //     first_pose = std::prev(path->poses.end());
    //     RCLCPP_WARN(this->get_logger(), "No pose found within lookahead distance.");
    // }
    return first_pose;
    
}

double spin_control::fit_circle(std::vector<geometry_msgs::msg::PoseStamped> &points)
{
    int n =points.size();
    double sum_x,sum_y,sum_x2,sum_y2,sum_x3,sum_y3,sum_xy,sum_x2y,sum_xy2 = 0;
    for(const auto &point : points)
    {
        double x = point.pose.position.x;
        double y = point.pose.position.y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x * x;
        sum_y2 += y * y;
        sum_x3 += x * x * x;
        sum_y3 += y * y * y;
        sum_xy += x * y;
        sum_x2y += x * x * y;
        sum_xy2 += x * y * y;

    }
    double C = n * sum_x2 - sum_x * sum_x;
    double D = n * sum_xy - sum_x * sum_y;
    double E = n * sum_x3 + n * sum_xy2 - sum_x * (sum_x2 + sum_y2);
    double G = n * sum_y2 + sum_y * sum_y;
    double H = n * sum_x2y + n * sum_y3 - sum_y*(sum_x2 + sum_y2);
    double a = (H*D - E*G)/(C*G  - D*D);
    double b = (H*C - E*D)/(D*D - G*C);
    double c = -(sum_x2 + sum_y2 + a * sum_x + b* sum_y)/n;
    double center_x = a / -2;
    double center_y = b / -2;
    double kappa = 2/sqrt(a*a + b*b - 4*c);
    double radius = 1 / kappa;
    points.clear();


    RCLCPP_INFO(this->get_logger(),"kappa = %f",kappa);
    // RCLCPP_INFO(this->get_logger(),"n = %d",n);

    // visualization_msgs::msg::Marker marker;
    // marker.header.frame_id = "map";  // Replace with your frame
    // marker.header.stamp = this->get_clock()->now();
    // marker.ns = "circle";
    // marker.id = 0;
    // marker.type = visualization_msgs::msg::Marker::CYLINDER;
    // marker.action = visualization_msgs::msg::Marker::ADD;

    // // Set the position and scale of the marker
    // marker.pose.position.x = center_x;
    // marker.pose.position.y = center_y;
    // marker.pose.position.z = 0.0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;

    // marker.scale.x = radius * 2.0;  // Diameter of the circle
    // marker.scale.y = radius * 2.0;  // Diameter of the circle
    // marker.scale.z = 0.01;  // Thickness of the cylinder

    // marker.color.a = 1.0;  // Alpha
    // marker.color.r = 1.0;  // Red
    // marker.color.g = 0.0;  // Green
    // marker.color.b = 0.0;  // Blue

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // 使用你的坐标系
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "circle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 修改类型为LINE_STRIP
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.05;  // 线条宽度

    marker.color.a = 1.0;  // 透明度
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;  // 绿色
    marker.color.b = 0.0;  // 蓝色

    // 创建圆的点
    int num_points = 100;
    for (int i = 0; i <= num_points; ++i) {
        double angle = 2 * M_PI * i / num_points;
        geometry_msgs::msg::Point p;
        p.x = center_x + radius * cos(angle);
        p.y = center_y + radius * sin(angle);
        p.z = 0.0;
        marker.points.push_back(p);
    }



    circle_pub->publish(marker);


    return kappa;
    
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<spin_control>();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        // node->base_link_callback();
    }
    
    rclcpp::shutdown();
    return 0;
}