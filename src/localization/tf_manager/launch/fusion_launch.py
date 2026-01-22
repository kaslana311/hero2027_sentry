import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'tf_manager'
    share_dir = get_package_share_directory(pkg_name)
    ekf_config_path = os.path.join(share_dir, 'config', 'ekf.yaml')

    # --- 定义静态变换 ---
    # 这是物理事实，基于您的精确测量
    
    #  base_link -> imu_link (飞控IMU的安装位置)
    #  参数: [x, y, z, yaw, pitch, roll]
    # static_tf_imu_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_imu',
    #     arguments=['0.08', '-0.02', '-0.165', '0', '0', '0', 'base_link', 'chassis_link']
    # )

    #  base_link -> lidar_link (激光雷达的安装位置)
    static_tf_joint_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0.2', '0', '0', '0', '0.6108652', '0', 'base_link', 'lidar_link']
    )

    # static_tf_lidar_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_lidar',
    #     arguments=['0.15', '0', '0', '0', '0', '0', 'base_link', 'joint_link']
    # )

    # --- 启动 robot_localization EKF 节点 ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    return LaunchDescription([
        # static_tf_imu_node,
        # static_tf_lidar_node,
        static_tf_joint_node,
        ekf_node
    ])