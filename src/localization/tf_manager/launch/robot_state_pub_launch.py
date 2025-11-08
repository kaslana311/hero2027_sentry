import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'tf_manager'
    share_dir = get_package_share_directory(pkg_name)
    
    # URDF 文件路径
    urdf_file_path = os.path.join(share_dir, 'urdf', 'sentry.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # EKF 配置文件路径
    ekf_config_path = os.path.join(share_dir, 'config', 'ekf.yaml')

    # 启动 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 启动 robot_localization EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ekf_node
    ])