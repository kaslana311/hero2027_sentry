import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
  

  tf_manage_node = Node(
    package='tf_manager',
    executable='tf_manage_node',
    output='screen',
    parameters=[os.path.join(os.path.join(get_package_share_directory('tf_manager')),"config","tf_manage_node.yaml")]
  )

  return LaunchDescription([tf_manage_node])