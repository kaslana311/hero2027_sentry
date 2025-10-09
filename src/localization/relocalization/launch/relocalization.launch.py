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

  relocalization_node = Node(
    package='relocalization',
    executable='relocalization_node',
    output='screen',
    parameters=[os.path.join(os.path.join(get_package_share_directory('relocalization')),"config","relocalization.yaml")]
  )

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    'rviz_config',
      default_value=os.path.join(os.path.join(get_package_share_directory('relocalization')), 'config', 'relocalization.rviz'),
      description='Full path to the RVIZ config file to use')
  
  rviz_config_file = LaunchConfiguration('rviz_config')
      # Launch rviz
  start_rviz_cmd = Node(
          condition=UnlessCondition('False'),
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config_file],
          output='screen')



  
  return LaunchDescription([relocalization_node,declare_rviz_config_file_cmd,start_rviz_cmd])
  # return LaunchDescription([odom_base_link_node]) 