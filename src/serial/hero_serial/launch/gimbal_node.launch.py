import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
  
  node = Node(
    package='hero_serial',
    executable='gimbal_node',
    output='screen'
  )
  
  return LaunchDescription([node])