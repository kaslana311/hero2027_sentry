"""Transitional experiment: truth navigation pose, Point-LIO registered cloud.

Only cloud source changes. A one-time, explicitly truth-seeded initial transform
places the LIO local world in map; subsequent drift is never corrected by truth.
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sibling(filename):
    path = os.path.join(os.path.dirname(__file__), filename)
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context):
    robot = LaunchConfiguration("namespace").perform(context).strip("/")
    lidar_mode = LaunchConfiguration("lidar_mode").perform(context).lower()
    if not robot or lidar_mode != "dual":
        raise RuntimeError("Point-LIO cloud experiment requires namespace and lidar_mode:=dual")
    truth = _sibling("perception_truth_sim.launch.py")
    shadow = _sibling("point_lio_shadow.launch.py")
    return [
        LogInfo(msg="CLOUD-ONLY experiment: truth navigation pose + Point-LIO DOG-map cloud. "
                    "ONE truth-seeded initial alignment, no continuous truth correction. "
                    "Do not also start the truth/shadow algorithm launches."),
        *truth._launch_setup(context, perception_cloud_topic="pointlio/cloud_registered_map"),
        *shadow._setup(context, publish_cloud_alignment=True, rviz_argument="point_lio_rviz"),
        Node(
            package="hero_sim_bringup", executable="sim_point_lio_cloud_bridge",
            namespace=f"{robot}/pointlio", output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("field", default_value="rmuc_2026"),
        DeclareLaunchArgument("lidar_mode", default_value="dual",
                              description="This snapshot Point-LIO input requires dual lidar"),
        DeclareLaunchArgument("costmap_mode", default_value="combined"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Navigation RViz; dynamic obstacles now use LIO cloud"),
        DeclareLaunchArgument("point_lio_rviz", default_value="false",
                              description="Optional truth/Point-LIO comparison RViz"),
        DeclareLaunchArgument("relay_cmd_vel", default_value="true"),
        DeclareLaunchArgument("publish_debug_visualizations", default_value="false"),
        DeclareLaunchArgument("filter_high_columns", default_value="false"),
        DeclareLaunchArgument("high_column_min_z", default_value="0.30",
                              description="Absolute map Z threshold (metres), not local ground height"),
        OpaqueFunction(function=_setup),
    ])
