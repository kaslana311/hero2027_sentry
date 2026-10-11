"""C2: Point-LIO pose AND cloud feed navigation; truth is diagnostic only.

Uses the C1 frozen initial alignment (one truth-seeded starting pose). No ongoing
truth correction or truth fallback. Does not alter planning/control parameters.
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
    if not robot or LaunchConfiguration("lidar_mode").perform(context).lower() != "dual":
        raise RuntimeError("Point-LIO navigation requires namespace and lidar_mode:=dual")
    return [
        LogInfo(msg="C2: Point-LIO navigation pose + cloud. ONE truth-seeded initial alignment, "
                    "then NO truth correction/fallback. Restart the entire algorithm entry "
                    "after resetting the simulator. Do not start another algorithm launch."),
        *_sibling("perception_truth_sim.launch.py")._launch_setup(
            context, perception_cloud_topic="pointlio/cloud_registered_map", point_lio_navigation=True),
        *_sibling("point_lio_shadow.launch.py")._setup(
            context, publish_cloud_alignment=True, rviz_argument="point_lio_rviz"),
        Node(package="hero_sim_bringup", executable="sim_point_lio_cloud_bridge",
             namespace=f"{robot}/pointlio", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="hero_sim_bringup", executable="sim_point_lio_navigation_bridge",
             namespace=robot, output="screen",
             parameters=[{"use_sim_time": True,
                          "relay_cmd_vel": LaunchConfiguration("relay_cmd_vel")}],
             remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("field", default_value="rmuc_2026"),
        DeclareLaunchArgument("lidar_mode", default_value="dual"),
        DeclareLaunchArgument("costmap_mode", default_value="combined"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("point_lio_rviz", default_value="false"),
        DeclareLaunchArgument("relay_cmd_vel", default_value="true"),
        DeclareLaunchArgument("publish_debug_visualizations", default_value="false"),
        DeclareLaunchArgument("filter_high_columns", default_value="false"),
        DeclareLaunchArgument("high_column_min_z", default_value="0.30",
                              description="Absolute map Z threshold (metres), not local ground height"),
        OpaqueFunction(function=_setup),
    ])
