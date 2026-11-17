"""C2: Point-LIO pose AND cloud feed navigation; truth is diagnostic only.

One map->odom TF holds the initial pose and later global corrections. Both pose
and cloud use that main tree. No ongoing truth correction or truth fallback.
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _sibling(filename):
    path = os.path.join(os.path.dirname(__file__), filename)
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(context):
    source = LaunchConfiguration("data_source", default="sim").perform(context)
    if source == "real":
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("hero_real_bringup"), "launch", "navigation_point_lio_real.launch.py")))]
    if source != "sim":
        raise ValueError("data_source must be sim or real")
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
            context, publish_cloud_alignment=True, rviz_argument="point_lio_rviz", navigation_chain=True),
        Node(package="hero_sim_bringup", executable="sim_point_lio_cloud_bridge",
             namespace=f"{robot}/pointlio", output="screen",
             parameters=[{"use_sim_time": True, "use_navigation_tf": True}],
             remappings=[("/tf", f"/{robot}/tf"), ("/tf_static", f"/{robot}/tf_static")]),
        Node(package="hero_sim_bringup", executable="sim_point_lio_navigation_bridge",
             namespace=robot, output="screen",
             parameters=[{"use_sim_time": True,
                          "relay_cmd_vel": LaunchConfiguration("relay_cmd_vel"),
                          "publish_map_to_odom": ParameterValue(
                              LaunchConfiguration("publish_map_to_odom"), value_type=bool),
                          **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                             for name in ("map_odom_offset_x", "map_odom_offset_y", "map_odom_offset_yaw")}}],
             remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("data_source", default_value="sim",
                              description="sim preserves the parent branch; real selects calibrated hardware input"),
        DeclareLaunchArgument("field", default_value="rmuc_2026"),
        DeclareLaunchArgument("lidar_mode", default_value="dual"),
        DeclareLaunchArgument("costmap_mode", default_value="combined"),
        DeclareLaunchArgument("split_local_refinement", default_value="false",
                              description="Opt-in rmuc_2027 split-only local soft route refinement"),
        DeclareLaunchArgument("stuck_cold_start", default_value="false",
                              description="Opt-in cold MINCO restart after confirmed motion stall"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("point_lio_rviz", default_value="false"),
        DeclareLaunchArgument("relay_cmd_vel", default_value="true"),
        DeclareLaunchArgument("publish_map_to_odom", default_value="true",
                              description="Startup-only TF ownership; false only with an external map->odom publisher"),
        DeclareLaunchArgument("map_odom_offset_x", default_value="0.0"),
        DeclareLaunchArgument("map_odom_offset_y", default_value="0.0"),
        DeclareLaunchArgument("map_odom_offset_yaw", default_value="0.0"),
        DeclareLaunchArgument("publish_debug_visualizations", default_value="false"),
        DeclareLaunchArgument("filter_high_columns", default_value="false"),
        DeclareLaunchArgument("dog_resolution_xy", default_value="0.05",
                              description="DOG XY voxel size; 0 follows PGM; startup-only"),
        DeclareLaunchArgument("high_column_min_z", default_value="0.30",
                              description="Absolute map Z threshold (metres), not local ground height"),
        OpaqueFunction(function=_setup),
    ])
