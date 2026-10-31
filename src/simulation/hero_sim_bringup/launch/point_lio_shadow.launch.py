"""Point-LIO shadow only: raw simulator sensors in, isolated estimates out.

Estimator inputs never use truth; a separate display observer compares truth.
Does not start navigation, load a prior PCD or relay motors.
Run alongside the unchanged perception_truth_sim.launch.py for comparisons.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, publish_cloud_alignment=False, rviz_argument="use_rviz", navigation_chain=False):
    robot = LaunchConfiguration("namespace").perform(context).strip("/")
    if not robot:
        raise RuntimeError("namespace must identify the simulated dual-lidar robot")
    share = get_package_share_directory("hero_sim_bringup")
    namespace = f"{robot}/pointlio"
    # C0/C1 keep private shadow TF. C2 uses ONE navigation tree and suppresses
    # the frontend's competing odom->IMU TF; its rigid adapter supplies that chain.
    tf_remaps = ([("/tf", f"/{robot}/tf"), ("/tf_static", f"/{robot}/tf_static")]
                 if navigation_chain else [("/tf", "tf"), ("/tf_static", "tf_static")])
    frontend_overrides = ({"odometry.odom_frame_id": "odom", "publish.tf_send_en": False}
                          if navigation_chain else {})
    return [
        LogInfo(msg=f"Point-LIO estimator at /{namespace}; " +
                ("unified navigation TF" if navigation_chain else "private shadow TF")),
        Node(
            package="hero_sim_bringup", executable="sim_point_lio_input_bridge",
            namespace=namespace, output="screen",
            parameters=[{
                "use_sim_time": True,
                "left_topic": f"/{robot}/livox/lidar_left",
                "right_topic": f"/{robot}/livox/lidar_right",
                "imu_topic": f"/{robot}/livox/imu_right",
            }],
        ),
        Node(
            package="point_lio", executable="pointlio_mapping",
            namespace=namespace, output="screen",
            parameters=[os.path.join(share, "config", "point_lio_snapshot.yaml"), frontend_overrides],
            remappings=tf_remaps + [("/odometry/filtered", "odometry")],
        ),
        Node(
            package="hero_sim_bringup", executable="sim_point_lio_evaluation",
            namespace=namespace, output="screen", remappings=tf_remaps,
            parameters=[{
                "use_sim_time": True,
                "truth_topic": f"/{robot}/odometry/truth_3d",
                "publish_cloud_alignment": publish_cloud_alignment,
                "use_navigation_tf": navigation_chain,
                "odom_frame": "odom" if navigation_chain else "pointlio_odom",
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration(rviz_argument)),
            package="rviz2", executable="rviz2", name="pointlio_rviz",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": True}],
            # Display only: subscribe to the existing navigation map, not another map_server.
            remappings=tf_remaps + [("evaluation/static_map", f"/{robot}/map")],
            arguments=["-d", os.path.join(share, "rviz", "point_lio_shadow.rviz")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="red_standard_robot1"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
