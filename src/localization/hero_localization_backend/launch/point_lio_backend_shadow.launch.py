"""Sidecar only: subscribe to an existing Point-LIO, never start a second one."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from hero_localization_backend.configuration import parameters, debugger_prefix


def setup(context):
    robot = LaunchConfiguration('namespace').perform(context).strip('/')
    share = get_package_share_directory('hero_localization_backend')
    with open(os.path.join(get_package_share_directory('backend'), 'config', 'backend.yaml')) as f:
        base = yaml.safe_load(f)
    with open(os.path.join(share, 'config', 'backend_point_lio.yaml')) as f:
        overrides = yaml.safe_load(f)
    config = parameters(base, overrides, robot, LaunchConfiguration('prior_map').perform(context))
    namespace = f'{robot}/z_backend'
    debug = LaunchConfiguration('backend_debug').perform(context)
    return [
        LogInfo(msg='BACKEND SHADOW ONLY: Point-LIO raw odometry/cloud -> prior-map factor graph. '
                    'NO cmd_vel, NO navigation pose replacement, private TF. UWB disabled. '
                    'Start at the normal 2027 spawn before driving; full frontend-reset integration is NOT implemented.'),
        Node(package='backend', executable='backend_node', name='backend', namespace=namespace,
             prefix=debugger_prefix(debug),
             output='both' if debug == 'true' else 'screen',
             parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        Node(package='hero_localization_backend', executable='backend_shadow_monitor.py',
             name='comparison', namespace=namespace, output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'truth_topic': f'/{robot}/odometry/truth_3d'}]),
        Node(package='rviz2', executable='rviz2', name='backend_shadow_rviz', namespace=namespace,
             condition=IfCondition(LaunchConfiguration('backend_rviz')), output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             arguments=['-d', os.path.join(share, 'rviz', 'backend_shadow.rviz')],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                         ('comparison/truth_path', f'/{robot}/pointlio/evaluation/truth_path'),
                         ('comparison/frontend_path', f'/{robot}/pointlio/evaluation/pointlio_path')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='red_standard_robot1'),
        DeclareLaunchArgument('prior_map', description='Absolute path to generated 3-D map bundle, NOT a PGM'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('backend_rviz', default_value='true'),
        DeclareLaunchArgument('backend_debug', default_value='false',
                             description='Run only the shadow backend under gdb; fatal stacks enter launch.log'),
        OpaqueFunction(function=setup),
    ])
