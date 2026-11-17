"""C3 shadow: keep C2 navigation intact and observe an isolated localization backend."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

DEFAULTS = {
    'namespace': 'red_standard_robot1', 'field': 'rmuc_2027', 'lidar_mode': 'dual',
    'costmap_mode': 'split', 'use_rviz': 'true', 'point_lio_rviz': 'false',
    'split_local_refinement': 'false', 'stuck_cold_start': 'false',
    'relay_cmd_vel': 'true', 'publish_debug_visualizations': 'false',
    'filter_high_columns': 'false', 'dog_resolution_xy': '0.05', 'high_column_min_z': '0.20',
    'publish_map_to_odom': 'true', 'map_odom_offset_x': '0.0',
    'map_odom_offset_y': '0.0', 'map_odom_offset_yaw': '0.0',
}


def setup(context):
    source = LaunchConfiguration('data_source', default='sim').perform(context)
    if source == 'real':
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hero_real_bringup'), 'launch',
            'navigation_point_lio_backend_shadow_real.launch.py')))]
    if source != 'sim':
        raise ValueError('data_source must be sim or real')
    if LaunchConfiguration('field').perform(context) != 'rmuc_2027':
        raise ValueError('This first backend prior is calibrated for rmuc_2027 only; old entries remain available')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hero_sim_bringup'), 'launch', 'navigation_point_lio_sim.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in DEFAULTS}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hero_localization_backend'), 'launch', 'point_lio_backend_shadow.launch.py')),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'prior_map': LaunchConfiguration('prior_map'),
                              'backend_debug': LaunchConfiguration('backend_debug'),
                              'backend_rviz': LaunchConfiguration('backend_rviz'), 'use_sim_time': 'true'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in DEFAULTS.items()),
        DeclareLaunchArgument('data_source', default_value='sim'),
        DeclareLaunchArgument('prior_map', default_value='', description='Absolute 3-D prior bundle path, not the PGM'),
        DeclareLaunchArgument('backend_rviz', default_value='true'),
        DeclareLaunchArgument('backend_debug', default_value='false'),
        OpaqueFunction(function=setup),
    ])
