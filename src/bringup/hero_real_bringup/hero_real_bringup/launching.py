"""Real I/O around the existing four-mode navigation, never another planner."""
import importlib.util
import math
import os
from pathlib import Path
import shutil
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .configuration import backend_overrides, frontend_overrides, load_hardware, temporary_yaml, topic


DEFAULTS = {
    'namespace': '', 'field': 'rmuc_2027', 'lidar_mode': 'dual', 'costmap_mode': 'split',
    'split_local_refinement': 'false', 'stuck_cold_start': 'false',
    'filter_high_columns': 'false', 'high_column_min_z': '0.20', 'dog_resolution_xy': '0.05',
    'publish_debug_visualizations': 'false', 'use_rviz': 'true', 'point_lio_rviz': 'false',
    'backend_rviz': 'true', 'backend_debug': 'false', 'prior_map': '', 'map': '',
    'hardware_profile': '', 'point_lio_config': '', 'livox_config': '', 'color_map': '',
    'initial_x': '0.0', 'initial_y': '0.0', 'initial_z': '0.0', 'initial_yaw': '0.0',
    'map_odom_offset_x': '0.0', 'map_odom_offset_y': '0.0', 'map_odom_offset_yaw': '0.0',
    'publish_map_to_odom': 'true',
    'start_lidar_driver': 'true', 'start_lidar_merge': 'true',
    # Opening the original serial node can itself command the chassis/spin.
    # Require an explicit real-hardware opt-in, independently of planning.
    'relay_cmd_vel': 'false', 'start_serial': 'false',
    'start_vision_bridge': 'false', 'start_zenoh': 'false', 'start_decision': 'false',
}


def factory():
    path = os.path.join(share('hero_sim_bringup'), 'launch', 'perception_truth_sim.launch.py')
    spec = importlib.util.spec_from_file_location('hero_shared_navigation_modes', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def setup(context, with_backend=True):
    def value(name):
        return LaunchConfiguration(name, default=DEFAULTS[name]).perform(context)

    def enabled(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return text == 'true'

    robot, mode = value('namespace').strip('/'), value('lidar_mode')
    package = share('hero_real_bringup')
    hardware = load_hardware(value('hardware_profile') or os.path.join(package, 'config', 'hardware.yaml'))
    start = [float(value('initial_' + axis)) for axis in ('x', 'y', 'z', 'yaw')]
    if not all(math.isfinite(v) for v in start):
        raise ValueError('Initial placement must contain finite XYZ/yaw values')
    tf_remaps = [('/tf', topic(robot, 'tf')), ('/tf_static', topic(robot, 'tf_static'))]
    nodes = []
    if enabled('start_lidar_driver'):
        driver_config = value('livox_config') or os.path.join(package, 'config', 'livox_dual.json')
        if not Path(driver_config).is_file():
            raise ValueError(f'Livox JSON not found: {driver_config}')
        nodes.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                          name='livox_lidar_publisher', output='screen', parameters=[{
                              'xfer_format': 1, 'multi_topic': 1, 'data_src': 0,
                              'publish_freq': 20.0, 'output_data_type': 0,
                              'frame_id': 'livox_frame', 'user_config_path': driver_config,
                              'cmdline_input_bd_code': 'livox0000000001', 'use_sim_time': False}]))
    if enabled('start_lidar_merge') and mode == 'dual':
        nodes.append(Node(package='lidar_merge', executable='lidar_merge_dual_node',
                          name='lidar_merge_node', output='screen', parameters=[{
                              **hardware['merge'], 'lidar1_topic': hardware['lidar1_topic'],
                              'lidar2_topic': hardware['lidar2_topic'], 'use_sim_time': False}],
                          remappings=[('/livox/fused_cloud', hardware['fused_cloud_topic'])]))
    config = value('point_lio_config') or os.path.join(package, 'config', 'point_lio_real.yaml')
    nodes.extend([
        Node(package='point_lio', executable='pointlio_mapping', namespace=topic(robot, 'pointlio'),
             parameters=[config, frontend_overrides(hardware, robot, mode)], output='screen',
             remappings=tf_remaps + [('/odometry/filtered', 'odometry')]),
        # This reused executable is a pure acquisition-time TF cloud transform;
        # use_navigation_tf=True has NO truth/evaluator subscription or seed.
        Node(package='hero_sim_bringup', executable='sim_point_lio_cloud_bridge',
             name='registered_cloud_to_map', namespace=topic(robot, 'pointlio'), output='screen',
             parameters=[{'use_sim_time': False, 'use_navigation_tf': True}], remappings=tf_remaps),
        Node(package='hero_real_bringup', executable='real_point_lio_navigation_bridge',
             namespace=robot, output='screen', parameters=[{
                 'use_sim_time': False, 'base_to_imu_xyz': hardware['base_to_imu_xyz'],
                 'base_to_imu_rpy': hardware['base_to_imu_rpy'], 'initial_base_pose': start,
                 'relay_cmd_vel': enabled('relay_cmd_vel'),
                 'publish_map_to_odom': enabled('publish_map_to_odom'),
                 **{key: float(value(key)) for key in ('map_odom_offset_x', 'map_odom_offset_y', 'map_odom_offset_yaw')},
             }], remappings=tf_remaps),
    ])
    # Identical PGM modes, A* recovery, BT, MINCO/MPC and switches as the parent
    # branch. Remove only the simulator truth bridge and select wall time.
    nodes.extend(factory()._launch_setup(context, perception_cloud_topic='pointlio/cloud_registered_map',
                                         point_lio_navigation=True, real_sensor_input=True))
    if enabled('start_serial'):
        nodes.append(Node(package='hero_serial', executable='chassis_node', output='screen',
                          parameters=[{'use_sim_time': False, 'serial_port': hardware['serial_port'],
                                       'serial_baud': int(hardware['serial_baud'])}],
                          remappings=[('cmd_vel_controller', topic(robot, 'cmd_vel')),
                                      ('/odometry/filtered', topic(robot, 'odometry/filtered'))]))
    if enabled('start_vision_bridge'):
        map_yaml = value('map') or os.path.join(share('hero_sim_bringup'), 'maps', value('field') + '.yaml')
        with open(map_yaml, encoding='utf-8') as stream:
            image = yaml.safe_load(stream)['image']
        color_map = value('color_map') or str(Path(map_yaml).parent / image)
        if not Path(color_map).is_file():
            raise ValueError(f'Vision region image not found: {color_map}')
        nodes.append(Node(package='nav_cv_bridge', executable='autoaim_tracker_node', output='screen',
                          parameters=[{'use_sim_time': False, 'yaml_file': map_yaml,
                                       'color_map_file': color_map, 'debug_image_file': ''}], remappings=tf_remaps))
    if enabled('start_zenoh'):
        executable = shutil.which('zenoh-bridge-ros2dds')
        if not executable:
            raise ValueError('start_zenoh=true requires the original zenoh-bridge-ros2dds tool')
        nodes.append(ExecuteProcess(cmd=[executable, '-d', os.environ.get('ROS_DOMAIN_ID', '0'),
                                        '-e', hardware['zenoh_endpoint'], '-c',
                                        os.path.join(package, 'config', 'zenoh_bridge.json5')], output='screen'))
    if enabled('start_decision'):
        if robot:
            raise ValueError('Original match decision uses root actions; use namespace:="" with start_decision=true')
        # Do not let the legacy YAML's absolute /home/dji/hero2026_-sentry
        # filename silently load a different, dirty checkout's decision tree.
        decision_share = share('bt')
        nodes.append(Node(package='bt', executable='decision_node', name='decision_node',
                          output='screen', parameters=[
                              os.path.join(decision_share, 'config', 'yaml', 'decision_node_param.yaml'),
                              {'xml_config_path': os.path.join(decision_share, 'config', 'xml',
                                                               'RMUC_26_5_29.xml'),
                               'use_sim_time': False}]))
    if enabled('point_lio_rviz'):
        nodes.append(Node(package='rviz2', executable='rviz2', namespace=robot, name='pointlio_real_rviz',
                          output='screen', parameters=[{'use_sim_time': False}], remappings=tf_remaps,
                          arguments=['-d', os.path.join(package, 'rviz', 'real_point_lio.rviz')]))
    if with_backend:
        with open(os.path.join(share('hero_localization_backend'), 'config', 'backend_point_lio.yaml')) as stream:
            backend = backend_overrides(yaml.safe_load(stream), hardware, start)
        backend_config = temporary_yaml(backend, 'hero_real_backend_')
        with open(os.path.join(share('hero_localization_backend'), 'rviz', 'backend_shadow.rviz')) as stream:
            display = yaml.safe_load(stream)
        for item in display['Visualization Manager']['Displays']:
            if 'truth chassis' in item.get('Name', ''):
                item.update(Name='No ground truth on real hardware', Enabled=False)
            if item.get('Topic', {}).get('Value') == 'comparison/frontend_path':
                item['Topic']['Value'] = topic(robot, 'z_backend/comparison/reference_path')
        real_backend_rviz = temporary_yaml(display, 'hero_real_backend_rviz_')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('hero_localization_backend'), 'launch', 'point_lio_backend_shadow.launch.py')),
            launch_arguments={
                'namespace': robot, 'use_sim_time': 'false', 'prior_map': value('prior_map'),
                'allow_no_prior': 'true', 'backend_config': backend_config,
                'backend_debug': value('backend_debug'), 'backend_rviz': value('backend_rviz'),
                'comparison_reference_topic': topic(robot, 'base_link_in_map_odom'),
                'comparison_reference_label': 'frontend',
                'backend_rviz_config': real_backend_rviz,
            }.items()))
    return [LogInfo(msg='REAL SENSOR INPUT; reference is last-season dual-MID360 calibration. '
                         'Re-survey hardware after any remount. NO Gazebo/ground-truth input. '
                         'NO automatic chassis activation. Backend stays isolated from navigation. '
                         'Without a real 3-D prior bundle it computes odometry factors only; '
                         'frontend/backend difference is NOT an absolute accuracy measurement.'), *nodes]


def description(with_backend=True):
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default) for name, default in DEFAULTS.items()),
        OpaqueFunction(function=lambda context: setup(context, with_backend)),
    ])
