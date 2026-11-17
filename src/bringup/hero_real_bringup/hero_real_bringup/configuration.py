"""Pure, testable hardware contracts. No ROS network or hardware side effects."""
from copy import deepcopy
from pathlib import Path
import math
import re
import tempfile
import yaml


def topic(namespace, name):
    robot = namespace.strip('/')
    if robot and not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', robot):
        raise ValueError('Invalid ROS namespace')
    return '/' + '/'.join(part for part in (robot, name.strip('/')) if part)


def load_hardware(path):
    with open(path, encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    for key in ('base_to_imu_xyz', 'base_to_imu_rpy'):
        values = profile[key]
        if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f'{key} requires three finite values')
        profile[key] = [float(v) for v in values]
    for name in ('lidar1_topic', 'lidar2_topic', 'imu_topic', 'fused_cloud_topic'):
        if not str(profile[name]).startswith('/'):
            raise ValueError(f'{name} must be an absolute external hardware topic')
    # The wire protocol stays in hero_serial. These values are passed as node
    # parameters; they never request elevated device permissions or change udev.
    if not str(profile['serial_port']).startswith('/dev/') or int(profile['serial_baud']) <= 0:
        raise ValueError('Invalid serial device/baud')
    return profile


def frontend_overrides(profile, namespace, lidar_mode):
    if lidar_mode not in ('single', 'dual'):
        raise ValueError('lidar_mode must be single or dual')
    return {
        'use_sim_time': False,
        'common.lid_topic': profile['fused_cloud_topic'] if lidar_mode == 'dual' else profile['lidar2_topic'],
        'common.imu_topic': profile['imu_topic'],
        'odometry.output_imu_pose': True,
        'odometry.publish_odometry_without_downsample': False,
        'odometry.odom_frame_id': 'odom', 'odometry.body_frame_id': 'pointlio_imu',
        'publish.body_frame_id': 'pointlio_imu', 'publish.tf_send_en': False,
    }


def backend_overrides(source, profile, initial_pose):
    if len(initial_pose) != 4 or not all(math.isfinite(float(v)) for v in initial_pose):
        raise ValueError('Initial pose must be finite XYZ and yaw radians')
    result = deepcopy(source)
    body = result.setdefault('common', {}).setdefault('body', {})
    body['offset'] = dict(zip(('x', 'y', 'z'), (-v for v in profile['base_to_imu_xyz'])))
    body['rotation'] = dict(zip(('roll_deg', 'pitch_deg', 'yaw_deg'),
                               (math.degrees(v) for v in profile['base_to_imu_rpy'])))
    result['startup_hint'] = dict(enabled=True, force_until_matched=True,
                                  x=float(initial_pose[0]), y=float(initial_pose[1]),
                                  z=float(initial_pose[2]), yaw_deg=math.degrees(initial_pose[3]))
    # Preserve the accepted backend fixes, sync policy and recovery guards.
    # A real scan and scan-end pose are paired without nearest/latest fallback.
    return result


def temporary_yaml(data, prefix):
    with tempfile.NamedTemporaryFile(mode='w', prefix=prefix, suffix='.yaml',
                                     encoding='utf-8', delete=False) as output:
        yaml.safe_dump(data, output, sort_keys=False)
        return output.name
