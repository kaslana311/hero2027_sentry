"""Explicit input/output contracts for the pinned upstream backend."""
from copy import deepcopy
from pathlib import Path
import re


def merge(base, overrides):
    result = deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def parameters(base, overrides, namespace, prior_map):
    robot = namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', robot):
        raise ValueError('namespace must be a nonempty ROS namespace')
    directory = Path(prior_map).expanduser().resolve()
    for name in ('keyframes.txt', 'sc_descriptors.bin'):
        if not (directory / name).is_file():
            raise ValueError(f'Missing {directory / name}; generate the 3-D prior bundle first')
    if not any((directory / 'clouds').glob('*.pcd')):
        raise ValueError(f'No local keyframe clouds in {directory}')
    config = merge(base, overrides)
    inputs = config['common']
    inputs.update(frontend='legacy', odom_frame='pointlio_odom', sensor_frame='pointlio_imu',
                  map_frame='map', input_cloud_in_local_frame=False,
                  odom_topic=f'/{robot}/pointlio/odometry',
                  sync_odom_topic=f'/{robot}/pointlio/odometry',
                  cloud_topic=f'/{robot}/pointlio/cloud_registered')
    config['prior_map'].update(enabled=True, directory=str(directory))
    config['uwb']['enabled'] = False
    config['loop']['enabled'] = False
    config['trajectory']['save_on_shutdown'] = False
    config['output']['legacy']['enabled'] = False
    config['output']['world_cloud'].update(
        topic=f'/{robot}/z_backend/cloud_registered_map', publish_untrusted=False)
    config['services'].update(save_map='~/save_map', relocalize='~/relocalize', reset='~/reset')
    # No implementation exists in Point-LIO: never target another stack's reset service.
    config['reset']['service'] = f'/{robot}/z_backend/unsupported_frontend_reset'
    return config
