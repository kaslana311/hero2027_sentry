"""Fast I/O/map-format contracts; runtime correction is tested separately."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import struct
import sys
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from hero_localization_backend.configuration import merge, parameters


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def bundle(tmp_path):
    (tmp_path/'clouds').mkdir()
    for name in ('keyframes.txt', 'sc_descriptors.bin', 'clouds/000000.pcd'):
        (tmp_path/name).touch()
    return tmp_path


def base():
    return {'common': {}, 'prior_map': {}, 'uwb': {}, 'loop': {}, 'trajectory': {},
            'output': {'legacy': {}, 'world_cloud': {}}, 'services': {}, 'reset': {}}


def test_raw_inputs_private_outputs_no_truth_feedback(bundle):
    source = base()
    original = deepcopy(source)
    overrides = yaml.safe_load((ROOT/'config/backend_point_lio.yaml').read_text())
    config = parameters(source, overrides, 'red_standard_robot1', bundle)
    assert source == original
    assert config['common']['cloud_topic'] == '/red_standard_robot1/pointlio/cloud_registered'
    assert config['common']['sync_odom_topic'] == config['common']['odom_topic']
    assert config['common']['input_cloud_in_local_frame'] is False
    assert config['common']['odom_cloud_sync']['max_interval'] < .05
    assert config['common']['body']['offset'] == {'x': 0., 'y': .167, 'z': -.121}
    assert config['uwb']['enabled'] is config['loop']['enabled'] is False
    assert not config['output']['legacy']['enabled']
    assert not config['output']['world_cloud']['publish_untrusted']
    assert config['reset']['service'].startswith('/red_standard_robot1/z_backend/')
    assert 'truth' not in str(config)


@pytest.mark.parametrize('name', ['', '/', 'bad name', 'bad-name'])
def test_bad_namespace(bundle, name):
    with pytest.raises(ValueError):
        parameters(base(), {}, name, bundle)


def test_missing_map_fails_before_starting_backend(tmp_path):
    with pytest.raises(ValueError, match='3-D prior'):
        parameters(base(), {}, 'robot', tmp_path)


def test_merge_preserves_unrelated_upstream_defaults():
    assert merge({'a': {'x': 1, 'y': 2}}, {'a': {'x': 3}}) == {'a': {'x': 3, 'y': 2}}


def test_monitor_time_interpolation_not_latest_and_bounded():
    module = load_script('backend_shadow_monitor')
    assert module.interpolate_position([(100, (0., 0., 0.)), (200, (2., 4., 6.))], 150) == (1., 2., 3.)
    assert module.interpolate_position([(100, (0., 0., 0.))], 200) is None
    assert module.interpolate_position([(100, (0., 0., 0.)), (200, (1., 1., 1.))], 150, 50) is None
    assert module.parse_state('status=LOST;pose_trusted=false;reason=a%3Bb')['reason'] == 'a;b'
    source = (ROOT/'scripts/backend_shadow_monitor.py').read_text()
    assert 'maxlen=2000' in source
    assert 'TransformBroadcaster' not in source and 'cmd_vel' not in source


def test_surface_sampling_preserves_slope_and_binary_map_format(tmp_path):
    module = load_script('prepare_rmuc_2027_prior')
    triangle = np.array([[[0., 0., 0.], [1., 0., .2], [0., 1., .1]]])
    points = module.sample_surface(triangle, .1)
    assert len(points) > 40
    assert np.allclose(points[:, 2], .2*points[:, 0]+.1*points[:, 1])
    assert np.array_equal(points, module.sample_surface(triangle, .1))
    target = tmp_path/'cloud.pcd'
    module.write_pcd(target, points)
    header, binary = target.read_bytes().split(b'DATA binary\n')
    assert b'FIELDS x y z intensity' in header
    assert len(binary) == len(points)*16
    assert np.isfinite(np.frombuffer(binary, dtype='<f4')).all()
    with pytest.raises(FileExistsError):
        module.write_pcd(target, points)


def test_support_heights_preserve_grade_and_headroom():
    module = load_script('prepare_rmuc_2027_prior')
    floor = np.array([[[0., 0., 0.], [1., 0., .05], [0., 1., .05]]])
    assert module.support_height(floor, (floor.min(1), floor.max(1)), .2, .2) == pytest.approx(.02)
    both = np.concatenate([floor, floor+np.array([0., 0., .1])])
    assert module.support_height(both, (both.min(1), both.max(1)), .2, .2) == pytest.approx(.12)


def test_bad_stl_is_rejected(tmp_path):
    module = load_script('prepare_rmuc_2027_prior')
    path = tmp_path/'bad.stl'
    path.write_bytes(b'\0'*80+struct.pack('<I', 1))
    with pytest.raises(ValueError):
        module.read_triangles(path)
