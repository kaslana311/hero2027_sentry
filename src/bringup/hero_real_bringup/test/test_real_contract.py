"""Real/simulation isolation, mount consistency and wire contracts, no hardware."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import math
import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(ROOT / 'src/localization/hero_localization_backend'))
from hero_real_bringup.configuration import backend_overrides, frontend_overrides, load_hardware, topic
from hero_localization_backend.configuration import parameters


def factory():
    path = ROOT / 'src/simulation/hero_sim_bringup/launch/perception_truth_sim.launch.py'
    spec = importlib.util.spec_from_file_location('shared_modes_real_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def reset_clock(value):
    if isinstance(value, dict):
        for k, v in value.items():
            if k == 'use_sim_time':
                assert v is False
                value[k] = True
            else:
                reset_clock(v)
    elif isinstance(value, list):
        for v in value:
            reset_clock(v)


@pytest.mark.parametrize('mode', ['combined', 'static_only', 'dynamic_only', 'split'])
@pytest.mark.parametrize('namespace', ['', 'red_standard_robot1'])
@pytest.mark.parametrize('stuck', [False, True])
def test_real_clock_only_does_not_change_algorithm_or_costmap(mode, namespace, stuck):
    module = factory()
    args = (str(ROOT / 'src/planning/hero2025_nav_bringup/config/nav2_params.yaml'),
            str(ROOT / 'src/simulation/hero_sim_bringup/maps/rmuc_2027.yaml'),
            str(ROOT / 'src/simulation/hero_sim_bringup/behavior_trees/navigate_to_pose_sim.xml'),
            mode, namespace, False, 'pointlio/cloud_registered_map', False, .20, .05,
            mode == 'split', stuck)
    sim_file, real_file = (Path(module._make_sim_params(*args, use_sim_time=clock)) for clock in (True, False))
    try:
        sim, real = (yaml.safe_load(p.read_text()) for p in (sim_file, real_file))
        reset_clock(real)
        if stuck:
            # Both generate equivalent temporary BTs with different filenames.
            s = sim['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
            r = real['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
            assert Path(s).read_bytes() == Path(r).read_bytes()
            real['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = s
        assert sim == real  # Includes every MINCO/MPC weight, recovery and mode plugin.
    finally:
        sim_file.unlink()
        real_file.unlink()


def test_hardware_input_units_and_timestamps_are_not_simulated():
    p = load_hardware(PACKAGE / 'config/hardware.yaml')
    dual = frontend_overrides(p, '', 'dual')
    single = frontend_overrides(p, '', 'single')
    assert dual['common.lid_topic'] == '/livox/fused_cloud'
    assert single['common.lid_topic'] == p['lidar2_topic']
    assert dual['common.imu_topic'] == '/livox/imu_192_168_1_153'
    assert dual['odometry.publish_odometry_without_downsample'] is False
    assert dual['publish.tf_send_en'] is False
    cfg = yaml.safe_load((PACKAGE/'config/point_lio_real.yaml').read_text())['/**']['ros__parameters']
    assert cfg['mapping']['acc_norm'] == 1.0
    assert cfg['mapping']['satu_acc'] == 5.0
    assert cfg['mapping']['vel_cov'] == 20.0
    assert cfg['mapping']['extrinsic_T'] == [-.011, -.02329, .04412]
    assert 'input/imu' not in str(dual)


def test_backend_uses_same_non_identity_mount_as_frontend_adapter():
    p = load_hardware(PACKAGE/'config/hardware.yaml')
    base = yaml.safe_load((ROOT/'src/localization/hero_localization_backend/config/backend_point_lio.yaml').read_text())
    original = deepcopy(base)
    cfg = backend_overrides(base, p, [2., 3., .15, .7])
    assert base == original
    body = cfg['common']['body']
    assert list(body['offset'].values()) == pytest.approx([-v for v in p['base_to_imu_xyz']])
    assert list(body['rotation'].values()) == pytest.approx([math.degrees(v) for v in p['base_to_imu_rpy']])
    assert cfg['startup_hint']['yaw_deg'] == pytest.approx(math.degrees(.7))
    assert cfg['startup_hint']['x'] == 2.
    assert cfg['recovery'] == base['recovery']


def test_root_namespace_backend_stays_private_and_no_prior_is_explicit():
    base = {'common': {}, 'prior_map': {}, 'uwb': {}, 'loop': {}, 'trajectory': {},
            'output': {'legacy': {}, 'world_cloud': {}}, 'services': {}, 'reset': {}}
    cfg = parameters(base, {}, '', '', allow_no_prior=True)
    assert cfg['common']['odom_topic'] == '/pointlio/odometry'
    assert cfg['common']['cloud_topic'] == '/pointlio/cloud_registered'
    assert cfg['output']['world_cloud']['topic'] == '/z_backend/cloud_registered_map'
    assert cfg['prior_map']['enabled'] is False
    with pytest.raises(ValueError, match='3-D prior'):
        parameters(base, {}, '', '')


def test_real_entry_no_truth_or_fabricated_sensors_and_serial_not_bypassed():
    source = (PACKAGE/'hero_real_bringup/launching.py').read_text()
    assert 'sim_ground_truth_bridge' not in source
    assert 'sim_point_lio_input_bridge' not in source
    assert 'sim_point_lio_evaluation' not in source
    assert "'relay_cmd_vel': 'false'" in source and "'start_serial': 'false'" in source
    assert "('cmd_vel_controller', topic(robot, 'cmd_vel'))" in source
    assert 'fake_vel_transform' not in source
    assert "'comparison_reference_label': 'frontend'" in source
    assert "'xml_config_path': os.path.join(decision_share" in source
    assert topic('', 'pointlio/odometry') == '/pointlio/odometry'
    assert topic('red_standard_robot1', 'tf') == '/red_standard_robot1/tf'


def test_each_original_entry_keeps_sim_default_and_adds_explicit_real_choice():
    for name in ('navigation_point_lio_sim.launch.py', 'navigation_point_lio_backend_shadow_sim.launch.py'):
        source = (ROOT/'src/simulation/hero_sim_bringup/launch'/name).read_text()
        assert 'data_source' in source and 'default_value=' in source
        assert 'hero_real_bringup' in source


def test_dual_merge_target_and_original_serial_protocol_unchanged():
    cmake = (ROOT/'src/perception/lidar_merge/CMakeLists.txt').read_text()
    assert 'target_compile_definitions(lidar_merge_dual_node PRIVATE TWO_LIDARS)' in cmake
    source = (ROOT/'src/serial/hero_serial/src/chassis_node.cpp').read_text()
    assert 'MPS_TO_MMPS = 1000.0' in source
    assert 'Init(serial_port, serial_baud)' in source
