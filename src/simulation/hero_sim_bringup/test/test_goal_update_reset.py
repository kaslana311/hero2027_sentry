"""Actual Nav2 control plugins, deterministic leaves, no ROS/motor commands."""
import os
import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
TREE = PACKAGE / 'behavior_trees/navigate_to_pose_sim.xml'
CASES = ('normal_arrival', 'throttled', 'planner_running', 'smoother_running',
         'unchanged_goal_replanning', 'stamp_only', 'yaw_change', 'frame_change',
         'many_goals', 'cancel_restart', 'new_goal_after_success', 'planning_failure',
         'old_race', 'old_race_inflight', 'old_safe_ready')


@pytest.fixture(scope='session')
def executables(tmp_path_factory):
    from ament_index_python.packages import get_package_prefix
    nav2_lib = Path(get_package_prefix('nav2_behavior_tree')) / 'lib'
    binary = os.environ.get('GOAL_RESET_TEST_EXECUTABLE')
    plugin = os.environ.get('GOAL_RESET_PLUGIN')
    actions = os.environ.get('GOAL_RESET_ACTIONS_EXECUTABLE')
    if binary and plugin and actions:
        return Path(binary), Path(plugin), nav2_lib, Path(actions)
    # A temporary control-only build for running pytest without a full
    # BUILD_TESTING=ON workspace. Production installation still uses the safe
    # workspace script; this creates/installs nothing in the user's workspace.
    temp = tmp_path_factory.mktemp('goal-reset-offline-build')
    (temp/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(goal_reset_control_test)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
add_library(hero_goal_update_reset_bt_node SHARED "${PACKAGE}/src/reset_on_goal_update.cpp")
target_include_directories(hero_goal_update_reset_bt_node PRIVATE "${PACKAGE}/include")
target_compile_features(hero_goal_update_reset_bt_node PRIVATE cxx_std_17)
target_compile_definitions(hero_goal_update_reset_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(hero_goal_update_reset_bt_node behaviortree_cpp_v3 geometry_msgs)
add_executable(goal_update_reset_control_flow "${PACKAGE}/test/test_goal_update_reset_control_flow.cpp")
target_compile_features(goal_update_reset_control_flow PRIVATE cxx_std_17)
ament_target_dependencies(goal_update_reset_control_flow behaviortree_cpp_v3 geometry_msgs nav_msgs)
add_executable(goal_update_reset_actions "${PACKAGE}/test/test_goal_update_reset_actions.cpp")
target_compile_features(goal_update_reset_actions PRIVATE cxx_std_17)
ament_target_dependencies(goal_update_reset_actions behaviortree_cpp_v3 geometry_msgs nav_msgs nav2_msgs rclcpp rclcpp_action)
''')
    subprocess.run(['cmake', '-S', str(temp), '-B', str(temp/'build'),
                    f'-DPACKAGE={PACKAGE}', '-DCMAKE_BUILD_TYPE=Release'],
                   check=True, capture_output=True, text=True, timeout=60)
    subprocess.run(['cmake', '--build', str(temp/'build'), '--parallel', '2'],
                   check=True, capture_output=True, text=True, timeout=60)
    return (temp/'build/goal_update_reset_control_flow',
            Path(plugin) if plugin else temp/'build/libhero_goal_update_reset_bt_node.so',
            nav2_lib, temp/'build/goal_update_reset_actions')


@pytest.mark.parametrize('case', CASES)
def test_goal_switch_control_flow(case, executables, tmp_path):
    root = ET.parse(TREE).getroot()
    wrapper = root.find('.//ResetOnGoalUpdate')
    assert wrapper is not None and wrapper.get('goal') == '{goal}'
    assert len(list(wrapper)) == 1 and list(wrapper)[0].tag == 'PipelineSequence'
    # Counterfactuals use the exact same production subtree without the guard.
    child = list(wrapper)[0] if case.startswith('old_') else wrapper
    xml = '<root main_tree_to_execute="Test"><BehaviorTree ID="Test">' + ET.tostring(child, encoding='unicode') + '</BehaviorTree></root>'
    path = tmp_path/'pipeline.xml'; path.write_text(xml)
    binary, plugin, nav2_lib, _ = executables
    run = subprocess.run([str(binary), str(plugin), str(path), str(nav2_lib), case],
                         capture_output=True, text=True, timeout=10)
    assert run.returncode == 0, run.stdout + run.stderr
    assert f'PASS {case}' in run.stdout


def test_reset_is_only_on_navigation_pipeline_not_recovery_actions():
    recovery = ET.parse(TREE).getroot().find(".//RecoveryNode[@name='NavigateRecovery']")
    assert [c.tag for c in recovery] == ['ResetOnGoalUpdate', 'ReactiveFallback']
    assert len(ET.parse(TREE).getroot().findall('.//ResetOnGoalUpdate')) == 1
    pipeline = recovery.find('ResetOnGoalUpdate/PipelineSequence')
    assert pipeline.find('RateController').get('hz') == '3.0'
    assert pipeline.find("RecoveryNode[@name='FollowPath']").get('number_of_retries') == '10'


@pytest.mark.parametrize('case', ('old_result_queued', 'throttled', 'planner_running',
                                 'smoother_running', 'delayed_cancel', 'legacy_race'))
def test_real_async_action_clients(case, executables, tmp_path):
    wrapper = ET.parse(TREE).getroot().find('.//ResetOnGoalUpdate')
    child = list(wrapper)[0] if case == 'legacy_race' else wrapper
    xml = '<root main_tree_to_execute="Test"><BehaviorTree ID="Test">' + ET.tostring(child, encoding='unicode') + '</BehaviorTree></root>'
    path = tmp_path/'async_pipeline.xml'; path.write_text(xml)
    _, plugin, nav2_lib, binary = executables
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='198', ROS_LOCALHOST_ONLY='1')
    run = subprocess.run([str(binary), str(plugin), str(path), str(nav2_lib), case],
                         env=env, capture_output=True, text=True, timeout=20)
    assert run.returncode == 0, run.stdout + run.stderr
    assert f'PASS async {case}' in run.stdout
    if case == 'delayed_cancel':
        assert 'Failed to get result for compute_path_to_pose in node halt!' in run.stderr


@pytest.mark.parametrize('mode', ('combined', 'static_only', 'dynamic_only', 'split'))
@pytest.mark.parametrize('field', ('rmuc_2025', 'rmuc_2026', 'rmuc_2027'))
def test_plugin_added_once_without_mutating_real_robot_yaml(mode, field):
    import yaml
    spec = importlib.util.spec_from_file_location('goal_reset_launch_factory', PACKAGE/'launch/perception_truth_sim.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    base = Path(__file__).resolve().parents[4]/'src/planning/hero2025_nav_bringup/config/nav2_params.yaml'
    before = base.read_bytes()
    original = yaml.safe_load(before)['bt_navigator']['ros__parameters']['plugin_lib_names']
    paths = []
    try:
        for source in (str(base), None):
            path = Path(module._make_sim_params(source or str(paths[-1]),
                str(PACKAGE/'maps'/f'{field}.yaml'), str(TREE), mode,
                'red_standard_robot1', False))
            paths.append(path)
            params = yaml.safe_load(path.read_text())
            plugins = params['bt_navigator']['ros__parameters']['plugin_lib_names']
            assert plugins == original + ['hero_goal_update_reset_bt_node']
            assert params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == str(TREE)
        assert base.read_bytes() == before
    finally:
        for path in paths:
            path.unlink()
