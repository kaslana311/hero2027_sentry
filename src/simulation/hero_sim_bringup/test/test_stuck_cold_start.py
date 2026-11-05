"""Opt-in wiring must not change maps, MPC, PVA policy or retry behavior."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from test_split_mode import BASE, PACKAGE, factory

TREE = PACKAGE / "behavior_trees/navigate_to_pose_sim.xml"


@pytest.mark.parametrize("mode", ("combined", "static_only", "dynamic_only", "split"))
def test_opt_in_changes_only_one_bt_port_and_two_smoother_parameters(mode):
    module = factory()
    args = (str(BASE), str(PACKAGE / "maps/rmuc_2027.yaml"), str(TREE),
            mode, "red_standard_robot1", False)
    original = TREE.read_bytes()
    baseline_path = Path(module._make_sim_params(*args))
    enabled_path = Path(module._make_sim_params(*args, stuck_cold_start=True))
    generated_tree = None
    try:
        baseline = yaml.safe_load(baseline_path.read_text())
        enabled = yaml.safe_load(enabled_path.read_text())
        nav = enabled["bt_navigator"]["ros__parameters"]
        generated_tree = Path(nav["default_nav_to_pose_bt_xml"])
        generated = ET.parse(generated_tree).getroot()
        stuck = generated.find(".//IsStuckCondition")
        assert stuck.attrib.pop("motion_stall_topic") == "navigation/motion_stalled"
        assert ET.tostring(generated) == ET.tostring(ET.parse(TREE).getroot())
        nav["default_nav_to_pose_bt_xml"] = str(TREE)
        smoother = enabled["smoother_server"]["ros__parameters"]["minco_smoother"]
        assert smoother.pop("stuck_state_topic") == "navigation/motion_stalled"
        assert smoother.pop("stuck_state_timeout") == 1.0
        assert enabled == baseline
        assert TREE.read_bytes() == original
    finally:
        baseline_path.unlink()
        enabled_path.unlink()
        if generated_tree is not None:
            generated_tree.unlink()


def test_flag_must_be_boolean():
    with pytest.raises(ValueError, match="stuck_cold_start must be a boolean"):
        factory()._make_sim_params(str(BASE), str(PACKAGE / "maps/rmuc_2027.yaml"),
            str(TREE), "split", "red_standard_robot1", False, stuck_cold_start="true")


@pytest.mark.parametrize("filename", ("perception_truth_sim.launch.py",
    "perception_point_lio_sim.launch.py", "navigation_point_lio_sim.launch.py",
    "navigation_point_lio_backend_shadow_sim.launch.py"))
def test_public_entry_exposes_false_default(filename):
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location("startup_switch_test", PACKAGE / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    flags = [item for item in description.entities
             if isinstance(item, DeclareLaunchArgument) and item.name == "stuck_cold_start"]
    assert len(flags) == 1
    assert "".join(part.text for part in flags[0].default_value) == "false"
