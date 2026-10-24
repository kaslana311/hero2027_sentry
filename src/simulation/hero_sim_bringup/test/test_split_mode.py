"""Split routing and exact legacy configuration snapshots; no ROS processes."""
from contextlib import contextmanager
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).parents[1]
ROOT = Path(__file__).parents[4]
BASE = ROOT / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"
FIELDS = ("rmuc_2025", "rmuc_2026", "rmuc_2027")


def factory():
    spec = importlib.util.spec_from_file_location(
        "split_factory", PACKAGE / "launch/perception_truth_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@contextmanager
def generated(mode, field, namespace="red_standard_robot1"):
    path = Path(factory()._make_sim_params(
        str(BASE), str(PACKAGE / "maps" / f"{field}.yaml"), "/tmp/test_bt.xml",
        mode, namespace, False, "pointlio/cloud_registered_map", True, .20))
    try:
        yield yaml.safe_load(path.read_text())
    finally:
        path.unlink()


# Canonical JSON hashes from pre-split commit 1c04855. Only the absolute
# selected-map YAML filename is replaced with <map_yaml>; every parameter is
# otherwise covered, including DOG, MINCO, MPC, BT, frequencies and old layers.
# The later, explicitly requested DOG XY setting is asserted separately, then
# removed before hashing; the original baseline hashes remain unchanged.
LEGACY_HASHES = {
    "combined": "07349ba456bbddae2bc0a631140cace2c99c4119e7543f38ceb6393e1a49ca73",
    "static_only": "2153b4a41212d13cc78e5d8f3156c0a25c10726f4decc21e92f0130594ff4eba",
    "rmuc_2025": "6946c7b51c6742c09835d52e9346830e39d24ab7369e636d5217899f1a404d67",
    "rmuc_2026": "207fff8f9271f192d1ef77f8130e450f52c24811379e5bd97ec8a46d08f02cb5",
    "rmuc_2027": "08792d019c5cfa67db560a04c1541ce42d8a99304438a71f14fd73c4a4c6d143",
}


@pytest.mark.parametrize("field", FIELDS)
@pytest.mark.parametrize("mode", ("combined", "static_only", "dynamic_only"))
def test_legacy_modes_match_every_pre_split_parameter(field, mode):
    with generated(mode, field) as params:
        assert params["dog_map_node"]["ros__parameters"].pop("resolution_xy") == .05
        # The common simulation BT now loads exactly one goal-isolation plugin.
        # Keep all historical hashes: only this explicitly allowed addition is
        # removed before comparing every remaining pre-split parameter.
        plugins = params["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
        assert plugins.pop() == "hero_goal_update_reset_bt_node"
        assert "hero_goal_update_reset_bt_node" not in plugins
        text = yaml.safe_dump(params).replace(
            str(PACKAGE / "maps" / f"{field}.yaml"), "<map_yaml>")
        canonical = json.dumps(yaml.safe_load(text), sort_keys=True, separators=(",", ":"))
        expected = LEGACY_HASHES[field if mode == "dynamic_only" else mode]
        assert hashlib.sha256(canonical.encode()).hexdigest() == expected


@pytest.mark.parametrize("field", FIELDS)
@pytest.mark.parametrize("namespace", ("red_standard_robot1", "blue_standard_robot2", ""))
def test_split_changes_only_planner_input_not_dynamic_maps_or_optimizers(field, namespace):
    original = BASE.read_bytes()
    with generated("split", field, namespace) as split, generated("dynamic_only", field, namespace) as dynamic:
        prior = split.pop("static_prior_costmap")["static_prior_costmap"]["ros__parameters"]
        assert prior["plugins"] == ["prior_static_layer", "inflation_layer"]
        assert not any(key in prior for key in ("fast_layer", "static_layer", "boundary_layer"))
        topic = f"/{namespace}/planning_prior/map" if namespace else "/planning_prior/map"
        assert prior["prior_static_layer"] == {
            "plugin": "nav2_costmap_2d::StaticLayer", "map_topic": topic,
            "map_subscribe_transient_local": True, "subscribe_to_updates": False,
            "footprint_clearing_enabled": False,
        }
        assert prior["rolling_window"] is False and prior["track_unknown_space"] is True
        master = dynamic["global_costmap"]["global_costmap"]["ros__parameters"]
        for key in ("global_frame", "robot_base_frame", "robot_radius", "resolution",
                    "inflation_layer", "update_frequency", "publish_frequency"):
            assert prior[key] == master[key]
        smoother = split["smoother_server"]["ros__parameters"]
        assert smoother.pop("costmap_topic") == "global_costmap/costmap_raw"
        planner = split["planner_server"]["ros__parameters"]["GridBased"]
        assert planner["plugin"] == "hero_static_prior_planner/StaticPriorPlanner"
        planner["plugin"] = dynamic["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
        assert split == dynamic
    assert BASE.read_bytes() == original


@pytest.mark.parametrize("field", FIELDS)
@pytest.mark.parametrize("namespace", ("red_standard_robot1", ""))
@pytest.mark.parametrize("use_rviz", ("true", "false"))
@pytest.mark.parametrize("cloud,navigation", (("cloud_registered", False),
    ("pointlio/cloud_registered_map", False), ("pointlio/cloud_registered_map", True)))
def test_split_prior_map_is_navigation_input_even_without_rviz(
    monkeypatch, field, namespace, use_rviz, cloud, navigation
):
    from launch import LaunchContext
    module = factory()
    context = LaunchContext()
    context.launch_configurations.update(namespace=namespace, field=field,
        lidar_mode="dual", costmap_mode="split", use_rviz=use_rviz,
        relay_cmd_vel="false", publish_debug_visualizations="false")
    nodes, files = [], []
    monkeypatch.setattr(module, "Node", lambda **kwargs: nodes.append(kwargs) or kwargs)
    monkeypatch.setattr(module, "_make_dynamic_boundary_map", lambda path: "/tmp/split_boundary.yaml")
    original = module._make_sim_params

    def params(*args):
        result = original(*args)
        files.append(Path(result))
        return result

    monkeypatch.setattr(module, "_make_sim_params", params)
    try:
        module._launch_setup(context, cloud, navigation)
        boundary = next(n for n in nodes if n.get("name") == "map_server")
        assert boundary["parameters"][0]["yaml_filename"] == "/tmp/split_boundary.yaml"
        prior = next(n for n in nodes if n.get("name") == "static_prior_map_server")
        lifecycle = next(n for n in nodes if n.get("name") == "lifecycle_manager_static_prior")
        assert prior["namespace"] == f"{namespace}/planning_prior"
        assert "condition" not in prior and "condition" not in lifecycle
        assert Path(prior["parameters"][0]["yaml_filename"]).name == f"{field}.yaml"
        assert lifecycle["parameters"][0]["node_names"] == ["static_prior_map_server"]
        assert not any(n.get("name") == "reference_map_server" for n in nodes)
        rviz = next(n for n in nodes if n.get("name") == "rviz2")
        assert ("map", "planning_prior/map") in rviz["remappings"]
        assert ("pointlio/cloud_registered_map", cloud) in rviz["remappings"]
    finally:
        for path in files:
            path.unlink()
