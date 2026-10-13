"""Launch-configuration isolation; no ROS nodes, Gazebo or motor commands."""
from contextlib import contextmanager
import copy
import importlib.util
import math
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).parents[1]
ROOT = Path(__file__).parents[4]
BASE = ROOT / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"


def factory():
    spec = importlib.util.spec_from_file_location(
        "costmap_mode_factory", PACKAGE / "launch/perception_truth_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@contextmanager
def generated(mode, map_yaml):
    path = Path(factory()._make_sim_params(
        str(BASE), str(map_yaml), "/tmp/test_bt.xml", mode,
        "red_standard_robot1", False, "pointlio/cloud_registered_map", True, .20))
    try:
        yield yaml.safe_load(path.read_text())
    finally:
        path.unlink()


@pytest.mark.parametrize("field", ["rmuc_2025", "rmuc_2026"])
def test_dynamic_global_covers_selected_field_without_static_layer(field):
    map_yaml = PACKAGE / "maps" / f"{field}.yaml"
    metadata = yaml.safe_load(map_yaml.read_text())
    with generated("dynamic_only", map_yaml) as params:
        global_map = params["global_costmap"]["global_costmap"]["ros__parameters"]
        assert global_map["plugins"] == ["fast_layer", "inflation_layer"]
        assert global_map["rolling_window"] is False
        assert global_map["track_unknown_space"] is False
        assert type(global_map["width"]) is int and global_map["width"] == 30
        assert type(global_map["height"]) is int and global_map["height"] == 17
        assert global_map["resolution"] == metadata["resolution"]
        assert global_map["origin_x"] == metadata["origin"][0]
        assert global_map["origin_y"] == metadata["origin"][1]
        # Both ends of the current field are valid, not just a +/-6m window.
        assert global_map["origin_x"] <= 0 < global_map["origin_x"] + global_map["width"]
        assert global_map["origin_x"] <= 15 < global_map["origin_x"] + global_map["width"]


def test_only_dynamic_global_geometry_and_unknown_policy_change():
    original_yaml = BASE.read_bytes()
    map_yaml = PACKAGE / "maps/rmuc_2026.yaml"
    with generated("combined", map_yaml) as combined, generated("dynamic_only", map_yaml) as dynamic:
        combined = copy.deepcopy(combined)
        dynamic = copy.deepcopy(dynamic)
        for map_name in ("local_costmap", "global_costmap"):
            combined[map_name][map_name]["ros__parameters"].pop("plugins")
            dynamic[map_name][map_name]["ros__parameters"].pop("plugins")
        expected = combined["global_costmap"]["global_costmap"]["ros__parameters"]
        actual = dynamic["global_costmap"]["global_costmap"]["ros__parameters"]
        for key in ("rolling_window", "track_unknown_space", "origin_x", "origin_y",
                    "resolution", "width", "height"):
            expected.pop(key, None)
            actual.pop(key, None)
        assert dynamic == combined  # DOG, local geometry, MINCO, MPC, BT, frequencies.
    assert BASE.read_bytes() == original_yaml


@pytest.mark.parametrize("mode,plugins", [
    ("combined", ["static_layer", "fast_layer", "inflation_layer"]),
    ("static_only", ["static_layer", "inflation_layer"]),
])
def test_other_modes_keep_original_global_settings_and_do_not_read_pgm(mode, plugins):
    base = yaml.safe_load(BASE.read_text())
    expected = copy.deepcopy(base["global_costmap"]["global_costmap"]["ros__parameters"])
    expected.update(use_sim_time=True, plugins=plugins)
    expected["static_layer"]["map_topic"] = "/red_standard_robot1/map"
    expected["static_layer"]["cloud_update_topic"] = "/red_standard_robot1/rog_map/fi"
    expected["fast_layer"]["pointcloud_topic"] = "/red_standard_robot1/rog_map/inf_occ"
    with generated(mode, "/does-not-exist/map.yaml") as params:
        actual = params["global_costmap"]["global_costmap"]["ros__parameters"]
        assert actual == expected
        local = params["local_costmap"]["local_costmap"]["ros__parameters"]
        assert local["plugins"] == plugins
        assert local["rolling_window"] is True
        assert (local["width"], local["height"]) == (5, 5)


@pytest.mark.parametrize("magic", ["P2", "P5"])
def test_dynamic_geometry_ignores_black_white_pixels_and_accepts_comments(tmp_path, magic):
    image = tmp_path / "field.pgm"
    metadata = tmp_path / "field.yaml"
    metadata.write_text(yaml.safe_dump({"image": image.name, "resolution": .1,
                                       "origin": [-2., -1., 0.]}))
    results = []
    for gray in (0, 255):
        header = f"{magic}\n# test map\n40 30\n255\n".encode()
        pixels = (f"{gray} " * 1200).encode() if magic == "P2" else bytes([gray]) * 1200
        image.write_bytes(header + pixels)
        results.append(factory()._dynamic_global_geometry(str(metadata)))
    assert results[0] == results[1] == {
        "origin_x": -2., "origin_y": -1., "resolution": .1, "width": 4, "height": 3}


@pytest.mark.parametrize("resolution", [0., -.05, math.nan, math.inf])
def test_dynamic_geometry_rejects_invalid_resolution(tmp_path, resolution):
    metadata = tmp_path / "field.yaml"
    metadata.write_text(yaml.safe_dump({"image": "unused.pgm", "resolution": resolution,
                                       "origin": [0., 0., 0.]}))
    with pytest.raises(ValueError, match="finite and positive"):
        factory()._dynamic_global_geometry(str(metadata))


@pytest.mark.parametrize("header", [b"P6\n40 30\n255\n", b"P5\n0 30\n255\n", b"P5\n"])
def test_dynamic_geometry_rejects_invalid_pgm_header(tmp_path, header):
    image = tmp_path / "field.pgm"
    image.write_bytes(header)
    metadata = tmp_path / "field.yaml"
    metadata.write_text(yaml.safe_dump({"image": image.name, "resolution": .05,
                                       "origin": [0., 0., 0.]}))
    with pytest.raises(ValueError):
        factory()._dynamic_global_geometry(str(metadata))
