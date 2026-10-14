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
def test_dynamic_global_covers_selected_field_with_boundary_only_layer(field):
    map_yaml = PACKAGE / "maps" / f"{field}.yaml"
    metadata = yaml.safe_load(map_yaml.read_text())
    with generated("dynamic_only", map_yaml) as params:
        global_map = params["global_costmap"]["global_costmap"]["ros__parameters"]
        assert global_map["plugins"] == ["boundary_layer", "fast_layer", "inflation_layer"]
        assert global_map["boundary_layer"] == {
            "plugin": "nav2_costmap_2d::StaticLayer",
            "map_topic": "/red_standard_robot1/map",
            "map_subscribe_transient_local": True,
            "subscribe_to_updates": False,
            "use_maximum": True,
        }
        local_map = params["local_costmap"]["local_costmap"]["ros__parameters"]
        assert local_map["plugins"] == global_map["plugins"]
        assert local_map["boundary_layer"] == global_map["boundary_layer"]
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


def test_only_dynamic_boundary_layer_global_geometry_and_unknown_policy_change():
    original_yaml = BASE.read_bytes()
    map_yaml = PACKAGE / "maps/rmuc_2026.yaml"
    with generated("combined", map_yaml) as combined, generated("dynamic_only", map_yaml) as dynamic:
        combined = copy.deepcopy(combined)
        dynamic = copy.deepcopy(dynamic)
        for map_name in ("local_costmap", "global_costmap"):
            combined[map_name][map_name]["ros__parameters"].pop("plugins")
            dynamic[map_name][map_name]["ros__parameters"].pop("plugins")
            dynamic[map_name][map_name]["ros__parameters"].pop("boundary_layer")
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
        assert "boundary_layer" not in actual
        assert "boundary_layer" not in local


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


def read_generated_pgm(path):
    with Path(path).open("rb") as stream:
        assert stream.readline() == b"P5\n"
        width, height = (int(v) for v in stream.readline().split())
        assert stream.readline() == b"255\n"
        pixels = stream.read()
    assert len(pixels) == width * height
    return width, height, pixels


@pytest.mark.parametrize("field", ["rmuc_2025", "rmuc_2026"])
def test_boundary_map_preserves_originals_has_free_interior_and_occupied_padding(tmp_path, field):
    module = factory()
    original = PACKAGE / "maps" / f"{field}.yaml"
    metadata = yaml.safe_load(original.read_text())
    original_image = original.parent / metadata["image"]
    before = (original.read_bytes(), original_image.read_bytes())
    _, arena_x, arena_y = module._dynamic_map_info(str(original))
    generated_yaml = module._make_dynamic_boundary_map(str(original), output_directory=tmp_path)
    generated_metadata = yaml.safe_load(Path(generated_yaml).read_text())
    width, height, pixels = read_generated_pgm(generated_metadata["image"])
    assert (width, height) == (600, 340)
    assert generated_metadata["origin"] == metadata["origin"]
    assert generated_metadata["resolution"] == metadata["resolution"]
    rim = math.ceil(.20 / metadata["resolution"])
    for y in range(height):
        for x in range(width):
            free = rim <= x < arena_x - rim and rim <= y < arena_y - rim
            assert pixels[(height - 1 - y) * width + x] == (255 if free else 0)
    assert set(pixels) == {0, 255}  # No unknown gray, no original interior walls.
    assert (original.read_bytes(), original_image.read_bytes()) == before


@pytest.mark.parametrize("magic", ["P2", "P5"])
def test_boundary_map_is_independent_of_source_pixels(tmp_path, magic):
    image = tmp_path / "field.pgm"
    metadata = tmp_path / "field.yaml"
    metadata.write_text(yaml.safe_dump({"image":image.name, "resolution":.1,
                                       "origin":[-2.,-1.,0.]}))
    results = []
    for gray in (0, 255):
        header = f"{magic}\n# pixels deliberately ignored\n37 26\n255\n".encode()
        pixels = (f"{gray} " * 962).encode() if magic == "P2" else bytes([gray])*962
        image.write_bytes(header + pixels)
        result = factory()._make_dynamic_boundary_map(str(metadata), output_directory=tmp_path / str(gray))
        results.append(read_generated_pgm(yaml.safe_load(Path(result).read_text())["image"]))
    assert results[0] == results[1]
    assert results[0][:2] == (40, 30)


@pytest.mark.parametrize("boundary_width", [0., -.1, math.nan, math.inf, 100.])
def test_boundary_map_rejects_invalid_or_oversized_rim(tmp_path, boundary_width):
    with pytest.raises(ValueError):
        factory()._make_dynamic_boundary_map(str(PACKAGE / "maps/rmuc_2026.yaml"),
                                            boundary_width, tmp_path)
