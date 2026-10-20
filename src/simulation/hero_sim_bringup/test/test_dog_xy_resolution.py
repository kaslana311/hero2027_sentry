"""Production grid geometry and isolated DOG; never publishes motor commands."""
import hashlib
import math
from pathlib import Path
import shlex
import subprocess

import pytest
import rclpy
import yaml

from test_costmap_modes import BASE, PACKAGE, factory
from test_high_column_filter import DogHarness, LOW

ROOT = Path(__file__).parents[4]


@pytest.fixture(scope="module")
def grid_probe(tmp_path_factory):
    executable = tmp_path_factory.mktemp("dog_xy_cpp") / "probe"
    flags = subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "opencv4"], text=True)
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "src/planning/dog_map/include"),
                    str(Path(__file__).with_name("dog_xy_grid_probe.cpp")),
                    "-o", str(executable), *shlex.split(flags)], check=True, timeout=30)
    return executable


def write_pgm(path, width, height, pixels):
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode() + bytes(pixels))


def probe_grid(executable, image, native, requested):
    result = subprocess.run([str(executable), str(image), str(native), str(requested)],
                            check=True, text=True, capture_output=True, timeout=5)
    lines = result.stdout.splitlines()
    geometry = [float(value) for value in lines[0].split()]
    pixels = [[int(value) for value in line.split()] for line in lines[1:]]
    return geometry, pixels


@pytest.mark.parametrize("requested", [0., .025])
def test_native_grid_is_unchanged_and_does_not_copy(grid_probe, tmp_path, requested):
    image = tmp_path / "map.pgm"
    source = list(range(15))
    write_pgm(image, 5, 3, source)
    geometry, pixels = probe_grid(grid_probe, image, .025, requested)
    assert geometry == pytest.approx([5, 3, .025, .125, .075, 1])
    assert [value for row in pixels for value in row] == source


def test_coarse_grid_is_lower_left_aligned_and_keeps_exact_extent(grid_probe, tmp_path):
    image = tmp_path / "odd.pgm"
    write_pgm(image, 5, 3, [10*y+x for y in range(3) for x in range(5)])
    before = image.read_bytes()
    geometry, pixels = probe_grid(grid_probe, image, .025, .05)
    assert geometry == pytest.approx([3, 2, .05, .125, .075, 0])
    assert pixels == [[0, 2, 4], [10, 12, 14]]
    assert image.read_bytes() == before


def test_coarsening_preserves_thin_wall_and_unknown_pixels(grid_probe, tmp_path):
    image = tmp_path / "walls.pgm"
    pixels = [255]*16
    pixels[1], pixels[14] = 0, 205
    write_pgm(image, 4, 4, pixels)
    _, actual = probe_grid(grid_probe, image, .025, .05)
    assert actual == [[0, 255], [255, 205]]


def test_real_2027_grid_is_560_by_321_without_changing_pgm(grid_probe):
    image = PACKAGE / "maps/rmuc_2027.pgm"
    before = hashlib.sha256(image.read_bytes()).hexdigest()
    geometry, _ = probe_grid(grid_probe, image, .025, .05)
    assert geometry == pytest.approx([560, 321, .05, 28., 16.05, 0])
    assert hashlib.sha256(image.read_bytes()).hexdigest() == before


@pytest.mark.parametrize("native,requested", [(.025, -.05), (.025, math.nan),
    (.025, math.inf), (.025, .03), (.05, .025), (0., .05), (math.nan, .05)])
def test_invalid_grid_settings_fail_explicitly(grid_probe, tmp_path, native, requested):
    image = tmp_path / "map.pgm"
    write_pgm(image, 2, 2, [255]*4)
    result = subprocess.run([str(grid_probe), str(image), str(native), str(requested)],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 2 and result.stderr


@pytest.mark.parametrize("mode", ["combined", "static_only", "dynamic_only", "split"])
@pytest.mark.parametrize("field", ["rmuc_2025", "rmuc_2026", "rmuc_2027"])
def test_sim_defaults_only_change_dog_xy_setting(mode, field):
    module = factory()
    files = []
    try:
        for requested in (.05, 0.):
            files.append(Path(module._make_sim_params(str(BASE),
                str(PACKAGE / "maps" / f"{field}.yaml"), "/tmp/test_bt.xml",
                mode, "red_standard_robot1", False, dog_resolution_xy=requested)))
        coarse, old = [yaml.safe_load(path.read_text()) for path in files]
        assert coarse["dog_map_node"]["ros__parameters"].pop("resolution_xy") == .05
        assert old["dog_map_node"]["ros__parameters"].pop("resolution_xy") == 0.
        assert coarse == old
        assert coarse["dog_map_node"]["ros__parameters"]["resolution_z"] == .03
    finally:
        for path in files:
            path.unlink()


@pytest.mark.parametrize("value", [-.05, math.nan, math.inf])
def test_launch_rejects_invalid_xy_setting(value):
    with pytest.raises(ValueError, match="finite and nonnegative"):
        factory()._make_sim_params("unused", "unused", "unused", "combined",
                                  "test", False, dog_resolution_xy=value)


@pytest.fixture(scope="module")
def xy_ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize("resolution_xy,expected_wall", [(0., False), (.025, False), (.05, True)])
def test_actual_dog_groups_sparse_wall_without_changing_filter_rules(
    tmp_path, xy_ros_context, resolution_xy, expected_wall
):
    dog = DogHarness.__new__(DogHarness)
    try:
        dog.__init__(tmp_path, False, .20, map_resolution=.025, resolution_xy=resolution_xy)
        # Each 2.5 cm column spans <=9 cm and fails the unchanged 9.1 cm rule;
        # the same points span 18 cm in ONE 5 cm column and pass.
        wall = [(1.211+.006*k, .811, .03+.03*k) for k in range(7)]
        for _ in range(15):
            dog.feed(wall + LOW)
        points = dog.points()
        assert any(point[1] < 0 for point in points), "dense low-wall control disappeared"
        assert any(point[1] > 0 for point in points) is expected_wall
        control = next(point for point in points if point[1] < 0)
        expected_xy = (1.225, -.825) if resolution_xy == .05 else (1.2125, -.8125)
        assert control[:2] == pytest.approx(expected_xy, abs=1e-5)
        if expected_wall:
            log = dog.log_path.read_text()
            assert "PGM resolution=0.025, DOG XY resolution=0.05, DOG cells=160x160" in log
    finally:
        dog.close()
