"""Simulation-only sensor mounts must agree throughout the registration chain."""
from pathlib import Path
import re

PACKAGE = Path(__file__).resolve().parents[1]
EXPECTED_LEFT = (-.031029689, .157956386, .103822859, .004491681, .593489259, 1.574538158)
EXPECTED_RIGHT = (0., -.167, .121, 0., .61086, -1.57079)


def default_vector(filename, parameter):
    source = (PACKAGE / "src" / filename).read_text()
    match = re.search(r'"' + re.escape(parameter) + r'"\s*,\s*\{([^}]+)\}', source)
    assert match, f"missing mount parameter {parameter} in {filename}"
    return tuple(float(value.strip()) for value in match[1].split(","))


def test_bridge_mounts_and_virtual_imu_agree():
    for filename in ("sim_ground_truth_bridge.cpp", "sim_point_lio_input_bridge.cpp"):
        assert default_vector(filename, "left_lidar_pose") == EXPECTED_LEFT
        assert default_vector(filename, "right_lidar_pose") == EXPECTED_RIGHT
        assert default_vector(filename, "lidar_sensor_pose") == (0., 0., .03, 0., 0., 0.)
    for filename in ("sim_point_lio_evaluation.cpp", "sim_point_lio_navigation_bridge.cpp"):
        assert default_vector(filename, "chassis_to_virtual_imu") == EXPECTED_RIGHT[:3]


def test_point_lio_still_receives_points_in_the_virtual_imu_frame():
    import yaml
    config = yaml.safe_load((PACKAGE / "config/point_lio_snapshot.yaml").read_text())
    mapping = config["/**"]["ros__parameters"]["mapping"]
    assert mapping["extrinsic_T"] == [0., 0., 0.]
    assert mapping["extrinsic_R"] == [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    source = (PACKAGE / "src/sim_point_lio_input_bridge.cpp").read_text()
    assert "right.getOrigin()" in source
    assert "virtual_imu.inverse() * left * ray" in source
    assert "virtual_imu.inverse() * right * ray" in source
