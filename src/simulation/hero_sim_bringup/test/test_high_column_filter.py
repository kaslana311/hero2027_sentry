"""Opt-in high-column filter; isolated ROS domain, no simulator or motor topics."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).parents[4]
DOG = ROOT / "src/planning/dog_map"


@pytest.fixture(scope="module")
def classifier(tmp_path_factory):
    executable = tmp_path_factory.mktemp("high_column_cpp") / "probe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(DOG / "include"),
                    str(Path(__file__).with_name("high_column_filter_probe.cpp")),
                    "-o", str(executable)], check=True, timeout=30)
    return executable


@pytest.mark.parametrize("enabled,index,threshold,offset,expected", [
    (False, 25, .30, .30, False),
    (True, 20, .30, .30, False),
    (True, 21, .30, .30, True),
    (True, 19, .30, .30, False),
    (True, 22, .30, .30, True),
    (True, 22, .50, .30, False),
    (True, 10, .30, .30, False),
    (True, 16, .30, .30, False),
    (True, 19, .30, .26, True),
    (True, -1, .30, .30, False),
])
def test_production_classifier_boundary_and_offset(classifier, enabled, index, threshold, offset, expected):
    result = subprocess.run([str(classifier), str(enabled).lower(), str(index), ".03",
                             str(threshold), str(offset)], check=True,
                            capture_output=True, text=True, timeout=5)
    assert result.stdout.strip() == str(expected).lower()


@pytest.mark.parametrize("threshold", ["nan", "inf", "-0.01"])
def test_production_classifier_rejects_invalid_height(classifier, threshold):
    result = subprocess.run([str(classifier), "true", "22", ".03", threshold, ".30"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and "finite and nonnegative" in result.stderr


def factory():
    path = ROOT / "src/simulation/hero_sim_bringup/launch/perception_truth_sim.launch.py"
    spec = importlib.util.spec_from_file_location("high_column_launch_factory", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_generated_parameters_change_only_the_two_filter_settings():
    module = factory()
    base = ROOT / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"
    files = []
    try:
        for enabled in (False, True):
            files.append(module._make_sim_params(str(base), "/tmp/map.yaml", "/tmp/bt.xml",
                         "combined", "red_standard_robot1", False,
                         "pointlio/cloud_registered_map", enabled, .36))
        off, on = [yaml.safe_load(Path(name).read_text()) for name in files]
        assert off["dog_map_node"]["ros__parameters"]["filter_high_columns"] is False
        assert on["dog_map_node"]["ros__parameters"]["filter_high_columns"] is True
        assert on["dog_map_node"]["ros__parameters"]["high_column_min_z"] == .36
        on["dog_map_node"]["ros__parameters"]["filter_high_columns"] = False
        assert on == off
    finally:
        for name in files:
            Path(name).unlink()


@pytest.mark.parametrize("threshold", [math.nan, math.inf, -.01])
def test_launch_rejects_invalid_height_before_writing_parameters(threshold):
    with pytest.raises(ValueError, match="finite and nonnegative"):
        factory()._make_sim_params("unused", "unused", "unused", "combined", "test",
                                  False, filter_high_columns=True, high_column_min_z=threshold)


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class DogHarness:
    def __init__(self, tmp_path, enabled, threshold):
        self.namespace = "/high_column_test_" + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node("high_column_probe", namespace=self.namespace)
        self.messages = []
        self.node.create_subscription(PointCloud2, "rog_map/inf_occ", self.messages.append,
                                      qos_profile_sensor_data)
        self.pub = self.node.create_publisher(PointCloud2, "input", 10)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_pub = self.node.create_publisher(TFMessage, "tf_static", durable)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "base_link"
        tf.transform.rotation.w = 1.
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        (tmp_path / "free.pgm").write_text("P2\n80 80\n255\n" + "255 " * 6400 + "\n")
        map_yaml = tmp_path / "map.yaml"
        map_yaml.write_text(yaml.safe_dump({"image": "free.pgm", "resolution": .1,
                          "origin": [-4., -4., 0.]}))
        base = ROOT / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"
        params = yaml.safe_load(base.read_text())["dog_map_node"]["ros__parameters"]
        params.update(map_yaml_path=str(map_yaml), cloud_topic="input", use_sim_time=False,
                      filter_high_columns=enabled, high_column_min_z=float(threshold))
        params_yaml = tmp_path / "params.yaml"
        params_yaml.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        executable = os.environ.get("DOG_MAP_EXECUTABLE") or str(
            Path(get_package_prefix("dog_map")) / "lib/dog_map/dog_map_node_lidars")
        self.log_path = tmp_path / "dog.log"
        self.log = self.log_path.open("w")
        self.process = subprocess.Popen([executable, "--ros-args",
            "-r", f"__ns:={self.namespace}", "--params-file", str(params_yaml),
            "-r", f"/tf:={self.namespace}/tf", "-r", f"/tf_static:={self.namespace}/tf_static"],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.until(lambda: self.pub.get_subscription_count() > 0)
        self.spin_for(.25)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log_path.read_text()
            rclpy.spin_once(self.node, timeout_sec=.02)

    def until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin_for(.02)
        assert predicate(), self.log_path.read_text()

    def feed(self, rows):
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(rows)
        msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(("x", "y", "z"))]
        msg.point_step, msg.row_step, msg.is_dense = 12, 12*len(rows), True
        msg.data = b"".join(struct.pack("<fff", *row) for row in rows)
        self.pub.publish(msg)
        self.spin_for(.11)

    def points(self):
        if not self.messages:
            return []
        msg = self.messages[-1]
        offsets = {field.name: field.offset for field in msg.fields}
        return [tuple(struct.unpack_from("<f", msg.data, row*msg.row_step + col*msg.point_step +
                      offsets[name])[0] for name in ("x", "y", "z"))
                for row in range(msg.height) for col in range(msg.width)]

    def close(self):
        if hasattr(self, "process"):
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if hasattr(self, "log"):
            self.log.close()
        if hasattr(self, "node"):
            self.node.destroy_node()


@pytest.fixture
def dog(tmp_path, ros_context, request):
    probe = DogHarness.__new__(DogHarness)
    try:
        enabled, threshold = getattr(request, "param", (True, .30))
        probe.__init__(tmp_path, enabled, threshold)
        yield probe
    finally:
        probe.close()


HIGH = [(1.21, .81, .37 + .03*k) for k in range(7)]
LOW = [(1.21, -.81, .03 + .03*k) for k in range(7)]


@pytest.mark.parametrize("dog,expected_high", [((False, .30), True), ((True, .30), False),
                                              ((True, .50), True)], indirect=["dog"])
def test_actual_dog_preserves_low_obstacles_and_filters_only_high_columns(dog, expected_high):
    for _ in range(15):
        dog.feed(HIGH + LOW)
        if any(p[1] < 0 for p in dog.points()):
            break
    assert any(p[1] < 0 for p in dog.points()), "low step/wall was lost"
    assert any(p[1] > 0 for p in dog.points()) is expected_high


def test_actual_dog_does_not_remove_ground_connected_column(dog):
    wall = [(1.21, .81, .03 + .03*k) for k in range(20)]
    for _ in range(15):
        dog.feed(wall)
        if dog.points():
            break
    assert dog.points(), "a column containing low occupied voxels must not be removed"


def test_actual_dog_preserves_nonempty_only_publication(dog):
    for _ in range(15):
        dog.feed(HIGH)
    assert not dog.messages, "historical behavior must not publish an empty filtered result"

    for _ in range(15):
        dog.feed(HIGH + LOW)
        if dog.points():
            break
    assert dog.points()
    for _ in range(45):
        dog.feed(HIGH)
    previous = len(dog.messages)
    for _ in range(10):
        dog.feed(HIGH)
    assert len(dog.messages) == previous, "empty results must leave the last published cloud unchanged"
    assert all(msg.width * msg.height > 0 for msg in dog.messages)
