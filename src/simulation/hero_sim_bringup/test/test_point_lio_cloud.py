"""Registered-cloud deployment: isolated ROS domain, no Gazebo/motor commands."""
import copy
import importlib.util
import json
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
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from test_cloud_registration import stamp
from test_point_lio_evaluation import EvaluationHarness
from test_point_lio_input import InputHarness


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class CloudHarness:
    def __init__(self, sim_time=False, namespace=None):
        self.namespace = namespace or "/cloud_test_" + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node("cloud_probe", namespace=self.namespace)
        self.clouds, self.statuses = [], []
        self.durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.node.create_publisher(PointCloud2, "cloud_registered", qos_profile_sensor_data)
        self.align_pub = self.node.create_publisher(TransformStamped, "evaluation/cloud_alignment",
                                                    self.durable)
        self.node.create_subscription(PointCloud2, "cloud_registered_map", self.clouds.append,
                                      qos_profile_sensor_data)
        self.node.create_subscription(String, "cloud_deployment/status",
                                      lambda m: self.statuses.append(json.loads(m.data)), self.durable)
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.process = subprocess.Popen([
            os.environ["POINT_LIO_CLOUD_EXECUTABLE"], "--ros-args",
            "-r", f"__ns:={self.namespace}", "-p", f"use_sim_time:={str(sim_time).lower()}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.until(lambda: self.pub.get_subscription_count()
                   and self.align_pub.get_subscription_count() and self.statuses)
        self.spin_for(.15)

    def until(self, predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert self.process.poll() is None, "cloud bridge exited unexpectedly"
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert predicate(), "timed out waiting for cloud bridge"

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None
            rclpy.spin_once(self.node, timeout_sec=.02)

    def align(self, seconds=10., translation=(5., 2., .3), yaw=math.pi/2):
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(seconds), "map", "pointlio_odom"
        t, q = msg.transform.translation, msg.transform.rotation
        t.x, t.y, t.z = translation
        q.z, q.w = math.sin(yaw/2), math.cos(yaw/2)
        self.align_pub.publish(msg)
        return msg

    def clock(self, seconds):
        msg = Clock()
        msg.clock = stamp(seconds)
        self.clock_pub.publish(msg)
        self.spin_for(.15)

    def close(self):
        if hasattr(self, "process"):
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if hasattr(self, "node"):
            self.node.destroy_node()


@pytest.fixture
def harness():
    probe = CloudHarness.__new__(CloudHarness)
    try:
        probe.__init__()
        yield probe
    finally:
        probe.close()


def cloud(seconds=10., rows=((1., 0., 0., 42.),)):
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp(seconds), "pointlio_odom"
    msg.height, msg.width = len(rows), 1
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
    msg.point_step, msg.row_step, msg.is_dense = 16, 20, True
    msg.data = b"".join(struct.pack("<ffff", *point) + b"PAD!" for point in rows)
    return msg


def test_rigid_transform_preserves_acquisition_stamp_fields_and_padding(harness):
    harness.align()
    msg = cloud(10.1, ((1., 0., 0., 42.), (0., 2., 1., 7.)))
    harness.pub.publish(msg)
    harness.until(lambda: bool(harness.clouds))
    out = harness.clouds[-1]
    assert out.header.frame_id == "map" and out.header.stamp == msg.header.stamp
    assert out.fields == msg.fields and out.row_step == msg.row_step
    assert out.width == msg.width and out.height == msg.height
    assert struct.unpack_from("<ffff", out.data, 0) == pytest.approx((5., 3., .3, 42.))
    assert struct.unpack_from("<ffff", out.data, 20) == pytest.approx((3., 2., 1.3, 7.))
    assert bytes(out.data[16:20]) == bytes(out.data[36:40]) == b"PAD!"


def test_cloud_waits_for_alignment_and_never_falls_back(harness):
    harness.pub.publish(cloud(10.1))
    harness.spin_for(.15)
    assert not harness.clouds
    assert harness.statuses[-1]["state"] == "waiting_for_alignment"
    harness.align()
    harness.until(lambda: bool(harness.clouds))
    harness.pub.publish(cloud(10.2))
    harness.until(lambda: len(harness.clouds) == 2)
    assert harness.clouds[-1].header.stamp == stamp(10.2)


def test_output_delivers_to_reliable_consumers_and_best_effort_probes(harness):
    reliable_clouds = []
    harness.node.create_subscription(PointCloud2, "cloud_registered_map",
                                      reliable_clouds.append, QoSProfile(depth=10))
    topic = harness.namespace + "/cloud_registered_map"
    endpoints = harness.node.get_publishers_info_by_topic(topic)
    assert len(endpoints) == 1
    assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    harness.align()
    harness.spin_for(.2)
    msg = cloud(10.1)
    harness.pub.publish(msg)
    harness.until(lambda: reliable_clouds and harness.clouds)
    assert reliable_clouds[-1] == harness.clouds[-1]
    assert reliable_clouds[-1].header.stamp == msg.header.stamp


def test_actual_dog_map_receives_cloud_and_publishes_nonempty_occupancy(harness, tmp_path):
    """Run the installed production DOG map, not a SensorDataQoS stand-in."""
    import yaml
    from ament_index_python.packages import get_package_prefix

    # A small free map bounds the test; perception thresholds are copied unchanged
    # from the tracked navigation configuration. No Gazebo, Nav2 or motor topics.
    (tmp_path / "free.pgm").write_text("P2\n80 80\n255\n" + "255 " * (80 * 80) + "\n")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(yaml.safe_dump({
        "image": "free.pgm", "resolution": .1, "origin": [-4., -4., 0.],
        "negate": 0, "occupied_thresh": .65, "free_thresh": .196,
    }))
    base = Path(__file__).parents[4] / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"
    params = yaml.safe_load(base.read_text())["dog_map_node"]["ros__parameters"]
    params.update(map_yaml_path=str(map_yaml), cloud_topic="cloud_registered_map",
                  use_sim_time=False)
    params_yaml = tmp_path / "dog.yaml"
    params_yaml.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    received, occupied = [], []
    harness.node.create_subscription(PointCloud2, "rog_map/ground", received.append,
                                      qos_profile_sensor_data)
    harness.node.create_subscription(PointCloud2, "rog_map/inf_occ", occupied.append,
                                      qos_profile_sensor_data)
    tf_pub = harness.node.create_publisher(TFMessage, "tf_static", harness.durable)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "map", "base_link"
    tf.transform.rotation.w = 1.
    tf_pub.publish(TFMessage(transforms=[tf]))
    executable = os.environ.get("DOG_MAP_EXECUTABLE") or str(
        Path(get_package_prefix("dog_map")) / "lib/dog_map/dog_map_node_lidars")
    log_path = tmp_path / "dog.log"
    process = None
    with log_path.open("w") as log:
        try:
            process = subprocess.Popen([
                executable, "--ros-args", "-r", f"__ns:={harness.namespace}",
                "--params-file", str(params_yaml),
                "-r", f"/tf:={harness.namespace}/tf",
                "-r", f"/tf_static:={harness.namespace}/tf_static",
            ], stdout=log, stderr=subprocess.STDOUT)
            topic = harness.namespace + "/cloud_registered_map"
            harness.until(lambda: any(e.node_name == "dog_map_node" for e in
                harness.node.get_subscriptions_info_by_topic(topic)))
            endpoints = harness.node.get_subscriptions_info_by_topic(topic)
            dog = next(e for e in endpoints if e.node_name == "dog_map_node")
            assert dog.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            harness.align(10., translation=(0., 0., 0.), yaw=0.)
            harness.spin_for(.3)
            assert not received and not occupied
            # A dense vertical patch away from the self-filter; its height span
            # exceeds the real ground threshold. DOG map's existing z shift stays.
            rows = [(2.01, 1.01 + j * .1, k * .03, 42.)
                    for j in range(4) for k in range(14)]
            for i in range(20):
                assert process.poll() is None, log_path.read_text()
                harness.pub.publish(cloud(10.1 + i * .05, rows))
                harness.spin_for(.08)
                if received and any(m.width * m.height > 0 for m in occupied):
                    break
            assert received, "DOG map never received the adapted cloud:\n" + log_path.read_text()
            assert occupied and occupied[-1].width * occupied[-1].height > 0, log_path.read_text()
            out = occupied[-1]
            assert out.header.frame_id == "map"
            offsets = {field.name: field.offset for field in out.fields}
            x = struct.unpack_from("<f", out.data, offsets["x"])[0]
            y = struct.unpack_from("<f", out.data, offsets["y"])[0]
            assert 1.8 < x < 2.3 and .8 < y < 1.5
            assert "incompatible QoS" not in log_path.read_text()
        finally:
            if process is not None:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)


def test_expired_or_pre_alignment_cloud_is_not_replayed(harness):
    harness.pub.publish(cloud(9.))
    harness.spin_for(1.15)
    harness.align(10.)
    harness.pub.publish(cloud(9.9))
    harness.spin_for(.2)
    assert not harness.clouds
    harness.pub.publish(cloud(10.1))
    harness.until(lambda: len(harness.clouds) == 1)


def test_invalid_layout_frame_stamp_rejected_and_nan_is_preserved(harness):
    harness.align()
    valid = cloud(10.1)
    invalid = []
    bad = copy.deepcopy(valid); bad.header.frame_id = "base_link"; invalid.append(bad)
    bad = copy.deepcopy(valid); bad.header.stamp = stamp(0.); invalid.append(bad)
    bad = copy.deepcopy(valid); bad.row_step = 10; invalid.append(bad)
    bad = copy.deepcopy(valid); bad.is_bigendian = True; invalid.append(bad)
    bad = copy.deepcopy(valid); bad.fields[1].offset = 2; invalid.append(bad)
    bad = copy.deepcopy(valid); bad.fields.append(copy.deepcopy(bad.fields[0])); invalid.append(bad)
    for msg in invalid:
        harness.pub.publish(msg)
        harness.spin_for(.04)
    assert not harness.clouds
    harness.pub.publish(cloud(10.2, ((float("nan"), 0., 0., 42.),)))
    harness.until(lambda: bool(harness.clouds))
    assert math.isnan(struct.unpack_from("<f", harness.clouds[-1].data)[0])
    assert not harness.clouds[-1].is_dense


def test_normals_rotate_without_translation(harness):
    harness.align()
    msg = cloud(10.1)
    msg.fields += [PointField(name=name, offset=16+i*4, datatype=PointField.FLOAT32, count=1)
                   for i, name in enumerate(("normal_x", "normal_y", "normal_z"))]
    msg.point_step, msg.row_step = 28, 28
    msg.data = struct.pack("<fffffff", 1., 0., 0., 42., 1., 0., 0.)
    harness.pub.publish(msg)
    harness.until(lambda: bool(harness.clouds))
    assert struct.unpack_from("<fff", harness.clouds[-1].data, 16) == pytest.approx((0., 1., 0.),
                                                                                 abs=1e-6)


def test_new_alignment_or_cloud_time_reset_requires_restart(harness):
    seed = harness.align()
    harness.pub.publish(cloud(10.1))
    harness.until(lambda: bool(harness.clouds))
    harness.align_pub.publish(seed)  # same seed delivery is harmless
    harness.pub.publish(cloud(10.2))
    harness.until(lambda: len(harness.clouds) == 2)
    harness.align(11., translation=(50., 2., .3))
    harness.until(lambda: harness.statuses[-1]["state"] == "restart_required")
    harness.pub.publish(cloud(11.1))
    harness.spin_for(.15)
    assert len(harness.clouds) == 2


def test_backwards_cloud_time_stops_output(harness):
    harness.align()
    harness.pub.publish(cloud(10.1))
    harness.until(lambda: bool(harness.clouds))
    harness.pub.publish(cloud(1.))
    harness.until(lambda: harness.statuses[-1]["state"] == "restart_required")
    harness.pub.publish(cloud(11.))
    harness.spin_for(.15)
    assert len(harness.clouds) == 1


def test_clock_reset_stops_output_even_before_new_cloud():
    probe = CloudHarness.__new__(CloudHarness)
    try:
        probe.__init__(sim_time=True)
        probe.clock(10.)
        probe.align()
        probe.pub.publish(cloud(10.1))
        probe.until(lambda: bool(probe.clouds))
        probe.clock(1.)
        probe.until(lambda: probe.statuses[-1]["state"] == "restart_required")
        probe.pub.publish(cloud(11.))
        probe.spin_for(.1)
        assert len(probe.clouds) == 1
    finally:
        probe.close()


def test_adapter_has_no_truth_odom_motor_or_global_tf_interfaces(harness):
    subscriptions = {name for name, _ in harness.node.get_subscriber_names_and_types_by_node(
        "sim_point_lio_cloud_bridge", harness.namespace)}
    publishers = {name for name, _ in harness.node.get_publisher_names_and_types_by_node(
        "sim_point_lio_cloud_bridge", harness.namespace)}
    assert harness.namespace + "/evaluation/cloud_alignment" in subscriptions
    assert not any(name.endswith(("/odometry", "truth_3d", "cmd_vel")) for name in subscriptions)
    assert not any(name.endswith(("/tf", "/tf_static", "/odometry", "cmd_vel"))
                   for name in publishers)


def test_observer_seed_is_once_latched_and_drift_is_not_corrected():
    observer = EvaluationHarness.__new__(EvaluationHarness)
    adapter = CloudHarness.__new__(CloudHarness)
    try:
        observer.__init__(cloud_alignment=True)
        observer.pair(10., (5., 2., .2, 0., 0., math.pi/2), (0.,)*6)
        # Start AFTER the seed: transient-local output must initialize a late subscriber.
        adapter.__init__(namespace=observer.namespace)
        adapter.until(lambda: adapter.statuses[-1]["state"] == "waiting_for_cloud")
        adapter.pub.publish(cloud(10.1))
        adapter.until(lambda: bool(adapter.clouds))
        assert struct.unpack_from("<fff", adapter.clouds[-1].data) == pytest.approx(
            (5.217, 3., .321), abs=1e-6)
        observer.pair(11., (5., 3., .2, 0., 0., math.pi/2), (1.3, 0., 0., 0., 0., 0.))
        observer.until(lambda: len(observer.errors) == 2)
        assert observer.errors[-1]["xy_error_m"] == pytest.approx(.3)
        adapter.pub.publish(cloud(11., ((1.3, 0., 0., 42.),)))
        adapter.until(lambda: len(adapter.clouds) == 2)
        assert struct.unpack_from("<fff", adapter.clouds[-1].data) == pytest.approx(
            (5.217, 3.3, .321), abs=1e-6)
    finally:
        adapter.close()
        observer.close()


def module(filename):
    path = Path(__file__).parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_only_dog_cloud_source_changes_in_generated_navigation_parameters():
    import yaml
    truth = module("perception_truth_sim.launch.py")
    # Locate the real tracked base YAML, not an invented fixture.
    base = Path(__file__).parents[4] / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"
    generated = []
    try:
        for topic in ("cloud_registered", "pointlio/cloud_registered_map"):
            path = truth._make_sim_params(str(base), "/tmp/map.yaml", "/tmp/bt.xml",
                                          "combined", "red_standard_robot1", False, topic)
            generated.append(Path(path))
        old, new = [yaml.safe_load(path.read_text()) for path in generated]
        assert new["dog_map_node"]["ros__parameters"]["cloud_topic"] == "pointlio/cloud_registered_map"
        new["dog_map_node"]["ros__parameters"]["cloud_topic"] = "cloud_registered"
        assert old == new  # Includes plugin lists, frequencies, odom, retries, MPC/minco parameters.
    finally:
        for path in generated:
            path.unlink()


def test_new_entry_is_dual_only_and_instantiates_one_estimator():
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch_ros.actions import Node
    launch = module("perception_point_lio_sim.launch.py")
    context = LaunchContext()
    context.launch_configurations.update({
        "namespace": "red_standard_robot1", "field": "rmuc_2026",
        "lidar_mode": "dual", "costmap_mode": "combined", "use_rviz": "false",
        "relay_cmd_vel": "false", "publish_debug_visualizations": "false", "point_lio_rviz": "false",
    })
    # Check installed package presence; launch actions are inspected, NOT executed.
    assert Path(get_package_share_directory("hero_sim_bringup"), "maps/rmuc_2026.yaml").is_file()
    actions = launch._setup(context)
    nodes = [a for a in actions if isinstance(a, Node)]
    executables = [a.node_executable for a in nodes]
    assert executables.count("pointlio_mapping") == 1
    assert executables.count("sim_point_lio_input_bridge") == 1
    assert executables.count("sim_point_lio_cloud_bridge") == 1
    assert executables.count("sim_ground_truth_bridge") == 1
    context.launch_configurations["lidar_mode"] = "single"
    with pytest.raises(RuntimeError, match="dual"):
        launch._setup(context)


def test_real_frontend_registered_cloud_layout_is_accepted():
    frontend = InputHarness.__new__(InputHarness)
    adapter = CloudHarness.__new__(CloudHarness)
    try:
        frontend.__init__(frontend=True)
        adapter.__init__(namespace=frontend.node.get_namespace())
        adapter.align(1.)
        points = []
        for i in range(30):
            for j in range(30):
                x, y = -.9 + i * .06, -.9 + j * .06
                points.extend(((x, y, -.3), (2., x, y), (x, 2., y)))
        for frame in range(35):
            base = 1. + frame * .05
            for i in range(10):
                frontend.imu(base + i * .005, acc=(0., 0., 9.8), gyro=(0., 0., 0.))
            frontend.scan(base + .045, 0, points=points)
            frontend.scan(base + .045, 1, points=points)
            frontend.spin(.035)
            adapter.spin_for(.01)
        frontend.until(lambda: len(frontend.registered) >= 3)
        adapter.until(lambda: len(adapter.clouds) >= 3)
        frontend.spin(.1)
        out = adapter.clouds[-1]
        original = next(m for m in frontend.registered if m.header.stamp == out.header.stamp)
        assert out.fields == original.fields and out.width == original.width
        offsets = {f.name: f.offset for f in original.fields}
        source = [struct.unpack_from("<f", original.data, offsets[n])[0] for n in ("x", "y", "z")]
        target = [struct.unpack_from("<f", out.data, offsets[n])[0] for n in ("x", "y", "z")]
        assert target == pytest.approx((5.-source[1], 2.+source[0], .3+source[2]), abs=1e-5)
        assert out.header.frame_id == "map"
        for name in ("intensity", "curvature"):
            assert out.data[offsets[name]:offsets[name]+4] == original.data[offsets[name]:offsets[name]+4]
    finally:
        adapter.close()
        frontend.close()
