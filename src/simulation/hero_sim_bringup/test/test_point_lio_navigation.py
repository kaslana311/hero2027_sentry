"""C2 black-box contract tests in an isolated ROS domain; no Gazebo/Nav2."""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from test_cloud_registration import stamp, rotation, set_pose


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class NavigationHarness:
    def __init__(self, relay=True):
        self.namespace = "/nav_lio_test_" + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node("nav_probe", namespace=self.namespace)
        self.odoms, self.tfs, self.static_tfs, self.commands, self.statuses = [], [], [], [], []
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.align_pub = self.node.create_publisher(TransformStamped,
            "pointlio/evaluation/cloud_alignment", durable)
        self.odom_pub = self.node.create_publisher(Odometry, "pointlio/odometry", qos_profile_sensor_data)
        self.cloud_pub = self.node.create_publisher(PointCloud2,
            "pointlio/cloud_registered_map", QoSProfile(depth=10))
        self.cmd_pub = self.node.create_publisher(Twist, "cmd_vel_nav2_result", 10)
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        self.node.create_subscription(Odometry, "odometry/filtered", self.odoms.append, 10)
        self.node.create_subscription(TFMessage, "tf", lambda m: self.tfs.extend(m.transforms), 10)
        self.node.create_subscription(TFMessage, "tf_static",
            lambda m: self.static_tfs.extend(m.transforms), durable)
        self.node.create_subscription(Twist, "cmd_vel", self.commands.append, 10)
        self.node.create_subscription(String, "pointlio/navigation/status",
            lambda m: self.statuses.append(json.loads(m.data)), durable)
        self.process = subprocess.Popen([
            os.environ["POINT_LIO_NAV_EXECUTABLE"], "--ros-args", "-r", f"__ns:={self.namespace}",
            "-r", "/tf:=tf", "-r", "/tf_static:=tf_static", "-p", "use_sim_time:=true",
            "-p", f"relay_cmd_vel:={str(relay).lower()}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.until(lambda: self.odom_pub.get_subscription_count() and
            self.align_pub.get_subscription_count() and self.cmd_pub.get_subscription_count()
            and self.cloud_pub.get_subscription_count() and self.statuses)
        self.clock(10.2)

    def until(self, condition, timeout=5.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            assert self.process.poll() is None, "navigation adapter exited"
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert condition(), "timed out waiting for navigation adapter"

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None
            rclpy.spin_once(self.node, timeout_sec=.02)

    def clock(self, seconds):
        m = Clock(clock=stamp(seconds))
        self.clock_pub.publish(m)
        self.spin_for(.08)

    def align(self, yaw=math.pi/2, translation=(5., 2., .3)):
        m = TransformStamped()
        m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(10.), "map", "pointlio_odom"
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = translation
        m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.align_pub.publish(m)
        self.spin_for(.08)
        return m

    def pose(self, seconds=10.1, chassis=(0., 0., 0., 0., 0., 0.), negative=False):
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(seconds), "pointlio_odom", "pointlio_imu"
        rot = rotation(chassis[3:])
        offset = (0., -.167, .121)
        imu = tuple(chassis[i] + sum(rot[i][j]*offset[j] for j in range(3)) for i in range(3))
        set_pose(m.pose.pose, (*imu, *chassis[3:]))
        if negative:
            q = m.pose.pose.orientation
            q.x, q.y, q.z, q.w = -q.x, -q.y, -q.z, -q.w
        self.odom_pub.publish(m)
        return m

    def cloud(self, seconds=10.15):
        m = PointCloud2()
        m.header.stamp, m.header.frame_id = stamp(seconds), "map"
        m.height, m.width, m.point_step, m.row_step = 1, 1, 12, 12
        m.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(("x", "y", "z"))]
        m.data = bytes(12)
        self.cloud_pub.publish(m)
        return m

    def ready(self):
        self.align()
        self.pose(10.1)
        self.until(lambda: len(self.odoms) == 1)
        self.pose(10.15, (.01, 0., 0., 0., 0., 0.))
        self.cloud()
        self.until(lambda: self.statuses[-1]["state"] == "ready")

    def command(self):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = .2, -.1, .15
        self.cmd_pub.publish(m)
        return m

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
    h = NavigationHarness.__new__(NavigationHarness)
    try:
        h.__init__()
        yield h
    finally:
        h.close()


def test_alignment_required_no_truth_fallback_and_single_navigation_source(harness):
    harness.pose()
    harness.command()
    harness.until(lambda: harness.commands)
    assert harness.commands[-1] == Twist() and not harness.odoms
    assert harness.statuses[-1]["state"] == "waiting_for_alignment"
    harness.ready()
    assert len(harness.node.get_publishers_info_by_topic(harness.namespace + "/odometry/filtered")) == 1
    topics = harness.node.get_subscriber_names_and_types_by_node(
        "sim_point_lio_navigation_bridge", harness.namespace)
    assert not any("truth" in name or "chassis_odometry" in name for name, _ in topics)
    assert {t.child_frame_id for t in harness.static_tfs} == {"odom"}


def test_chassis_extrinsic_timestamp_body_velocity_and_tilt_tf(harness):
    harness.align()
    source = harness.pose(10.1, (1., 2., .2, .1, .2, 0.))
    harness.until(lambda: harness.odoms)
    out = harness.odoms[-1]
    assert out.header.stamp == source.header.stamp
    assert out.header.frame_id == "odom" and out.child_frame_id == "base_link"
    p = out.pose.pose.position
    assert (p.x, p.y, p.z) == pytest.approx((3., 3., .5), abs=1e-8)
    harness.until(lambda: len(harness.tfs) >= 2)
    planar = next(t for t in harness.tfs if t.child_frame_id == "base_footprint")
    body = next(t for t in harness.tfs if t.child_frame_id == "base_link")
    assert planar.header.stamp == body.header.stamp == source.header.stamp
    assert planar.transform.translation.z == 0.
    assert body.transform.translation.z == pytest.approx(.5)
    assert abs(body.transform.rotation.y) > .08  # preserved slope pitch, not yaw-only
    source = harness.pose(10.15, (1.01, 2., .2, .1, .2, 0.))
    harness.until(lambda: len(harness.odoms) == 2)
    velocity = harness.odoms[-1].twist.twist.linear
    # Current body rotation transforms the world displacement into chassis axes.
    rot = rotation((.1, .2, 0.))
    assert (velocity.x, velocity.y, velocity.z) == pytest.approx(
        tuple(.2 * rot[0][i] for i in range(3)), abs=1e-8)


def test_angular_velocity_removes_imu_lever_arm_and_quaternion_sign(harness):
    harness.align(yaw=0., translation=(0., 0., 0.))
    harness.pose(10.1)
    harness.until(lambda: harness.odoms)
    # Equivalent quaternion signs must not produce a full-turn velocity.
    harness.pose(10.15, (0., 0., 0., 0., 0., .02), negative=True)
    harness.until(lambda: len(harness.odoms) == 2)
    twist = harness.odoms[-1].twist.twist
    assert (twist.linear.x, twist.linear.y, twist.linear.z) == pytest.approx((0., 0., 0.), abs=1e-8)
    assert twist.angular.z == pytest.approx(.4, abs=1e-8)


def test_cmd_unchanged_when_ready_and_zero_on_command_expiry(harness):
    harness.ready()
    m = harness.command()
    harness.until(lambda: harness.commands)
    assert harness.commands[-1] == m
    harness.until(lambda: harness.commands[-1] == Twist(), timeout=2.)
    assert harness.statuses[-1]["state"] == "ready"  # command timeout, not sensor failure


def test_stale_cloud_stops_motion_and_never_replays_old_command(harness):
    harness.ready()
    harness.command()
    harness.until(lambda: harness.commands and harness.commands[-1].linear.x == .2)
    harness.clock(11.3)
    harness.pose(11.2)
    harness.until(lambda: len(harness.odoms) == 3)
    harness.pose(11.25)
    harness.until(lambda: harness.statuses[-1]["state"] == "stale_cloud")
    harness.until(lambda: harness.commands[-1] == Twist())
    count = len(harness.commands)
    harness.cloud(11.25)
    harness.until(lambda: harness.statuses[-1]["state"] == "ready")
    harness.spin_for(.15)
    assert len(harness.commands) == count
    harness.command()
    harness.until(lambda: len(harness.commands) == count + 1)
    assert harness.commands[-1].linear.x == .2


def test_stale_pose_and_sim_pause_stop_using_wall_and_ros_age(harness):
    harness.ready()
    harness.command()
    harness.until(lambda: harness.commands)
    harness.spin_for(1.1)  # /clock does not advance; wall watchdog still detects lost data
    harness.until(lambda: harness.statuses[-1]["state"] == "stale_pose")
    assert harness.commands[-1] == Twist()
    harness.pose(10.2)
    harness.until(lambda: harness.statuses[-1]["state"] == "stale_cloud")


@pytest.mark.parametrize("kind", ["clock", "alignment", "odometry"])
def test_reset_or_changed_alignment_latches_restart_required(harness, kind):
    harness.ready()
    harness.command()
    harness.until(lambda: harness.commands)
    if kind == "clock":
        harness.clock(1.)
    elif kind == "alignment":
        harness.align(translation=(6., 2., .3))
    else:
        harness.pose(9.)
    harness.until(lambda: harness.statuses[-1]["state"] == "restart_required")
    assert harness.commands[-1] == Twist()
    count = len(harness.odoms)
    harness.clock(10.2)
    harness.pose(10.2)
    harness.cloud(10.2)
    harness.spin_for(.15)
    assert len(harness.odoms) == count
    assert harness.statuses[-1]["state"] == "restart_required"


def test_invalid_future_duplicate_frames_and_empty_cloud_do_not_enable_motion(harness):
    harness.align()
    m = harness.pose()
    harness.until(lambda: len(harness.odoms) == 1)
    for bad in [copy.deepcopy(m) for _ in range(3)]:
        bad.header.stamp = stamp(10.15)
        bad.pose.pose.position.x = float("nan")
        harness.odom_pub.publish(bad)
    harness.pose(11.)  # newer than current /clock, rejected
    wrong = copy.deepcopy(m)
    wrong.header.frame_id = "map"
    harness.odom_pub.publish(wrong)
    harness.odom_pub.publish(m)  # duplicate
    empty = PointCloud2()
    empty.header.stamp, empty.header.frame_id = stamp(10.15), "map"
    harness.cloud_pub.publish(empty)
    harness.spin_for(.1)
    assert len(harness.odoms) == 1
    harness.command()
    harness.until(lambda: harness.commands)
    assert harness.commands[-1] == Twist()


def test_relay_false_publishes_navigation_pose_but_no_motor_command():
    h = NavigationHarness.__new__(NavigationHarness)
    try:
        h.__init__(relay=False)
        h.ready()
        h.command()
        h.spin_for(.2)
        assert h.odoms and not h.commands
        assert not h.statuses[-1]["relay_enabled"]
    finally:
        h.close()


def test_c2_does_not_change_nav_parameters_and_isolates_truth_tf(monkeypatch):
    import yaml
    from launch import LaunchContext
    path = Path(__file__).parents[1] / "launch/perception_truth_sim.launch.py"
    spec = importlib.util.spec_from_file_location("truth_factory", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(namespace="red_standard_robot1", field="rmuc_2026",
        lidar_mode="dual", costmap_mode="combined", use_rviz="false", relay_cmd_vel="true",
        publish_debug_visualizations="false")
    nodes, files = [], []
    monkeypatch.setattr(module, "Node", lambda **kwargs: nodes.append(kwargs) or kwargs)
    original = module._make_sim_params
    def params(*args):
        result = original(*args)
        files.append(result)
        return result
    monkeypatch.setattr(module, "_make_sim_params", params)
    try:
        module._launch_setup(context, perception_cloud_topic="pointlio/cloud_registered_map")
        module._launch_setup(context, perception_cloud_topic="pointlio/cloud_registered_map",
                             point_lio_navigation=True)
        old = next(n for n in nodes[:3] if n["executable"] == "sim_ground_truth_bridge")
        new = next(n for n in nodes[3:] if n["executable"] == "sim_ground_truth_bridge")
        assert old["parameters"][0]["relay_cmd_vel"].perform(context) == "true"
        assert new["parameters"][0]["relay_cmd_vel"] is False
        assert new["parameters"][0]["output_odom_topic"] == "odometry/truth_planar"
        assert ("/tf", "truth/tf") in new["remappings"]
        assert ("/tf_static", "truth/tf_static") in new["remappings"]
        assert yaml.safe_load(Path(files[0]).read_text()) == yaml.safe_load(Path(files[1]).read_text())
    finally:
        for name in files:
            Path(name).unlink()


def test_actual_cloud_bridge_and_navigation_share_alignment_and_acquisition_stamp(harness):
    import struct
    from test_point_lio_cloud import CloudHarness, cloud
    c = CloudHarness.__new__(CloudHarness)
    try:
        c.__init__(namespace=harness.namespace + "/pointlio")
        harness.align()
        harness.pose(10.1)
        harness.until(lambda: len(harness.odoms) == 1)
        harness.pose(10.15, (.01, 0., 0., 0., 0., 0.))
        c.pub.publish(cloud(10.15, ((.01, 0., 0., 42.),)))
        c.until(lambda: c.clouds)
        harness.until(lambda: harness.statuses[-1]["state"] == "ready")
        nav = harness.odoms[-1]
        registered = c.clouds[-1]
        xyz = struct.unpack_from("<fff", registered.data, 0)
        position = nav.pose.pose.position
        assert xyz == pytest.approx((position.x, position.y, position.z), abs=1e-6)
        assert nav.header.stamp == registered.header.stamp == stamp(10.15)
        assert registered.header.frame_id == "map" and nav.header.frame_id == "odom"
        # No independent GT-driven correction or pose restamping is involved.
        assert harness.statuses[-1]["pose_stamp_ns"] == harness.statuses[-1]["cloud_stamp_ns"]
    finally:
        c.close()


def test_cloud_freshness_interlock_requests_reliable_delivery(harness):
    endpoints = harness.node.get_subscriptions_info_by_topic(
        harness.namespace + "/pointlio/cloud_registered_map")
    inputs = [e for e in endpoints if e.node_name == "sim_point_lio_navigation_bridge"]
    assert len(inputs) == 1
    assert inputs[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
