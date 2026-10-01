"""Display-only comparison; isolated domain, no Gazebo or motor commands."""

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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from test_cloud_registration import set_pose, stamp


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class EvaluationHarness:
    def __init__(self):
        self.namespace = "/eval_test_" + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node("probe", namespace=self.namespace)
        self.process = subprocess.Popen([
            os.environ["EVALUATION_EXECUTABLE"], "--ros-args",
            "-r", f"__ns:={self.namespace}", "-r", "/tf_static:=tf_static",
            "-r", "/tf:=tf",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.truths, self.estimates, self.errors, self.transforms, self.markers = [], [], [], [], []
        self.node.create_subscription(PoseStamped, "evaluation/truth_pose", self.truths.append, 10)
        self.node.create_subscription(PoseStamped, "evaluation/pointlio_pose", self.estimates.append, 10)
        self.node.create_subscription(String, "evaluation/error",
                                      lambda msg: self.errors.append(json.loads(msg.data)), 10)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TFMessage, "tf_static", self.transforms.append, durable)
        self.node.create_subscription(MarkerArray, "evaluation/markers", self.markers.append, durable)
        self.truth_pub = self.node.create_publisher(Odometry, "truth_3d", 10)
        self.estimate_pub = self.node.create_publisher(Odometry, "odometry", 10)
        self.until(lambda: self.truth_pub.get_subscription_count()
                   and self.estimate_pub.get_subscription_count())
        self.spin_for(.2)

    def until(self, predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert self.process.poll() is None, "evaluation process exited unexpectedly"
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert predicate(), "timed out waiting for comparison"

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None
            rclpy.spin_once(self.node, timeout_sec=.02)

    def send(self, seconds, pose, truth=False, frame=None):
        msg = Odometry()
        msg.header.stamp = stamp(seconds)
        msg.header.frame_id = frame or ("map" if truth else "pointlio_odom")
        msg.child_frame_id = "sim_cloud_chassis_truth" if truth else "pointlio_imu"
        set_pose(msg.pose.pose, pose)
        (self.truth_pub if truth else self.estimate_pub).publish(msg)

    def pair(self, seconds, actual, local):
        count = len(self.estimates)
        self.send(seconds, actual, truth=True)
        self.send(seconds, local)
        self.until(lambda: len(self.estimates) > count)

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
    probe = EvaluationHarness.__new__(EvaluationHarness)
    try:
        probe.__init__()
        yield probe
    finally:
        probe.close()


def xyz(message):
    p = message.pose.position
    return p.x, p.y, p.z


def test_alignment_is_once_and_both_outputs_are_chassis_positions(harness):
    harness.pair(10., (5., 2., .2, 0., 0., math.pi / 2), (0.,) * 6)
    harness.until(lambda: bool(harness.transforms))
    transform = harness.transforms[0].transforms[0]
    assert transform.header.frame_id == "map"
    assert transform.child_frame_id == "pointlio_odom"
    assert transform.transform.translation.x == pytest.approx(5.217)
    assert transform.transform.translation.z == pytest.approx(.321)
    harness.pair(11., (5., 3., .2, 0., 0., math.pi / 2), (1., 0., 0., 0., 0., 0.))
    assert xyz(harness.estimates[-1]) == pytest.approx((5., 3., .2))
    harness.pair(12., (5., 3., .2, 0., 0., math.pi / 2), (1.3, 0., 0., 0., 0., 0.))
    harness.until(lambda: len(harness.errors) == 3 and harness.markers
                   and harness.markers[-1].markers[0].header.stamp == stamp(12.))
    assert harness.errors[-1]["xy_error_m"] == pytest.approx(.3)
    assert harness.errors[-1]["alignment_stamp_sec"] == 10.
    assert len(harness.transforms) == 1  # later drift is not hidden by re-alignment
    markers = {m.id: m for m in harness.markers[-1].markers}
    assert markers[0].pose.position.x == markers[0].pose.position.y == 0.
    assert "MAP = ODOM ORIGIN" in markers[1].text
    assert markers[10].color.g == 1.
    assert markers[20].color.b == 1.


def test_delayed_truth_is_interpolated_at_estimator_stamp(harness):
    harness.pair(10., (0., 0., .2, 0., 0., 0.), (0.,) * 6)
    # IMU offset rotates as chassis turns; a direct IMU/chassis comparison is wrong.
    harness.send(10.05, (.05 + .217 * math.sin(.2),
                        .217 * (1 - math.cos(.2)), 0., 0., 0., .2))
    harness.spin_for(.1)
    assert len(harness.estimates) == 1
    harness.send(10.1, (.1, 0., .2, 0., 0., .4), truth=True)
    harness.until(lambda: len(harness.estimates) == 2 and len(harness.truths) == 2)
    assert xyz(harness.truths[-1]) == pytest.approx((.05, 0., .2), abs=1e-8)
    assert xyz(harness.estimates[-1]) == pytest.approx((.05, 0., .2), abs=1e-8)
    assert harness.truths[-1].header.stamp == harness.estimates[-1].header.stamp == stamp(10.05)


def test_large_truth_gap_has_no_latest_pose_fallback(harness):
    harness.pair(10., (0.,) * 6, (0.,) * 6)
    harness.send(11., (10., 0., 0., 0., 0., 0.), truth=True)
    harness.send(10.5, (0.,) * 6)
    harness.spin_for(1.15)
    assert len(harness.estimates) == 1


def test_missing_truth_and_invalid_frames_do_not_align(harness):
    harness.send(0., (0.,) * 6, truth=True)
    harness.send(10., (0.,) * 6, truth=True, frame="world")
    harness.send(10., (0.,) * 6, frame="odom")
    harness.send(10., (0.,) * 6)
    harness.spin_for(.15)
    assert not harness.estimates and not harness.transforms
    harness.send(10., (2., 3., 0., 0., 0., 0.), truth=True)
    harness.until(lambda: bool(harness.estimates))
    assert xyz(harness.estimates[-1]) == pytest.approx((2., 3., 0.))


def test_time_reset_stops_comparison_instead_of_realigning(harness):
    harness.pair(10., (0.,) * 6, (0.,) * 6)
    harness.send(1., (0.,) * 6, truth=True)
    harness.spin_for(.1)
    harness.send(11., (2., 0., 0., 0., 0., 0.), truth=True)
    harness.send(11., (2., 0., 0., 0., 0., 0.))
    harness.spin_for(.2)
    assert len(harness.estimates) == 1
    assert any("TIME RESET" in m.text for m in harness.markers[-1].markers)


def test_observer_does_not_publish_navigation_or_estimator_inputs(harness):
    topics = {name for name, _ in harness.node.get_publisher_names_and_types_by_node(
        "sim_point_lio_evaluation", harness.namespace)}
    assert harness.namespace + "/tf_static" in topics
    assert "/tf_static" not in topics
    assert not any(name.endswith(("cmd_vel", "cloud_registered", "/odometry"))
                   for name in topics)


def test_rviz_has_camera_tools_and_shared_map_frame():
    import yaml
    path = Path(__file__).parents[1] / "rviz/point_lio_shadow.rviz"
    config = yaml.safe_load(path.read_text())
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    assert manager["Views"]["Current"]["Target Frame"] == "map"
    assert manager["Views"]["Current"]["Distance"] >= 30
    assert "rviz_common/Views" in {p["Class"] for p in config["Panels"]}
    tools = {t["Class"] for t in manager["Tools"]}
    assert "rviz_default_plugins/MoveCamera" in tools
    assert "rviz_default_plugins/SetGoal" not in tools
    topics = {d["Topic"]["Value"] for d in manager["Displays"] if "Topic" in d}
    assert {"evaluation/static_map", "evaluation/markers", "evaluation/truth_path",
            "evaluation/pointlio_path"}.issubset(topics)
