"""Isolated synthetic sensors; no Gazebo, navigation or motor commands."""
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
from ament_index_python.packages import get_package_prefix
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

from test_cloud_registration import stamp


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class InputHarness:
    def __init__(self, frontend=False):
        ns = "/lio_test_" + uuid.uuid4().hex[:10]
        self.node = rclpy.create_node("probe", namespace=ns)
        left = [0., 0., 0., 0., 0., 0.] if frontend else [.1, .2, .3, 0., 0., math.pi / 2]
        right = [0., 0., 0., 0., 0., 0.] if frontend else [-.1, -.2, .3, 0., 0., -math.pi / 2]
        self.processes = []
        self.processes.append(subprocess.Popen([
            os.environ["POINT_LIO_INPUT_EXECUTABLE"], "--ros-args", "-r", f"__ns:={ns}",
            "-p", f"left_lidar_pose:={left}", "-p", f"right_lidar_pose:={right}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        self.clouds, self.imus, self.odoms, self.registered = [], [], [], []
        self.node.create_subscription(CustomMsg, "input/fused_cloud", self.clouds.append, 10)
        self.node.create_subscription(Imu, "input/imu", self.imus.append, 100)
        self.node.create_subscription(Odometry, "estimate", self.odoms.append, 10)
        self.node.create_subscription(PointCloud2, "cloud_registered", self.registered.append, 10)
        self.pubs = [self.node.create_publisher(PointCloud2, name, qos_profile_sensor_data)
                     for name in ("livox/lidar_left", "livox/lidar_right")]
        self.imu_pub = self.node.create_publisher(Imu, "livox/imu_right", qos_profile_sensor_data)
        if frontend:
            executable = os.environ.get("POINT_LIO_EXECUTABLE") or str(
                Path(get_package_prefix("point_lio")) / "lib/point_lio/pointlio_mapping")
            cfg = Path(__file__).resolve().parents[1] / "config/point_lio_snapshot.yaml"
            self.processes.append(subprocess.Popen([
                executable, "--ros-args", "-r", f"__ns:={ns}", "--params-file", str(cfg),
                "-r", "/odometry/filtered:=estimate", "-r", "/tf:=tf",
                "-r", "/tf_static:=tf_static", "-p", "use_sim_time:=false",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        self.until(lambda: all(pub.get_subscription_count() for pub in self.pubs)
                   and self.imu_pub.get_subscription_count())
        self.spin(.35)

    def spin(self, seconds):
        stop = time.monotonic() + seconds
        while time.monotonic() < stop:
            assert all(p.poll() is None for p in self.processes), "adapter/frontend exited"
            rclpy.spin_once(self.node, timeout_sec=.01)

    def until(self, predicate, timeout=6.):
        stop = time.monotonic() + timeout
        while not predicate() and time.monotonic() < stop:
            self.spin(.02)
        assert predicate(), "timed out waiting for Point-LIO pipeline"

    def scan(self, seconds, side=0, points=((1., 0., 0.),), bad_frame=False):
        msg = PointCloud2()
        msg.header.stamp = stamp(seconds)
        msg.header.frame_id = "wrong" if bad_frame else ("left_mid360" if side == 0 else "right_mid360")
        msg.width, msg.height = len(points), 1
        msg.point_step, msg.row_step = 16, len(points) * 16
        msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z", "intensity"))]
        msg.data = b"".join(struct.pack("<ffff", *p, 42.) for p in points)
        self.pubs[side].publish(msg)
        return msg

    def imu(self, seconds, acc=(1., 2., 3.), gyro=(0., 0., .5)):
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp(seconds), "right_mid360"
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.orientation.z, msg.orientation.w = math.sin(.2), math.cos(.2)
        msg.linear_acceleration_covariance = [1., 0., 0., 0., 2., 0., 0., 0., 3.]
        self.imu_pub.publish(msg)

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        self.node.destroy_node()


@pytest.fixture
def harness_factory():
    harnesses = []
    def make(**kwargs):
        item = InputHarness.__new__(InputHarness)
        harnesses.append(item)
        item.__init__(**kwargs)
        return item
    yield make
    for item in harnesses:
        item.close()


def test_dual_snapshot_static_mounts_and_exact_stamp(harness_factory):
    h = harness_factory()
    h.scan(10., 0)
    h.scan(10., 1)
    h.until(lambda: bool(h.clouds))
    msg = h.clouds[-1]
    assert msg.header.frame_id == "pointlio_imu"
    assert msg.timebase == 10_000_000_000 and msg.header.stamp == stamp(10.)
    assert msg.point_num == 2
    assert (msg.points[0].x, msg.points[0].y, msg.points[0].z) == pytest.approx((.2, 1.4, .03))
    assert (msg.points[1].x, msg.points[1].y, msg.points[1].z) == pytest.approx((0., -1., .03))
    assert all(p.offset_time == 0 and p.tag == 0x10 and p.line == 0 for p in msg.points)


def test_different_scan_times_are_not_falsely_synchronized(harness_factory):
    h = harness_factory()
    h.scan(10., 0)
    h.scan(10.001, 1)
    h.spin(.15)
    assert not h.clouds
    h.scan(10., 1)
    h.until(lambda: len(h.clouds) == 1)
    assert h.clouds[0].header.stamp == stamp(10.)


def test_imu_si_rotation_covariance_and_no_absolute_orientation(harness_factory):
    h = harness_factory()
    h.imu(10.)
    h.until(lambda: bool(h.imus))
    msg = h.imus[-1]
    assert msg.header.stamp == stamp(10.) and msg.header.frame_id == "pointlio_imu"
    assert (msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z) == pytest.approx((2., -1., 3.))
    assert msg.linear_acceleration_covariance == pytest.approx([2., 0., 0., 0., 1., 0., 0., 0., 3.])
    assert msg.orientation_covariance[0] == -1. and msg.orientation.w == 1.


def test_invalid_frames_zero_stamps_and_nonfinite_points(harness_factory):
    h = harness_factory()
    h.scan(0., 0); h.scan(0., 1)
    h.scan(10., 0, bad_frame=True); h.scan(10., 1)
    h.spin(.15)
    assert not h.clouds
    h.scan(11., 0, points=((math.nan, 0., 0.), (1., 0., 0.)))
    h.scan(11., 1)
    h.until(lambda: bool(h.clouds))
    assert h.clouds[-1].point_num == 2


def test_adapter_does_not_subscribe_to_truth_or_controls(harness_factory):
    h = harness_factory()
    ns = h.node.get_namespace()
    topics = h.node.get_subscriber_names_and_types_by_node("sim_point_lio_input_bridge", ns)
    names = {topic for topic, _ in topics}
    assert not any("odom" in name or "cmd_vel" in name or name.endswith("/tf") for name in names)
    assert {f"{ns}/livox/{name}" for name in ("lidar_left", "lidar_right", "imu_right")} <= names


def test_real_point_lio_initializes_from_synthetic_snapshots(harness_factory):
    h = harness_factory(frontend=True)
    points = []
    for i in range(30):
        for j in range(30):
            x, y = -.9 + i * .06, -.9 + j * .06
            points.extend(((x, y, -.3), (2., x, y), (x, 2., y)))
    for frame in range(35):
        base = 1. + frame * .05
        for i in range(10):
            h.imu(base + i * .005, acc=(0., 0., 9.81), gyro=(0., 0., 0.))
        h.scan(base + .045, 0, points=points)
        h.scan(base + .045, 1, points=points)
        h.spin(.035)
    h.until(lambda: len(h.registered) >= 3 and len(h.odoms) >= 3)
    odom = h.odoms[-1]
    assert odom.header.frame_id == "pointlio_odom" and odom.child_frame_id == "pointlio_imu"
    pose = odom.pose.pose
    assert all(math.isfinite(v) for v in (pose.position.x, pose.position.y, pose.position.z,
               pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    assert math.hypot(pose.position.x, pose.position.y) < .15
    assert abs(pose.position.z) < .15
    assert h.registered[-1].header.frame_id == "pointlio_odom"
    assert h.registered[-1].width > 100


def test_sim_config_isolated_from_real_robot_defaults():
    import yaml
    root = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((root / "config/point_lio_snapshot.yaml").read_text())["/**"]["ros__parameters"]
    assert params["mapping"]["acc_norm"] == 9.81
    assert params["mapping"]["satu_acc"] == pytest.approx(5 * 9.81)
    assert params["mapping"]["satu_acc"] * .99 > 9.81
    assert params["mapping"]["extrinsic_T"] == [0., 0., 0.]
    assert not params["prior_pcd"]["enable"] and not params["publish"]["path_en"]
    assert params["odometry"]["output_imu_pose"]
