"""Black-box regression tests: no Gazebo/GUI, motors, or real-robot topics."""

import math
import os
import signal
import struct
import subprocess
import time
import uuid

import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


LEFT_MOUNT = (0.1, 0.2, 0.3, 0.0, 0.0, math.pi / 2)
RIGHT_MOUNT = (-0.1, -0.2, 0.3, 0.0, 0.0, -math.pi / 2)
SENSOR_POSE = (0., 0., 0.03, 0., 0., 0.)


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    )
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def inverse_point(point, pose):
    matrix = rotation(pose[3:])
    shifted = [point[i] - pose[i] for i in range(3)]
    return tuple(sum(matrix[j][i] * shifted[j] for j in range(3)) for i in range(3))


def set_pose(message, pose):
    message.position.x, message.position.y, message.position.z = pose[:3]
    r, p, y = (angle / 2 for angle in pose[3:])
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    )
    message.orientation.x = sr * cp * cy - cr * sp * sy
    message.orientation.y = cr * sp * cy + sr * cp * sy
    message.orientation.z = cr * cp * sy - sr * sp * cy
    message.orientation.w = cr * cp * cy + sr * sp * sy


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class BridgeHarness:
    def __init__(self, dual=False, zero=False, queue_size=20):
        namespace = "/cloud_test_" + uuid.uuid4().hex[:10]
        remaps = ["-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]
        self.node = rclpy.create_node(
            "probe", namespace=namespace, cli_args=["--ros-args", *remaps]
        )
        args = [
            os.environ["BRIDGE_EXECUTABLE"], "--ros-args", "-r", f"__ns:={namespace}",
            *remaps, "-p", f"dual_lidar:={str(dual).lower()}",
            "-p", f"zero_at_first_odometry:={str(zero).lower()}",
            "-p", f"cloud_queue_size:={queue_size}",
            "-p", "relay_cmd_vel:=false",
            "-p", f"left_lidar_pose:={list(LEFT_MOUNT)}",
            "-p", f"right_lidar_pose:={list(RIGHT_MOUNT)}",
        ]
        self.process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.clouds, self.odoms = [], []
        self.node.create_subscription(PointCloud2, "cloud_registered", self.clouds.append, 10)
        self.node.create_subscription(Odometry, "odometry/filtered", self.odoms.append, 10)
        self.odom_pub = self.node.create_publisher(Odometry, "chassis_odometry_gt", 10)
        self.pose_pub = self.node.create_publisher(Odometry, "chassis_odometry_3d_gt", 10)
        topics = ["livox/lidar_left", "livox/lidar_right"] if dual else ["livox/lidar"]
        self.cloud_pubs = [
            self.node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
            for topic in topics
        ]
        self.static = StaticTransformBroadcaster(self.node)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = "chassis", "test_lidar"
        mount.transform.translation.x = 0.1
        mount.transform.translation.z = 0.2
        mount.transform.rotation.w = 1.0
        self.static.sendTransform(mount)
        self.until(lambda: all(pub.get_subscription_count() for pub in self.cloud_pubs)
                   and self.odom_pub.get_subscription_count()
                   and self.pose_pub.get_subscription_count())
        # Discovery is asynchronous: repeat only the initialization message,
        # not the test scans, until both ends of the output topic are matched.
        initial_pose = (5., 2., 0., 0., 0., 0.) if zero else (0.,) * 6
        init_timer = self.node.create_timer(.05, lambda: self.nav_pose(9., initial_pose))
        try:
            self.until(lambda: bool(self.odoms))
        finally:
            self.node.destroy_timer(init_timer)
        self.spin_for(0.1)

    def until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert self.process.poll() is None, "bridge process exited unexpectedly"
            rclpy.spin_once(self.node, timeout_sec=0.02)
        assert predicate(), "timed out waiting for bridge"

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def nav_pose(self, seconds, pose):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp(seconds), "world"
        msg.child_frame_id = "red_standard_robot1/chassis"
        set_pose(msg.pose.pose, pose)
        self.odom_pub.publish(msg)

    def chassis_pose(self, seconds, pose):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp(seconds), "world"
        msg.child_frame_id = "red_standard_robot1/chassis"
        set_pose(msg.pose.pose, pose)
        self.pose_pub.publish(msg)
        self.spin_for(0.04)

    def cloud(self, seconds, point, side=0):
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp(seconds), "test_lidar"
        msg.height, msg.width, msg.point_step, msg.row_step = 1, 1, 16, 16
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(("x", "y", "z", "intensity"))]
        msg.data = struct.pack("<ffff", *point, 42.)
        self.cloud_pubs[side].publish(msg)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.node.destroy_node()


@pytest.fixture
def bridge_factory():
    bridges = []

    def make(**kwargs):
        harness = BridgeHarness.__new__(BridgeHarness)
        bridges.append(harness)
        harness.__init__(**kwargs)
        return harness

    yield make
    for harness in bridges:
        harness.close()


def point_at_world(world_point, chassis_pose, mount=(0.1, 0., 0.2, 0., 0., 0.)):
    return inverse_point(inverse_point(inverse_point(world_point, chassis_pose), mount), SENSOR_POSE)


def assert_cloud(cloud, seconds, points):
    assert cloud.header.frame_id == "map"
    assert cloud.header.stamp == stamp(seconds)
    assert cloud.width == len(points)
    values = list(struct.iter_unpack("<ffff", bytes(cloud.data)))
    for actual, expected in zip(values, points):
        assert actual[:3] == pytest.approx(expected, abs=2e-5)
        assert actual[3] == 42.


def test_delayed_ground_cloud_does_not_use_latest_tilt(bridge_factory):
    bridge = bridge_factory()
    old_pose, new_pose = (0., 0., 1., 0., 0., 0.), (0.2, 0., 1.2, 0.1, 0.4, 0.2)
    bridge.chassis_pose(10., old_pose)
    bridge.chassis_pose(11., new_pose)
    bridge.nav_pose(11., (0.2, 0., 0., 0., 0., 0.2))
    bridge.cloud(10., point_at_world((2., 0., 0.), old_pose))
    bridge.until(lambda: len(bridge.clouds) == 1)
    assert_cloud(bridge.clouds[0], 10., [(2., 0., 0.)])
    bridge.until(lambda: bridge.odoms[-1].header.stamp == stamp(11.))
    assert bridge.odoms[-1].pose.pose.position.z == 0.  # navigation stays planar


def test_cloud_waits_for_its_own_pose(bridge_factory):
    bridge = bridge_factory()
    pose = (1., 2., 0.4, 0.1, 0.3, 0.2)
    bridge.cloud(12., point_at_world((3., 2., 0.), pose))
    bridge.spin_for(0.1)
    assert not bridge.clouds
    bridge.chassis_pose(12., pose)
    bridge.until(lambda: len(bridge.clouds) == 1)
    assert_cloud(bridge.clouds[0], 12., [(3., 2., 0.)])


def test_dual_lidars_register_at_different_acquisition_stamps(bridge_factory):
    bridge = bridge_factory(dual=True)
    left_pose, right_pose = (0., 0., .3, .1, .2, .3), (.2, .1, .4, .2, .4, .5)
    landmark = (2., 1., 0.)
    bridge.chassis_pose(10., left_pose)
    bridge.chassis_pose(10.05, right_pose)
    bridge.cloud(10., point_at_world(landmark, left_pose, LEFT_MOUNT), side=0)
    bridge.cloud(10.05, point_at_world(landmark, right_pose, RIGHT_MOUNT), side=1)
    bridge.until(lambda: len(bridge.clouds) == 1)
    assert_cloud(bridge.clouds[0], 10.05, [landmark, landmark])


def test_pose_interpolation_uses_cloud_stamp(bridge_factory):
    bridge = bridge_factory()
    bridge.chassis_pose(10., (0., 0., .2, 0., 0., 0.))
    bridge.chassis_pose(10.1, (.2, 0., .4, 0., .4, 0.))
    halfway = (.1, 0., .3, 0., .2, 0.)
    landmark = (1., 0., 0.)
    bridge.cloud(10.05, point_at_world(landmark, halfway))
    bridge.until(lambda: len(bridge.clouds) == 1)
    assert_cloud(bridge.clouds[0], 10.05, [landmark])


def test_missing_pose_expires_without_latest_fallback(bridge_factory):
    bridge = bridge_factory()
    bridge.chassis_pose(10., (0.,) * 6)
    bridge.cloud(20., (1., 0., 0.))
    bridge.spin_for(0.65)
    bridge.chassis_pose(20., (0.,) * 6)
    bridge.spin_for(.1)
    assert not bridge.clouds
    bridge.cloud(20., (.9, 0., -.23))
    bridge.until(lambda: len(bridge.clouds) == 1)
    assert_cloud(bridge.clouds[0], 20., [(1., 0., 0.)])


def test_zero_stamp_is_not_treated_as_latest(bridge_factory):
    bridge = bridge_factory()
    bridge.chassis_pose(10., (0.,) * 6)
    bridge.cloud(0., (1., 0., 0.))
    bridge.spin_for(.1)
    assert not bridge.clouds


def test_map_origin_matches_planar_navigation_origin(bridge_factory):
    bridge = bridge_factory(zero=True)
    pose = (5., 2., .3, .1, .2, .3)
    bridge.chassis_pose(10., pose)
    bridge.cloud(10., point_at_world((6., 2., 0.), pose))
    bridge.until(lambda: len(bridge.clouds) == 1)
    assert_cloud(bridge.clouds[0], 10., [(1., 0., 0.)])


def test_queue_overflow_drops_oldest_cloud(bridge_factory):
    bridge = bridge_factory(queue_size=2)
    bridge.chassis_pose(10., (0.,) * 6)
    for seconds in (10.02, 10.03, 10.04):
        bridge.cloud(seconds, (.9, 0., -.23))
        bridge.spin_for(.03)
    assert not bridge.clouds
    bridge.chassis_pose(10.1, (0.,) * 6)
    bridge.until(lambda: len(bridge.clouds) == 2)
    assert_cloud(bridge.clouds[0], 10.03, [(1., 0., 0.)])
    assert_cloud(bridge.clouds[1], 10.04, [(1., 0., 0.)])
