"""Actual C++ adapters and TF, wall-clock only, in an isolated ROS domain.

No chassis serial port, Livox driver, Gazebo, motor or navigation server starts.
"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def quaternion(rpy):
    r, p, y = (v / 2 for v in rpy)
    return (math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y),
            math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y),
            math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y),
            math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y))


def product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, relay=True):
        self.ns = '/real_probe_' + uuid.uuid4().hex[:8]
        self.node = rclpy.create_node('probe', namespace=self.ns)
        self.odoms, self.globals, self.commands, self.clouds, self.tfs, self.states = [], [], [], [], [], []
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose = self.node.create_publisher(Odometry, 'pointlio/odometry', 10)
        self.raw = self.node.create_publisher(PointCloud2, 'pointlio/cloud_registered', 10)
        self.cmd = self.node.create_publisher(Twist, 'cmd_vel_nav2_result', 10)
        self.node.create_subscription(Odometry, 'odometry/filtered', self.odoms.append, 10)
        self.node.create_subscription(Odometry, 'base_link_in_map_odom', self.globals.append, 10)
        self.node.create_subscription(Twist, 'cmd_vel', self.commands.append, 10)
        self.node.create_subscription(PointCloud2, 'pointlio/cloud_registered_map', self.clouds.append, 10)
        self.node.create_subscription(TFMessage, 'tf', lambda m: self.tfs.extend(m.transforms), 100)
        self.node.create_subscription(String, 'pointlio/navigation/status',
                                      lambda m: self.states.append(json.loads(m.data)), durable)
        bridge = os.environ['REAL_BRIDGE_EXECUTABLE']
        cloud_bridge = os.environ.get('REAL_CLOUD_EXECUTABLE') or str(Path(get_package_prefix('hero_sim_bringup')) /
            'lib/hero_sim_bringup/sim_point_lio_cloud_bridge')
        self.processes = []
        for exe, namespace, args in (
            (bridge, self.ns, ['-p', f'relay_cmd_vel:={str(relay).lower()}',
                              '-p', 'initial_base_pose:=[2.0,-3.0,0.15,0.7]']),
            (cloud_bridge, self.ns+'/pointlio', ['-p', 'use_navigation_tf:=true']),
        ):
            self.processes.append(subprocess.Popen([exe, '--ros-args', '-r', '__ns:='+namespace,
                '-r', '/tf:='+self.ns+'/tf', '-r', '/tf_static:='+self.ns+'/tf_static',
                '-p', 'use_sim_time:=false', *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        try:
            self.until(lambda: self.pose.get_subscription_count() and self.raw.get_subscription_count()
                       and self.cmd.get_subscription_count())
        except BaseException:
            self.close()
            raise

    def until(self, condition, seconds=6.):
        limit = time.monotonic()+seconds
        while not condition() and time.monotonic() < limit:
            assert all(p.poll() is None for p in self.processes), 'real adapter exited'
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert condition(), 'timed out waiting for real adapter'

    def pump(self, seconds=.12):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.01)

    def send(self):
        # Arbitrary odom gauge and non-identity mount. This is not a simulation
        # truth seed: the adapter receives only raw LIO pose + registered rays.
        base = np.array([10., 20., 1.])
        yaw = .4
        R = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                      [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        sensor = base + R @ np.array([.062208, -.207112, 0.])
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'pointlio_imu'
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p.x, p.y, p.z = map(float, sensor)
        q.x, q.y, q.z, q.w = product(quaternion((0., 0., yaw)), quaternion((0., .614120, -math.pi/2)))
        self.pose.publish(msg)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = msg.header.stamp, 'odom'
        cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = struct.pack('<fff', *base)
        cloud.is_dense = True
        self.raw.publish(cloud)
        self.pump()

    def ready(self):
        for _ in range(5):
            self.send()
        self.until(lambda: self.states and self.states[-1]['state'] == 'ready' and self.clouds and self.globals)

    def set(self, name, value):
        client = self.node.create_client(SetParametersAtomically,
                                         'real_point_lio_navigation_bridge/set_parameters_atomically')
        self.until(client.service_is_ready)
        future = client.call_async(SetParametersAtomically.Request(
            parameters=[Parameter(name, value=value).to_parameter_msg()]))
        self.until(future.done)
        result = future.result().result
        self.node.destroy_client(client)
        return result

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.node.destroy_node()


def xyz(msg):
    p = msg.pose.pose.position
    return p.x, p.y, p.z


def test_real_nonidentity_mount_initial_placement_and_offset_move_scan_with_pose():
    h = Harness()
    try:
        h.ready()
        assert xyz(h.odoms[-1]) == pytest.approx((10., 20., 1.), abs=1e-7)
        assert xyz(h.globals[-1]) == pytest.approx((2., -3., .15), abs=1e-7)
        assert struct.unpack_from('<fff', h.clouds[-1].data) == pytest.approx((2., -3., .15), abs=1e-5)
        assert h.set('map_odom_offset_y', .5).successful
        h.pump(.2)
        h.send()
        h.until(lambda: h.clouds and abs(struct.unpack_from('<fff', h.clouds[-1].data)[1]+2.5) < 1e-5)
        assert xyz(h.globals[-1]) == pytest.approx((2., -2.5, .15), abs=1e-7)
        assert xyz(h.odoms[-1]) == pytest.approx((10., 20., 1.), abs=1e-7)
        assert not h.set('base_to_imu_rpy', [0., 0., 0.]).successful
        assert h.set('map_odom_offset_x', float('nan')).successful is False
    finally:
        h.close()


@pytest.mark.parametrize('relay', [False, True])
def test_chassis_commands_are_not_double_rotated_and_stale_input_stops_output(relay):
    h = Harness(relay)
    try:
        h.ready()
        msg = Twist()
        msg.linear.x, msg.linear.y = .4, -.2
        for _ in range(3):
            h.cmd.publish(msg)
            h.pump(.03)
        if not relay:
            assert not h.commands
        else:
            h.until(lambda: any(abs(m.linear.x-.4) < 1e-9 for m in h.commands))
            nonzero = next(m for m in h.commands if m.linear.x)
            assert (nonzero.linear.x, nonzero.linear.y) == pytest.approx((.4, -.2))
            h.pump(.6)
            assert h.commands[-1].linear.x == h.commands[-1].linear.y == 0.
    finally:
        h.close()


def test_dual_merge_really_reads_both_livox_streams_with_native_point_times():
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    ns = '/dual_merge_probe_' + uuid.uuid4().hex[:8]
    node = rclpy.create_node('probe', namespace=ns)
    results = []
    node.create_subscription(CustomMsg, 'fused', results.append, 10)
    first = node.create_publisher(CustomMsg, 'left', 10)
    second = node.create_publisher(CustomMsg, 'right', 10)
    executable = str(Path(get_package_prefix('lidar_merge'))/'lib/lidar_merge/lidar_merge_dual_node')
    process = subprocess.Popen([executable, '--ros-args', '-r', '__ns:='+ns,
        '-r', '/livox/fused_cloud:='+ns+'/fused',
        '-p', 'lidar1_topic:='+ns+'/left', '-p', 'lidar2_topic:='+ns+'/right',
        '-p', 'lidar1_to_lidar2_x:=1.0', '-p', 'lidar1_to_lidar2_yaw:=1.5707963267948966'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        limit = time.monotonic()+6.
        while not (first.get_subscription_count() and second.get_subscription_count()) and time.monotonic() < limit:
            assert process.poll() is None
            rclpy.spin_once(node, timeout_sec=.02)
        assert first.get_subscription_count() and second.get_subscription_count()
        for _ in range(8):
            stamp = node.get_clock().now()
            a, b = CustomMsg(), CustomMsg()
            for msg in (a, b):
                msg.header.stamp = stamp.to_msg()
                msg.header.frame_id = 'livox_frame'
                msg.timebase = stamp.nanoseconds
                msg.point_num = 1
            a.points = [CustomPoint(x=2., y=0., z=0., offset_time=1000000, reflectivity=11, tag=0, line=0)]
            b.points = [CustomPoint(x=4., y=0., z=0., offset_time=2000000, reflectivity=22, tag=0, line=0)]
            first.publish(a)
            second.publish(b)
            end = time.monotonic()+.06
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
        assert results
        fused = results[-1]
        assert fused.point_num == len(fused.points) == 2
        assert [p.reflectivity for p in fused.points] == [11, 22]
        assert [p.offset_time for p in fused.points] == [1000000, 2000000]
        assert (fused.points[0].x, fused.points[0].y, fused.points[0].z) == pytest.approx((1., 2., 0.), abs=1e-6)
        assert (fused.points[1].x, fused.points[1].y, fused.points[1].z) == pytest.approx((4., 0., 0.), abs=1e-6)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=4)
        node.destroy_node()
