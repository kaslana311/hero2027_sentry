#!/usr/bin/env python3
"""Read-only observer. Truth NEVER enters the backend or any estimator."""
from bisect import bisect_left
from collections import deque
import copy
import json
import math
from urllib.parse import unquote

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def parse_state(text):
    return {k: unquote(v) for entry in text.split(';') if '=' in entry
            for k, v in [entry.split('=', 1)]}


def interpolate_position(samples, stamp, max_gap_ns=150000000):
    times = [x[0] for x in samples]
    index = bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        return samples[index][1]
    if index == 0 or index == len(times) or times[index] - times[index-1] > max_gap_ns:
        return None  # Never compare different acquisition times via a latest-pose fallback.
    ratio = (stamp-times[index-1]) / (times[index]-times[index-1])
    return tuple(a + ratio*(b-a) for a, b in zip(samples[index-1][1], samples[index][1]))


class Monitor(Node):
    def __init__(self):
        super().__init__('backend_shadow_monitor')
        self.truth = deque(maxlen=4000)  # Simulator truth may run at 1 kHz; retain four seconds.
        self.pending = deque(maxlen=100)
        self.path = deque(maxlen=2000)
        self.state = {}
        self.last_stamp = 0
        self.last_log_ns = 0
        self.last_error = None
        self.last_pose = None
        self.last_clock = 0
        self.last_pose_received_ns = 0
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'comparison/backend_path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, 'comparison/markers', durable)
        self.error_pub = self.create_publisher(String, 'comparison/error', 10)
        self.create_subscription(String, 'backend/localization_state', self.on_state, durable)
        self.create_subscription(PoseWithCovarianceStamped, 'backend/localization_pose', self.on_pose, 20)
        self.create_subscription(Odometry, self.declare_parameter('truth_topic',
                                 '/red_standard_robot1/odometry/truth_3d').value,
                                 self.on_truth, qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)

    def on_state(self, msg):
        self.state = parse_state(msg.data)

    def on_truth(self, msg):
        stamp = ns(msg.header.stamp)
        if msg.header.frame_id != 'map' or stamp <= 0:
            return
        if self.truth and stamp <= self.truth[-1][0]:
            return
        p = msg.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.truth.append((stamp, (p.x, p.y, p.z)))

    def on_pose(self, msg):
        stamp = ns(msg.header.stamp)
        if msg.header.frame_id != 'map' or stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.last_pose = msg
        self.last_pose_received_ns = self.get_clock().now().nanoseconds
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        if self.state.get('pose_trusted') != 'true':
            return  # Frozen/untrusted outputs are not credited as successful localization.
        point = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self.path.append(point)
        self.pending.append((stamp, (p.x, p.y, p.z)))

    def tick(self):
        now = self.get_clock().now()
        if self.last_clock and now.nanoseconds < self.last_clock - 500000000:
            self.truth.clear()
            self.pending.clear()
            self.path.clear()
            self.last_stamp = 0
            self.last_error = None
            self.last_pose = None
            self.state = {'status': 'RESTART_REQUIRED', 'pose_trusted': 'false'}
            self.get_logger().error('Simulation time reset: restart the complete launch, including backend.')
        self.last_clock = now.nanoseconds
        while self.pending:
            stamp, pose = self.pending[0]
            reference = interpolate_position(self.truth, stamp)
            if reference is None:
                if self.truth and stamp >= self.truth[-1][0] and now.nanoseconds-stamp < 1000000000:
                    break
                self.pending.popleft()
                continue
            self.pending.popleft()
            delta = tuple(a-b for a, b in zip(pose, reference))
            self.last_error = {'stamp_ns': stamp, 'dx': delta[0], 'dy': delta[1], 'dz': delta[2],
                               'xy_m': math.hypot(*delta[:2]), 'xyz_m': math.sqrt(sum(v*v for v in delta)),
                               'status': self.state.get('status', 'UNKNOWN'), 'shadow_only': True}
            self.error_pub.publish(String(data=json.dumps(self.last_error)))
        if self.path:
            self.path_pub.publish(Path(header=self.path[-1].header, poses=list(self.path)))
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = 'map', now.to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'backend_shadow', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.orientation.w = 1.0
        if self.last_pose is not None:
            marker.pose.position = copy.deepcopy(self.last_pose.pose.pose.position)
        marker.pose.position.z += 1.1
        marker.scale.z, marker.color.a = 0.25, 1.0
        fresh = self.last_pose_received_ns > 0 and now.nanoseconds-self.last_pose_received_ns < 2000000000
        trusted = self.state.get('pose_trusted') == 'true' and fresh
        marker.color.r, marker.color.g, marker.color.b = ((0.0, 1.0, 1.0) if trusted else (1.0, 0.3, 0.1))
        marker.text = 'BACKEND SHADOW (NOT NAVIGATION)\n' + self.state.get('status', 'WAITING_FOR_BACKEND')
        if self.last_pose is not None and not fresh:
            marker.text += ' | STALE POSE'
        if self.last_error is not None:
            marker.text += f"\nlast trusted XY error: {self.last_error['xy_m']:.3f} m"
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        if now.nanoseconds-self.last_log_ns >= 5000000000:
            self.last_log_ns = now.nanoseconds
            self.get_logger().info(marker.text.replace('\n', ' | '))


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
