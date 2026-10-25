#!/usr/bin/env python3
"""Isolated synthetic I/O/correction probe; NOT a Gazebo or rosbag acceptance test."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp(seconds):
    result = Clock().clock
    total = round(seconds*1e9)
    result.sec, result.nanosec = divmod(total, 1000000000)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prior-map', required=True)
    parser.add_argument('--evidence', required=True)
    args = parser.parse_args()
    root, evidence = Path(args.prior_map), Path(args.evidence)
    evidence.mkdir(parents=True, exist_ok=True)
    rows = [line.split() for line in (root/'keyframes.txt').read_text().splitlines()
            if line and not line.startswith('#')]
    first = np.array([float(v) for v in rows[0][2:5]])
    _, binary = (root/'clouds/000000.pcd').read_bytes().split(b'DATA binary\n', 1)
    scene = np.frombuffer(binary, dtype='<f4').reshape(-1, 4)[:, :3].astype(float)+first
    rclpy.init()
    node = rclpy.create_node('shadow_contract_observer')
    robot = 'shadow_contract_robot'
    namespace = f'/{robot}/z_backend'
    states, poses, global_tfs, motors, nav_poses = [], [], [], [], []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, namespace+'/backend/localization_state',
                             lambda msg: states.append(msg.data), durable)
    node.create_subscription(PoseWithCovarianceStamped, namespace+'/backend/localization_pose',
                             poses.append, 100)
    node.create_subscription(TFMessage, '/tf', global_tfs.append, 100)
    node.create_subscription(Twist, f'/{robot}/cmd_vel', motors.append, 10)
    node.create_subscription(Odometry, f'/{robot}/odometry/filtered', nav_poses.append, 10)
    odom_pub = node.create_publisher(Odometry, f'/{robot}/pointlio/odometry', 50)
    cloud_pub = node.create_publisher(PointCloud2, f'/{robot}/pointlio/cloud_registered', 50)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    out = (evidence/'runtime_backend.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'hero_localization_backend', 'point_lio_backend_shadow.launch.py',
        f'namespace:={robot}', f'prior_map:={root}', 'backend_rviz:=false'
    ], stdout=out, stderr=subprocess.STDOUT)
    metrics = {'test': 'synthetic scan-end I/O + injected odometry drift', 'gazebo_acceptance': False}
    try:
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if process.poll() is not None:
                raise AssertionError('Backend launch exited; inspect runtime_backend.log')
            if odom_pub.get_subscription_count() >= 2 and cloud_pub.get_subscription_count() >= 1:
                break
        else:
            raise AssertionError('Backend input subscriptions were not created')
        backend_subscriptions = node.get_subscriber_names_and_types_by_node('backend', namespace)
        metrics['backend_subscriptions'] = [name for name, _ in backend_subscriptions]
        assert not any('truth' in name for name, _ in backend_subscriptions)
        metrics['input_point_count'] = len(scene)
        endpoint = 0.
        # Three seconds to initialize, then 12 seconds of gradual odometry bias.
        for sequence in range(300):
            tick = time.monotonic()
            seconds = 10.+sequence*.05
            phase = max(0., (sequence-60)/239.)
            endpoint = .2*phase
            drift = .3*phase
            sensor = np.array([endpoint, -.167, .197])
            reported = np.array([endpoint+drift, 0., 0.])
            # Input cloud is in drifting odom, NOT already corrected map coordinates.
            cloud_points = np.empty((len(scene), 4), dtype='<f4')
            cloud_points[:, :3], cloud_points[:, 3] = scene-sensor+reported, 1.
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp(seconds), 'pointlio_odom'
            odom.child_frame_id, odom.pose.pose.orientation.w = 'pointlio_imu', 1.
            odom.pose.pose.position.x = float(reported[0])
            cloud = PointCloud2()
            cloud.header = odom.header
            cloud.height, cloud.width, cloud.point_step = 1, len(scene), 16
            cloud.row_step, cloud.is_dense = len(scene)*16, True
            cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(('x', 'y', 'z', 'intensity'))]
            cloud.data = cloud_points.tobytes()
            clock_pub.publish(Clock(clock=stamp(seconds)))
            odom_pub.publish(odom)
            cloud_pub.publish(cloud)
            while time.monotonic()-tick < .05:
                rclpy.spin_once(node, timeout_sec=.005)
            assert process.poll() is None
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.02)
        assert poses, 'No backend global pose output'
        trusted = [s for s in states if 'pose_trusted=true' in s]
        assert trusted, 'Prior registration never became trusted'
        # Align expected true motion to the actual stamp of the received output.
        last = poses[-1]
        seconds = last.header.stamp.sec+last.header.stamp.nanosec*1e-9
        phase = min(1., max(0., ((seconds-10.)/.05-60)/239.))
        p = last.pose.pose.position
        expected = (.2*phase, 0., .076)
        error = float(np.linalg.norm(np.array([p.x, p.y])-np.array(expected[:2])))
        metrics.update(states_seen=sorted({s.split(';')[0] for s in states}),
                       last_state=states[-1], received_poses=len(poses), final_stamp=seconds,
                       expected_body_xyz=expected, backend_body_xyz=(p.x, p.y, p.z),
                       injected_raw_xy_error_m=.3*phase, backend_xy_error_m=error,
                       global_tf_messages=len(global_tfs), motor_messages=len(motors),
                       navigation_pose_messages=len(nav_poses))
        assert phase > .95, 'Backend output stopped before drift injection completed'
        assert error < .12, f'Backend did not substantially reduce injected drift: {error:.3f} m'
        assert not global_tfs and not motors and not nav_poses, 'Shadow leaked into navigation'
        metrics['passed'] = True
    except BaseException as exc:
        metrics['passed'] = False
        metrics['error'] = repr(exc)
        raise
    finally:
        (evidence/'runtime_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
        print(json.dumps(metrics, indent=2), flush=True)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
        out.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
