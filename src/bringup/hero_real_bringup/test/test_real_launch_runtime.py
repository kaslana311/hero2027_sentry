"""Full real entry, synthetic LIO outputs, isolated localhost DDS; no devices."""
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
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from nav2_msgs.srv import ManageLifecycleNodes


@pytest.mark.parametrize('compatibility_entry', [False, True])
def test_full_entry_keeps_split_stuck_switches_and_computes_shadow_backend(compatibility_entry):
    rclpy.init()
    robot = 'real_stack_probe_' + uuid.uuid4().hex[:6]
    node = rclpy.create_node('observer', namespace='/'+robot)
    root = Path(os.environ.get('ROS_LOG_DIR', '/tmp/hero_real_smoke_logs'))
    root.mkdir(parents=True, exist_ok=True)
    output = (root/('compat_real_launch.log' if compatibility_entry else 'full_real_launch.log')).open('w')
    entry = ['hero_sim_bringup', 'navigation_point_lio_backend_shadow_sim.launch.py', 'data_source:=real'] \
        if compatibility_entry else ['hero_real_bringup', 'navigation_point_lio_backend_shadow_real.launch.py']
    process = subprocess.Popen([
        'ros2', 'launch', *entry,
        'namespace:='+robot, 'field:=rmuc_2027', 'costmap_mode:=split', 'lidar_mode:=dual',
        'start_lidar_driver:=false', 'start_lidar_merge:=false', 'start_serial:=false',
        'start_zenoh:=false', 'start_vision_bridge:=false', 'start_decision:=false',
        'relay_cmd_vel:=false', 'use_rviz:=false', 'backend_rviz:=false',
        'point_lio_rviz:=false', 'stuck_cold_start:=true', 'split_local_refinement:=true',
    ], stdout=output, stderr=subprocess.STDOUT)

    def until(condition, timeout=20.):
        limit = time.monotonic()+timeout
        while not condition() and time.monotonic() < limit:
            assert process.poll() is None, 'launch exited; inspect '+str(root/'full_real_launch.log')
            rclpy.spin_once(node, timeout_sec=.03)
        assert condition(), 'launch check timed out; inspect '+str(root/'full_real_launch.log')

    def params(name, keys):
        client = node.create_client(GetParameters, name+'/get_parameters')
        until(client.service_is_ready)
        deadline = time.monotonic()+20.
        values = []
        while len(values) != len(keys) and time.monotonic() < deadline:
            future = client.call_async(GetParameters.Request(names=keys))
            until(future.done)
            values = future.result().values
            if len(values) != len(keys):
                time.sleep(.05)  # Do not flood a lifecycle node before its plugins configure.
                rclpy.spin_once(node, timeout_sec=.05)
        assert len(values) == len(keys), 'parameters not ready: '+name
        node.destroy_client(client)
        return values

    poses, states, comparison = [], [], []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, 'z_backend/backend/localization_state',
                             lambda m: states.append(m.data), durable)
    node.create_subscription(String, 'z_backend/comparison/error',
                             lambda m: comparison.append(json.loads(m.data)), 10)
    from geometry_msgs.msg import PoseWithCovarianceStamped
    node.create_subscription(PoseWithCovarianceStamped, 'z_backend/backend/localization_pose', poses.append, 20)
    odom_pub = node.create_publisher(Odometry, 'pointlio/odometry', 10)
    cloud_pub = node.create_publisher(PointCloud2, 'pointlio/cloud_registered', 10)
    points = [(x*.15, y*.15, .0) for x in range(-20, 21) for y in range(-20, 21)]
    payload = b''.join(struct.pack('<fff', *p) for p in points)
    counter = 0

    def publish_scan():
        nonlocal counter
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'pointlio_imu'
        msg.pose.pose.position.x = counter*.01
        msg.pose.pose.orientation.w = 1.
        counter += 1
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width, cloud.point_step = 1, len(points), 12
        cloud.row_step = len(payload)
        cloud.fields = [PointField(name=n, offset=4*k, datatype=PointField.FLOAT32, count=1)
                        for k, n in enumerate(('x', 'y', 'z'))]
        cloud.data, cloud.is_dense = payload, True
        odom_pub.publish(msg)
        cloud_pub.publish(cloud)

    # Costmap lifecycle configuration waits for the main TF. Keep publishing
    # scan-end pairs while waiting for plugin parameter services to configure.
    timer = node.create_timer(.05, publish_scan)
    try:
        backend = params('z_backend/backend', ['use_sim_time', 'prior_map.enabled', 'common.cloud_topic'])
        assert not backend[0].bool_value and not backend[1].bool_value
        assert backend[2].string_value == '/'+robot+'/pointlio/cloud_registered'
        smooth = params('smoother_server', ['use_sim_time', 'minco_smoother.stuck_state_topic',
                                           'minco_smoother.safe_distance', 'minco_smoother.project_esdf_gradient'])
        assert not smooth[0].bool_value
        assert smooth[1].string_value == 'navigation/motion_stalled'
        assert smooth[2].double_value == .3 and smooth[3].bool_value
        planner = params('planner_server', ['GridBased.plugin', 'GridBased.local_refinement_enabled'])
        assert planner[0].string_value == 'hero_static_prior_planner/StaticPriorPlanner'
        assert planner[1].bool_value
        until(lambda: len(poses) > 5 and comparison and counter >= 70, timeout=12.)
        assert any('NORMAL' in state for state in states)
        assert all(m['reference'] == 'frontend' and not m['absolute_accuracy_measured'] for m in comparison)
        # The last managed node becoming active also means the BT was activated.
        state_client = node.create_client(GetState, 'velocity_smoother/get_state')
        until(state_client.service_is_ready)
        deadline = time.monotonic()+20.
        state = 0
        while state != 3 and time.monotonic() < deadline:
            future = state_client.call_async(GetState.Request())
            until(future.done)
            state = future.result().current_state.id
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_client(state_client)
        assert state == 3, 'Nav2 did not fully activate'
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        assert 'sim_ground_truth_bridge' not in names
        assert 'sim_point_lio_input_bridge' not in names
        assert 'chassis_node' not in names
        assert 'livox_lidar_publisher' not in names
    finally:
        if process.poll() is None:
            # Clean up managed costmap worker threads while their TF source is
            # still publishing. This changes only the test, not the user launch.
            client = node.create_client(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
            if client.wait_for_service(timeout_sec=1.):
                request = ManageLifecycleNodes.Request(command=4)
                future = client.call_async(request)
                limit = time.monotonic()+5.
                while not future.done() and time.monotonic() < limit:
                    rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_client(client)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        output.close()
        node.destroy_node()
        rclpy.shutdown()
