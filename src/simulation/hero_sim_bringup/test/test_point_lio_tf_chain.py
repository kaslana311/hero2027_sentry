"""Actual adapters + TF2 in an isolated ROS domain, not a Gazebo acceptance run."""
import copy
import importlib.util
import math
from pathlib import Path
import struct

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from test_cloud_registration import stamp
from test_point_lio_cloud import CloudHarness, cloud
from test_point_lio_evaluation import EvaluationHarness, xyz
from test_point_lio_navigation import NavigationHarness


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def correction(seconds, translation=(0., 0., 0.), yaw=0.):
    m = TransformStamped()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(seconds), "map", "odom"
    t, q = m.transform.translation, m.transform.rotation
    t.x, t.y, t.z = translation
    q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return m


def lookup(h, seconds, child="base_link", parent="map"):
    # Feed the actual production TF messages into TF2, rather than reconstructing
    # an expected path from the adapter's pose messages or reimplementing its math.
    buffer = Buffer()
    for transform in h.static_tfs:
        buffer.set_transform_static(transform, "production adapter")
    for transform in h.tfs:
        buffer.set_transform(transform, "production adapter")
    return buffer.lookup_transform(parent, child, Time.from_msg(stamp(seconds)))


def set_parameters(h, **values):
    client = h.node.create_client(SetParametersAtomically,
        "sim_point_lio_navigation_bridge/set_parameters_atomically")
    h.until(client.service_is_ready)
    request = SetParametersAtomically.Request(parameters=[
        Parameter(name=name, value=value).to_parameter_msg() for name, value in values.items()])
    future = client.call_async(request)
    h.until(future.done)
    result = future.result().result
    h.node.destroy_client(client)
    return result


@pytest.mark.parametrize("offset", [(0.5, 0., 0.), (0., 0.5, 0.), (0., 0., math.pi / 2)])
def test_global_correction_moves_cloud_and_robot_together_without_changing_local_velocity(offset):
    h, c = NavigationHarness.__new__(NavigationHarness), CloudHarness.__new__(CloudHarness)
    try:
        h.__init__()
        h.ready()
        c.__init__(namespace=h.namespace + "/pointlio", navigation_tf=True, tf_namespace=h.namespace)
        c.until(lambda: c.tf_pub.get_subscription_count() >= 2)
        # This probe intentionally joins after startup. Dynamic TF is volatile:
        # use a NEW sample, not one older than the listener's first TF sample.
        h.clock(10.3)
        h.pose(10.3, (.02, 0., 0., 0., 0., 0.))
        c.pub.publish(cloud(10.3, ((.02, 0., 0., 42.),), frame="odom"))
        c.until(lambda: c.clouds)
        assert set_parameters(h, map_odom_offset_x=offset[0], map_odom_offset_y=offset[1],
                              map_odom_offset_yaw=offset[2]).successful
        # Both TF brackets now carry the new correction; no interpolation across
        # the deliberately discontinuous user correction is used by this assertion.
        h.clock(10.4)
        h.clock(10.5)
        h.pose(10.45, (.02, 0., 0., 0., 0., 0.))
        c.pub.publish(cloud(10.45, ((.02, 0., 0., 42.),), frame="odom"))
        c.until(lambda: c.clouds[-1].header.stamp == stamp(10.45))
        h.until(lambda: h.odoms[-1].header.stamp == stamp(10.45)
                and any(t.child_frame_id == "base_link" and t.header.stamp == stamp(10.45)
                        for t in h.tfs))
        x, y, yaw = offset
        expected = (x + 5. * math.cos(yaw) - 2.02 * math.sin(yaw),
                    y + 5. * math.sin(yaw) + 2.02 * math.cos(yaw), .3)
        registered = c.clouds[-1]
        assert struct.unpack_from("<fff", registered.data) == pytest.approx(expected, abs=1e-6)
        world_body = lookup(h, 10.45).transform.translation
        assert (world_body.x, world_body.y, world_body.z) == pytest.approx(expected, abs=1e-8)
        local = h.odoms[-1]
        assert local.header.frame_id == "odom" and local.header.stamp == registered.header.stamp
        assert xyz(local.pose) == pytest.approx((.02, 0., 0.), abs=1e-8)
        velocity = local.twist.twist
        assert (velocity.linear.x, velocity.linear.y, velocity.linear.z,
                velocity.angular.x, velocity.angular.y, velocity.angular.z) == pytest.approx((0.,) * 6)
        # The frontend's virtual sensor frame is a rigid child of the SAME body.
        body_to_imu = next(t for t in h.static_tfs if t.child_frame_id == "pointlio_imu")
        assert body_to_imu.header.frame_id == "base_link"
        assert not any(t.child_frame_id == "odom" for t in h.static_tfs)
        assert all(s["state"] != "restart_required" for s in h.statuses)
    finally:
        c.close()
        h.close()


def test_main_sensor_tf_matches_native_imu_pose_with_nonzero_pitch_roll_and_lever_arm():
    h = NavigationHarness.__new__(NavigationHarness)
    try:
        h.__init__()
        h.align()
        native = h.pose(10.1, (1., 2., .2, .1, .2, .3))
        h.until(lambda: any(t.child_frame_id == "base_link" for t in h.tfs))
        sensor = lookup(h, 10.1, child="pointlio_imu", parent="odom").transform
        expected = native.pose.pose
        assert (sensor.translation.x, sensor.translation.y, sensor.translation.z) == pytest.approx(
            (expected.position.x, expected.position.y, expected.position.z), abs=1e-8)
        actual_q, expected_q = sensor.rotation, expected.orientation
        dot = sum(getattr(actual_q, axis) * getattr(expected_q, axis) for axis in ("x", "y", "z", "w"))
        assert abs(dot) == pytest.approx(1., abs=1e-8)
    finally:
        h.close()


def test_tf_cloud_waits_for_main_tf_and_uses_acquisition_time_not_latest_or_frozen_alignment():
    c = CloudHarness.__new__(CloudHarness)
    try:
        c.__init__(navigation_tf=True)
        c.until(lambda: c.tf_pub.get_subscription_count())
        c.align(translation=(99., 99., 99.))  # Legacy topic must have NO effect in C2.
        c.pub.publish(cloud(10.25, frame="odom"))
        c.spin_for(.15)
        assert not c.clouds and c.statuses[-1]["state"] == "waiting_for_tf"
        c.tf_pub.publish(TFMessage(transforms=[correction(10., (1., 0., 0.)),
                                                correction(11., (3., 0., 0.))]))
        c.until(lambda: c.clouds)
        # Interpolated correction at 10.25 is 1.5; source point X is 1.0.
        assert struct.unpack_from("<ffff", c.clouds[-1].data) == pytest.approx((2.5, 0., 0., 42.))
        assert c.clouds[-1].header.stamp == stamp(10.25)
        c.pub.publish(cloud(12.5, frame="odom"))  # No transform at this future stamp.
        c.spin_for(1.2)
        assert len(c.clouds) == 1 and c.statuses[-1]["dropped"] >= 1
    finally:
        c.close()


def test_external_backend_can_own_map_odom_without_any_simulator_seed_or_competing_tf():
    h, c = NavigationHarness.__new__(NavigationHarness), CloudHarness.__new__(CloudHarness)
    try:
        h.__init__(publish_map_to_odom=False)
        h.pose(10.1)
        h.until(lambda: h.odoms)
        h.pose(10.15, (.01, 0., 0., 0., 0., 0.))
        h.until(lambda: len(h.odoms) == 2)
        h.spin_for(.1)
        assert not h.align_pub.get_subscription_count()
        assert not any(t.child_frame_id == "odom" for t in h.tfs + h.static_tfs)
        c.__init__(namespace=h.namespace + "/pointlio", navigation_tf=True, tf_namespace=h.namespace)
        c.until(lambda: c.tf_pub.get_subscription_count() >= 2)
        c.tf_pub.publish(TFMessage(transforms=[correction(10., (2., -1., .2)),
                                                correction(10.2, (2., -1., .2))]))
        c.pub.publish(cloud(10.15, ((.01, 0., 0., 42.),), frame="odom"))
        c.until(lambda: c.clouds)
        h.until(lambda: h.statuses[-1]["state"] == "ready")
        assert struct.unpack_from("<fff", c.clouds[-1].data) == pytest.approx((2.01, -1., .2))
        body = lookup(h, 10.15).transform.translation
        assert (body.x, body.y, body.z) == pytest.approx((2.01, -1., .2))
    finally:
        c.close()
        h.close()


def test_tf_ownership_is_startup_only_and_nonfinite_offsets_are_rejected():
    h = NavigationHarness.__new__(NavigationHarness)
    try:
        h.__init__()
        assert not set_parameters(h, publish_map_to_odom=False).successful
        assert not set_parameters(h, map_odom_offset_x=float("nan")).successful
    finally:
        h.close()


def test_comparison_window_uses_main_tf_and_publishes_no_parallel_tf():
    e = EvaluationHarness.__new__(EvaluationHarness)
    try:
        e.__init__(cloud_alignment=True, navigation_tf=True)
        alignments = []
        from rclpy.qos import QoSProfile, DurabilityPolicy
        e.node.create_subscription(TransformStamped, "evaluation/cloud_alignment", alignments.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        tf_pub = e.node.create_publisher(TFMessage, e.namespace + "/tf", 10)
        e.until(lambda: tf_pub.get_subscription_count())
        e.send(10., (5., 2., .2, 0., 0., 0.), truth=True)
        e.send(10., (0.,) * 6)
        e.until(lambda: alignments)
        assert alignments[-1].child_frame_id == "odom"
        assert not e.estimates and not e.transforms
        first = copy.deepcopy(alignments[-1])
        second = copy.deepcopy(first)
        second.header.stamp = stamp(10.1)
        tf_pub.publish(TFMessage(transforms=[first, second]))
        e.until(lambda: e.estimates)
        assert xyz(e.estimates[-1]) == pytest.approx((5., 2., .2))
        third = copy.deepcopy(second)
        second.header.stamp = stamp(11.)
        second.transform.translation.x += .5
        third.header.stamp = stamp(12.)
        third.transform.translation.x += .5
        tf_pub.publish(TFMessage(transforms=[second, third]))
        e.send(11.5, (5., 2., .2, 0., 0., 0.), truth=True)
        e.send(11.5, (0.,) * 6)
        e.until(lambda: e.estimates[-1].header.stamp == stamp(11.5)
                and e.errors[-1]["stamp_sec"] == 11.5)
        assert xyz(e.estimates[-1]) == pytest.approx((5.5, 2., .2))
        assert e.errors[-1]["xy_error_m"] == pytest.approx(.5)
        assert len(alignments) == 1 and not e.transforms
        assert any("MAP ORIGIN" in m.text for m in e.markers[-1].markers)
    finally:
        e.close()


@pytest.mark.parametrize("navigation_chain", [False, True])
def test_launch_uses_one_main_tree_only_for_c2_and_preserves_legacy_shadow(monkeypatch, navigation_chain):
    from launch import LaunchContext
    path = Path(__file__).parents[1] / "launch/point_lio_shadow.launch.py"
    spec = importlib.util.spec_from_file_location("shadow_tf_factory", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(namespace="red_standard_robot1", use_rviz="false")
    nodes = []
    monkeypatch.setattr(module, "Node", lambda **kwargs: nodes.append(kwargs) or kwargs)
    module._setup(context, publish_cloud_alignment=True, navigation_chain=navigation_chain)
    frontend = next(n for n in nodes if n["executable"] == "pointlio_mapping")
    evaluation = next(n for n in nodes if n["executable"] == "sim_point_lio_evaluation")
    if navigation_chain:
        assert frontend["parameters"][1] == {
            "odometry.odom_frame_id": "odom", "publish.tf_send_en": False}
        assert ("/tf", "/red_standard_robot1/tf") in evaluation["remappings"]
    else:
        assert frontend["parameters"][1] == {}
        assert ("/tf", "tf") in evaluation["remappings"]
    assert evaluation["parameters"][0]["use_navigation_tf"] == navigation_chain
