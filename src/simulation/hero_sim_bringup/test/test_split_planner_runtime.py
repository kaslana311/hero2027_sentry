"""Real planner/plugin/costmap integration; isolated ROS domain, NO motor topics.

Run only with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=196 after the safe build.
The only launched executables are planner_server and smoother_server. Synthetic
map/TF/cloud publishers replace Gazebo, DOG and the robot; no cmd_vel is created.
"""
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).parents[4]
PACKAGE = Path(__file__).parents[1]


@pytest.mark.parametrize("remap_tf", (False, True))
def test_static_prior_is_independent_of_live_dynamic_master(tmp_path, remap_tf):
    if os.environ.get("ROS_DOMAIN_ID") != "196" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        pytest.skip("Explicit isolated domain 196 is required; never use the robot domain")
    spec = importlib.util.spec_from_file_location(
        "split_runtime_factory", PACKAGE / "launch/perception_truth_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    namespace = "split_probe_" + uuid.uuid4().hex[:10]
    prefix = "/" + namespace
    resolution, nx, ny = .05, 120, 80
    image = tmp_path / "prior.pgm"
    pixels = bytearray([255]) * (nx * ny)
    # A prior-only wall crosses the straight route but can be bypassed above it.
    for x in range(56, 64):
        for y in range(0, 50):
            pixels[(ny - 1 - y) * nx + x] = 0
    image.write_bytes(f"P5\n{nx} {ny}\n255\n".encode() + pixels)
    map_yaml = tmp_path / "prior.yaml"
    map_yaml.write_text(yaml.safe_dump(dict(image=str(image), resolution=resolution,
        origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.25)))
    generated = Path(module._make_sim_params(
        str(ROOT / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"),
        str(map_yaml), "/tmp/no_bt_started.xml", "split", namespace, False))
    params = yaml.safe_load(generated.read_text())
    generated.unlink()

    def wall_time(value):
        if isinstance(value, dict):
            for key in value:
                if key == "use_sim_time":
                    value[key] = False
                else:
                    wall_time(value[key])
    wall_time(params)
    config = tmp_path / "params.yaml"
    config.write_text(yaml.safe_dump({namespace: params}))
    processes, streams = [], []
    rclpy.init()
    node = rclpy.create_node("split_test_driver", namespace=prefix)
    maps = {}
    persistent = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    prior_pub = node.create_publisher(OccupancyGrid, "planning_prior/map", persistent)
    boundary_pub = node.create_publisher(OccupancyGrid, "map", persistent)
    tf_pub = node.create_publisher(TFMessage, "tf_static" if remap_tf else "/tf_static", persistent)
    cloud_pub = node.create_publisher(PointCloud2, "rog_map/inf_occ", qos_profile_sensor_data)
    subs = [node.create_subscription(Costmap, f"{name}/costmap_raw",
        lambda msg, key=name: maps.__setitem__(key, msg), persistent)
        for name in ("global_costmap", "static_prior_costmap")]

    def spin_until(predicate, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            for process in processes:
                assert process.poll() is None, "ROS server exited; see captured logs"
            rclpy.spin_once(node, timeout_sec=.05)
        raise AssertionError("Timed out waiting for isolated ROS test condition")

    def future_result(future):
        spin_until(future.done)
        return future.result()

    def transition(server, identifier):
        client = node.create_client(ChangeState, f"{server}/change_state")
        assert client.wait_for_service(timeout_sec=10.)
        request = ChangeState.Request()
        request.transition.id = identifier
        try:
            assert future_result(client.call_async(request)).success
        finally:
            node.destroy_client(client)

    def occupancy(data):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width, msg.info.height = nx, ny
        msg.info.origin.orientation.w = 1.
        msg.data = data
        return msg

    def cost_at(name, x, y):
        msg = maps[name]
        info = msg.metadata
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        return msg.data[my * info.size_x + mx]

    def set_parameter(name, value, accepted=True):
        client = node.create_client(SetParameters, "planner_server/set_parameters")
        assert client.wait_for_service(timeout_sec=10.)
        request = SetParameters.Request()
        request.parameters = [Parameter("GridBased."+name, value=value).to_parameter_msg()]
        try:
            results = future_result(client.call_async(request)).results
            assert len(results) == 1 and results[0].successful is accepted
        finally:
            node.destroy_client(client)

    def plan(start=(1., 1.), destination=(5., 1.), status=GoalStatus.STATUS_SUCCEEDED):
        client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        assert client.wait_for_server(timeout_sec=10.)
        goal = ComputePathToPose.Goal()
        goal.planner_id = "GridBased"
        goal.use_start = True
        for pose, xy in ((goal.start, start), (goal.goal, destination)):
            pose.header.frame_id = "map"
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = xy
            pose.pose.orientation.w = 1.
        try:
            handle = future_result(client.send_goal_async(goal))
            assert handle.accepted
            result = future_result(handle.get_result_async())
            if result.status != status:
                for name, msg in maps.items():
                    info = msg.metadata
                    (tmp_path / (name+"_failure.json")).write_text(json.dumps(dict(
                        width=info.size_x, height=info.size_y, resolution=info.resolution,
                        origin=[info.origin.position.x, info.origin.position.y],
                        start=start, goal=destination, costs=list(msg.data))))
                print("Failed planning request", start, "->", destination)
            assert result.status == status
            return [(p.pose.position.x, p.pose.position.y) for p in result.result.path.poses]
        finally:
            client.destroy()

    try:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = 1., 1.
        tf.transform.rotation.w = 1.
        tf_pub.publish(TFMessage(transforms=[tf]))
        prior_data = [100 if pixels[(ny-1-y)*nx+x] == 0 else 0
                      for y in range(ny) for x in range(nx)]
        prior_pub.publish(occupancy(prior_data))
        boundary_data = [100 if min(x, nx-1-x, y, ny-1-y) < 7 else 0
                         for y in range(ny) for x in range(nx)]
        boundary_pub.publish(occupancy(boundary_data))
        for package, executable in (("nav2_planner", "planner_server"),
                                    ("nav2_smoother", "smoother_server")):
            log = (tmp_path / f"{executable}.log").open("w")
            streams.append(log)
            arguments = [
                str(Path(get_package_prefix(package)) / "lib" / package / executable),
                "--ros-args", "-r", f"__ns:={prefix}", "--params-file", str(config)]
            if remap_tf:
                arguments += ["-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]
            processes.append(subprocess.Popen(arguments,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            transition(executable, Transition.TRANSITION_CONFIGURE)
        transition("planner_server", Transition.TRANSITION_ACTIVATE)
        spin_until(lambda: all(key in maps for key in ("global_costmap", "static_prior_costmap")))
        spin_until(lambda: cost_at("static_prior_costmap", 3., 1.) >= 253)
        assert cost_at("global_costmap", 3., 1.) == 0
        original_prior = bytes(maps["static_prior_costmap"].data)
        before = plan()
        assert len(before) > 2
        assert any(y > 2.65 for x, y in before), "Smac ignored the prior wall"

        # A previous goal may leave the real start inside only the PGM wall.
        # It must retain its actual first pose and reconnect, never erase PGM.
        for destination in ((5., 1.), (1., 1.), (5., 1.)):
            recovered = plan(start=(3., 1.), destination=destination)
            assert recovered[0] == pytest.approx((3., 1.))
            assert recovered[-1] == pytest.approx(destination, abs=.08)
            assert max(((b[0]-a[0])**2+(b[1]-a[1])**2)**.5
                       for a, b in zip(recovered, recovered[1:])) < .10
            spin_until(lambda: cost_at("static_prior_costmap", 3., 1.) >= 253)
            assert bytes(maps["static_prior_costmap"].data) == original_prior
        assert [v for p in plan() for v in p] == pytest.approx([v for p in before for v in p])

        # Radius/disable are checked on each request, not silently ignored at runtime.
        set_parameter("start_recovery_radius", .10)
        assert not plan(start=(3., 1.), status=GoalStatus.STATUS_ABORTED)
        set_parameter("start_recovery_radius", 0., accepted=False)
        set_parameter("start_recovery_radius", .60)
        set_parameter("start_recovery_enabled", False)
        assert not plan(start=(3., 1.), status=GoalStatus.STATUS_ABORTED)
        set_parameter("start_recovery_enabled", True)
        assert plan(start=(3., 1.))[0] == pytest.approx((3., 1.))

        # Regression: a collision-free start can still have no valid Dubin
        # departure. At cell 52 the next forward/turning primitives hit the
        # inflated prior wall. Use its centre, not the float32 cell boundary.
        # The old "start is free" early return got stuck.
        trapped_start = (2.625, 1.)
        # A nearby goal can still use Smac's valid analytic shortcut, which
        # must remain successful and unchanged. Farther goals exercise the
        # blocked lattice departure rather than that separate shortcut.
        trapped_goals = ((.60, 3.30), (5., 3.20))
        for destination in trapped_goals:
            assert sum((a-b)**2 for a, b in zip(trapped_start, destination))**.5 > (
                params["planner_server"]["ros__parameters"]["GridBased"]["analytic_expansion_max_length"])
        assert 0 < cost_at("static_prior_costmap", *trapped_start) < 253
        assert cost_at("global_costmap", *trapped_start) == 0
        set_parameter("start_recovery_enabled", False)
        assert not plan(start=trapped_start, destination=trapped_goals[0],
                        status=GoalStatus.STATUS_ABORTED)
        set_parameter("start_recovery_enabled", True)
        for destination in (*trapped_goals, trapped_goals[0]):
            recovered = plan(start=trapped_start, destination=destination)
            assert recovered[0] == pytest.approx(trapped_start)
            assert recovered[-1] == pytest.approx(destination, abs=.08)
            assert max(((b[0]-a[0])**2+(b[1]-a[1])**2)**.5
                       for a, b in zip(recovered, recovered[1:])) < .10
            assert bytes(maps["static_prior_costmap"].data) == original_prior
        assert [v for p in plan() for v in p] == pytest.approx([v for p in before for v in p])

        # The new trigger must not bypass a real dynamic collision at this
        # otherwise prior-valid start. Restore the map normally, never by an
        # artificial empty point cloud or by changing the static prior.
        blocked_departure = list(boundary_data)
        for y in range(18, 23):
            for x in range(49, 55):
                blocked_departure[y*nx+x] = 100
        boundary_pub.publish(occupancy(blocked_departure))
        spin_until(lambda: cost_at("global_costmap", *trapped_start) >= 253)
        assert not plan(start=trapped_start, destination=trapped_goals[0],
                        status=GoalStatus.STATUS_ABORTED)
        boundary_pub.publish(occupancy(boundary_data))
        spin_until(lambda: cost_at("global_costmap", *trapped_start) == 0)
        assert plan(start=trapped_start, destination=trapped_goals[0])[0] == pytest.approx(trapped_start)
        assert bytes(maps["static_prior_costmap"].data) == original_prior

        # A genuinely occupied dynamic start must still fail. Restore the full
        # synthetic map afterwards; do not introduce empty-cloud clearing.
        blocked = list(boundary_data)
        for y in range(18, 23):
            for x in range(58, 63):
                blocked[y*nx+x] = 100
        boundary_pub.publish(occupancy(blocked))
        spin_until(lambda: cost_at("global_costmap", 3., 1.) >= 253)
        assert not plan(start=(3., 1.), status=GoalStatus.STATUS_ABORTED)
        boundary_pub.publish(occupancy(boundary_data))
        spin_until(lambda: cost_at("global_costmap", 3., 1.) == 0)
        assert plan(start=(3., 1.))[0] == pytest.approx((3., 1.))
        assert bytes(maps["static_prior_costmap"].data) == original_prior

        # The configured MINCO really subscribes to the normal dynamic master,
        # not the wrapper's static map. It stays inactive: no trajectory is run.
        spin_until(lambda: any(info.node_name == "smoother_server" for info in
            node.get_subscriptions_info_by_topic(prefix + "/global_costmap/costmap_raw")))
        assert not any(info.node_name == "smoother_server" for info in
            node.get_subscriptions_info_by_topic(prefix + "/static_prior_costmap/costmap_raw"))

        # Add a dynamic-only barrier crossing the previous route. It must appear
        # in the global ESDF input, but not in the static map or A* output.
        cloud = PointCloud2()
        cloud.header.frame_id = "map"
        cloud.header.stamp = node.get_clock().now().to_msg()
        cloud.height, cloud.width = 1, 66
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(("x", "y", "z"))]
        cloud.point_step, cloud.row_step = 12, 12 * cloud.width
        cloud.data = b"".join(struct.pack("fff", 1.6, .35 + .05*i, .2) for i in range(cloud.width))
        assert cloud_pub.get_subscription_count() > 0
        cloud_pub.publish(cloud)
        spin_until(lambda: cost_at("global_costmap", 1.6, 1.) >= 253)
        assert cost_at("static_prior_costmap", 1.6, 1.) == 0
        assert bytes(maps["static_prior_costmap"].data) == original_prior
        after = plan()
        assert [v for point in after for v in point] == pytest.approx(
            [v for point in before for v in point])

        # Exercise thread teardown and a second navigation-server lifecycle.
        transition("smoother_server", Transition.TRANSITION_CLEANUP)
        transition("planner_server", Transition.TRANSITION_DEACTIVATE)
        transition("planner_server", Transition.TRANSITION_CLEANUP)
        transition("planner_server", Transition.TRANSITION_CONFIGURE)
        transition("planner_server", Transition.TRANSITION_ACTIVATE)
        assert [v for point in plan() for v in point] == pytest.approx(
            [v for point in before for v in point])
        transition("planner_server", Transition.TRANSITION_DEACTIVATE)
        transition("planner_server", Transition.TRANSITION_CLEANUP)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        # Include logs in failed-test output, and leave originals in pytest tmp.
        for log in tmp_path.glob("*.log"):
            print(f"\n--- {log.name} ---\n{log.read_text()[-10000:]}")
