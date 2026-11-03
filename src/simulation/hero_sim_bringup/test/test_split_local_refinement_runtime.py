"""Actual Nav2/plugin integration on isolated domain 196; no motor publishers."""
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
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).parents[4]
PACKAGE = Path(__file__).parents[1]


def test_refinement_uses_dynamic_map_without_mutating_prior_and_can_be_disabled(tmp_path):
    if os.environ.get("ROS_DOMAIN_ID") != "196" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        pytest.skip("Explicit isolated domain 196 required")
    spec = importlib.util.spec_from_file_location("local_refine_factory",
        PACKAGE / "launch/perception_truth_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    resolution, nx, ny = .05, 160, 80
    # Prior wall has exactly one 80-cm passage at y=1.4. It cannot be bypassed
    # around its ends inside this arena. Live perception puts it 60 cm higher.
    prior_data = [100 if (40 <= x <= 120 and not 20 <= y < 36) else 0
                  for y in range(ny) for x in range(nx)]
    image = tmp_path / "prior.pgm"
    image.write_bytes(f"P5\n{nx} {ny}\n255\n".encode() + bytes(
        0 if prior_data[y*nx+x] else 255 for y in reversed(range(ny)) for x in range(nx)))
    metadata = tmp_path / "prior.yaml"
    metadata.write_text(yaml.safe_dump(dict(image=str(image), resolution=resolution,
        origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.25)))
    namespace = "local_refine_" + uuid.uuid4().hex[:8]
    generated = Path(module._make_sim_params(
        str(ROOT / "src/planning/hero2025_nav_bringup/config/nav2_params.yaml"),
        str(metadata), "/tmp/no_bt.xml", "split", namespace, False))
    params = yaml.safe_load(generated.read_text())
    generated.unlink()
    settings = yaml.safe_load((PACKAGE / "config/split_local_refinement.yaml").read_text())
    settings["local_refinement_regions"] = [4., 1.4, 0., 4., .8]
    params["planner_server"]["ros__parameters"]["GridBased"].update(settings)

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
    rclpy.init()
    node = rclpy.create_node("refinement_test_driver", namespace="/"+namespace)
    persistent = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    prior_pub = node.create_publisher(OccupancyGrid, "planning_prior/map", persistent)
    boundary_pub = node.create_publisher(OccupancyGrid, "map", persistent)
    tf_pub = node.create_publisher(TFMessage, "tf_static", persistent)
    cloud_pub = node.create_publisher(PointCloud2, "rog_map/inf_occ", qos_profile_sensor_data)
    maps, paths, statuses = {}, {}, []
    subscriptions = [node.create_subscription(Costmap, f"{name}/costmap_raw",
        lambda message, key=name: maps.__setitem__(key, message), persistent)
        for name in ("global_costmap", "static_prior_costmap")]
    subscriptions += [node.create_subscription(PathMsg, f"GridBased/{name}",
        lambda message, key=name: paths.__setitem__(key, message), 1)
        for name in ("static_plan", "refined_plan")]
    subscriptions.append(node.create_subscription(String, "GridBased/refinement_status",
        lambda message: statuses.append(message.data), 1))
    processes, streams = [], []

    def occupancy(data):
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.info.resolution, message.info.width, message.info.height = resolution, nx, ny
        message.info.origin.orientation.w = 1.
        message.data = data
        return message

    def cloud():
        points = [(2.+.05*k, y, .2) for k in range(81) for y in (1.6, 2.4)]
        points += [(2., 1.6+.05*k, .2) for k in range(17)]  # Unfiltered lintel.
        message = PointCloud2()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.height, message.width = 1, len(points)
        message.fields = [PointField(name=name, offset=4*k, datatype=PointField.FLOAT32, count=1)
                          for k, name in enumerate(("x", "y", "z"))]
        message.point_step, message.row_step = 12, 12*len(points)
        message.data = b"".join(struct.pack("fff", *p) for p in points)
        return message

    def spin_until(condition, timeout=15.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if condition():
                return
            for process in processes:
                assert process.poll() is None, "Planner exited; inspect planner_server.log"
            rclpy.spin_once(node, timeout_sec=.03)
        raise AssertionError("Timed out in isolated local-refinement integration test")

    def result(future):
        spin_until(future.done)
        return future.result()

    def transition(which):
        service = node.create_client(ChangeState, "planner_server/change_state")
        assert service.wait_for_service(timeout_sec=10.)
        request = ChangeState.Request()
        request.transition.id = which
        try:
            assert result(service.call_async(request)).success
        finally:
            node.destroy_client(service)

    def toggle(value):
        service = node.create_client(SetParameters, "planner_server/set_parameters")
        assert service.wait_for_service(timeout_sec=5.)
        request = SetParameters.Request(parameters=[Parameter(
            "GridBased.local_refinement_enabled", value=value).to_parameter_msg()])
        try:
            assert result(service.call_async(request)).results[0].successful
        finally:
            node.destroy_client(service)

    def plan():
        client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        assert client.wait_for_server(timeout_sec=10.)
        request = ComputePathToPose.Goal()
        request.planner_id, request.use_start = "GridBased", True
        for pose, xy in ((request.start, (1., 2.)), (request.goal, (7.4, 1.4))):
            pose.header.frame_id = "map"
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = xy
            pose.pose.orientation.w = 1.
        try:
            handle = result(client.send_goal_async(request))
            assert handle.accepted
            output = result(handle.get_result_async())
            assert output.status == GoalStatus.STATUS_SUCCEEDED
            return [(p.pose.position.x, p.pose.position.y) for p in output.result.path.poses]
        finally:
            client.destroy()

    def map_cost(x, y):
        message = maps["global_costmap"]
        info = message.metadata
        ix = int((x-info.origin.position.x)/info.resolution)
        iy = int((y-info.origin.position.y)/info.resolution)
        return message.data[iy*info.size_x+ix]

    timer = None
    try:
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_link"
        transform.transform.translation.x, transform.transform.translation.y = 1., 2.
        transform.transform.rotation.w = 1.
        tf_pub.publish(TFMessage(transforms=[transform]))
        prior_pub.publish(occupancy(prior_data))
        boundary = [100 if min(x, nx-1-x, y, ny-1-y) < 3 else 0
                    for y in range(ny) for x in range(nx)]
        boundary_pub.publish(occupancy(boundary))
        log = (tmp_path / "planner_server.log").open("w")
        streams.append(log)
        executable = Path(get_package_prefix("nav2_planner")) / "lib/nav2_planner/planner_server"
        processes.append(subprocess.Popen([str(executable), "--ros-args", "-r", f"__ns:=/{namespace}",
            "--params-file", str(config), "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        transition(Transition.TRANSITION_CONFIGURE)
        transition(Transition.TRANSITION_ACTIVATE)
        timer = node.create_timer(.1, lambda: cloud_pub.publish(cloud()))
        spin_until(lambda: len(maps) == 2 and map_cost(4., 1.6) >= 253)
        frozen_prior = bytes(maps["static_prior_costmap"].data)
        refined = plan()
        spin_until(lambda: bool(statuses))
        assert any("result=applied" in status for status in statuses), statuses
        assert any(3.5 < x < 4.5 for x, y in refined)
        assert all(1.8 < y < 2.2 for x, y in refined if 3.5 < x < 4.5), statuses
        assert bytes(maps["static_prior_costmap"].data) == frozen_prior
        toggle(False)
        original = plan()
        assert all(y < 1.8 for x, y in original if 3.5 < x < 4.5)
        assert refined[0] == pytest.approx(original[0])
        assert refined[-1] == pytest.approx(original[-1])
        toggle(True)
        second = plan()
        assert all(1.8 < y < 2.2 for x, y in second if 3.5 < x < 4.5)
        assert bytes(maps["static_prior_costmap"].data) == frozen_prior
        (tmp_path / "paths_and_status.json").write_text(json.dumps(dict(
            refined=refined, original=original, enabled_again=second, statuses=statuses), indent=2))
        transition(Transition.TRANSITION_DEACTIVATE)
        transition(Transition.TRANSITION_CLEANUP)
        assert not node.get_publishers_info_by_topic("/"+namespace+"/cmd_vel")
    finally:
        if timer:
            node.destroy_timer(timer)
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
        for stream in streams:
            stream.close()
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()
