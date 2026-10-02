import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


VALID_COSTMAP_MODES = ("combined", "static_only", "dynamic_only")
VALID_LIDAR_MODES = ("single", "dual")
VALID_FIELDS = ("rmuc_2025", "rmuc_2026")


def _make_sim_params(
    source_file, map_yaml, behavior_tree_xml, mode, namespace, publish_debug_visualizations,
    perception_cloud_topic="cloud_registered",
):
    with open(source_file, "r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    dog_map = params["dog_map_node"]["ros__parameters"]
    dog_map["use_sim_time"] = True
    dog_map["odom_topic"] = "odometry/filtered"
    dog_map["cloud_topic"] = perception_cloud_topic
    dog_map["map_yaml_path"] = map_yaml

    for node_name in (
        "bt_navigator",
        "controller_server",
        "velocity_smoother",
    ):
        node_params = params[node_name]["ros__parameters"]
        if "odom_topic" in node_params:
            node_params["odom_topic"] = "odometry/filtered"

    bt_navigator = params["bt_navigator"]["ros__parameters"]
    # Use a lower replanning rate in simulation so transient point-cloud noise
    # cannot flood the smoother and RViz. The real-robot behavior tree remains
    # unchanged.
    bt_navigator["default_nav_to_pose_bt_xml"] = behavior_tree_xml
    # Be explicit in generated files and discard the historical misspelling.
    bt_navigator.pop("default_server_timfeout", None)
    bt_navigator["default_server_timeout"] = 100

    # In simulation a raised 3-D mesh edge can stop the chassis even though
    # the 2-D costmap still considers the cell traversable. Detect that case
    # promptly, while accepting slow but real progress through narrow gaps.
    progress_checker = params["controller_server"]["ros__parameters"][
        "progress_checker"
    ]
    progress_checker["required_movement_radius"] = 0.10
    progress_checker["movement_time_allowance"] = 5.0

    smoother = params["smoother_server"]["ros__parameters"].get("minco_smoother", {})
    if "odom_topic" in smoother:
        smoother["odom_topic"] = "odometry/filtered"
    smoother["publish_trajectory"] = publish_debug_visualizations

    mode_plugins = {
        "combined": ["static_layer", "fast_layer", "inflation_layer"],
        "static_only": ["static_layer", "inflation_layer"],
        "dynamic_only": ["fast_layer", "inflation_layer"],
    }

    topic_root = f"/{namespace.strip('/')}" if namespace.strip("/") else ""

    def robot_topic(name):
        return f"{topic_root}/{name}" if topic_root else f"/{name}"

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = params[costmap_name][costmap_name]["ros__parameters"]
        costmap["use_sim_time"] = True
        # Keep the lists independent. Otherwise PyYAML emits anchors/aliases,
        # which the ROS 2 parameter-file parser does not support.
        costmap["plugins"] = list(mode_plugins[mode])
        # Costmap plugins run below /<robot>/<local|global>_costmap. Absolute
        # robot-scoped topics prevent ROS from incorrectly resolving these as
        # /<robot>/<costmap>/map and /<robot>/<costmap>/rog_map/inf_occ.
        costmap.setdefault("static_layer", {})["map_topic"] = robot_topic("map")
        costmap["static_layer"]["cloud_update_topic"] = robot_topic("rog_map/fi")
        costmap.setdefault("fast_layer", {})["pointcloud_topic"] = robot_topic(
            "rog_map/inf_occ"
        )

    # Without a static layer there is no incoming map to size the global costmap.
    if mode == "dynamic_only":
        global_costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
        global_costmap["rolling_window"] = True
        # Nav2 declares these parameters as integers from the base YAML. ROS 2
        # rejects replacing an already-declared integer with a YAML double.
        global_costmap["width"] = 12
        global_costmap["height"] = 12

    def force_sim_time(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == "use_sim_time":
                    value[key] = True
                else:
                    force_sim_time(child)
        elif isinstance(value, list):
            for child in value:
                force_sim_time(child)

    force_sim_time(params)

    output = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"hero_nav2_{mode}_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(params, output, sort_keys=False)
    output.close()
    return output.name


def _launch_setup(context, perception_cloud_topic="cloud_registered"):
    namespace = LaunchConfiguration("namespace").perform(context)
    costmap_mode = LaunchConfiguration("costmap_mode").perform(context).lower()
    lidar_mode = LaunchConfiguration("lidar_mode").perform(context).lower()
    field = LaunchConfiguration("field").perform(context).lower()
    use_rviz = LaunchConfiguration("use_rviz")
    relay_cmd_vel = LaunchConfiguration("relay_cmd_vel")
    publish_debug_visualizations = (
        LaunchConfiguration("publish_debug_visualizations")
        .perform(context)
        .lower()
        in ("1", "true", "yes", "on")
    )

    if costmap_mode not in VALID_COSTMAP_MODES:
        raise RuntimeError(
            f"Unknown costmap_mode '{costmap_mode}'. Choose one of: {', '.join(VALID_COSTMAP_MODES)}"
        )
    if lidar_mode not in VALID_LIDAR_MODES:
        raise RuntimeError(
            f"Unknown lidar_mode '{lidar_mode}'. Choose one of: {', '.join(VALID_LIDAR_MODES)}"
        )
    if field not in VALID_FIELDS:
        raise RuntimeError(
            f"Unknown field '{field}'. Choose one of: {', '.join(VALID_FIELDS)}"
        )

    sim_share = get_package_share_directory("hero_sim_bringup")
    nav_share = get_package_share_directory("hero2025_nav_bringup")
    map_yaml = os.path.join(sim_share, "maps", f"{field}.yaml")
    if not os.path.isfile(map_yaml):
        raise RuntimeError(f"Static map metadata not found: {map_yaml}")
    behavior_tree_xml = os.path.join(
        sim_share, "behavior_trees", "navigate_to_pose_sim.xml"
    )
    source_params = os.path.join(nav_share, "config", "nav2_params.yaml")
    generated_params = _make_sim_params(
        source_params,
        map_yaml,
        behavior_tree_xml,
        costmap_mode,
        namespace,
        publish_debug_visualizations,
        perception_cloud_topic,
    )

    bridge = Node(
        package="hero_sim_bringup",
        executable="sim_ground_truth_bridge",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "zero_at_first_odometry": True,
                "relay_cmd_vel": relay_cmd_vel,
                "dual_lidar": lidar_mode == "dual",
            }
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            # The real-robot RViz file uses this absolute topic. Keep that file
            # unchanged and adapt it at the simulation entry point.
            ("/rog_map/inf_occ", "rog_map/inf_occ"),
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        namespace=namespace,
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": map_yaml}],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    navigation = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            # HeroMpcController uses this absolute single-robot topic in the
            # real-robot code. Connect it to the namespaced smoother output in
            # the multi-robot simulator without modifying the controller.
            SetRemap(
                "/smoother_server/minco_polynomial_trajectory",
                f"/{namespace}/smoother_server/minco_polynomial_trajectory",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": generated_params,
                    # The team launch evaluates use_composition through PythonExpression,
                    # so it needs Python boolean spelling rather than ROS-style lowercase.
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
        ]
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(sim_share, "rviz", "hero_sim.rviz")],
        parameters=[{"use_sim_time": True}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return [
        LogInfo(
            msg=(
                f"Starting HERO perception simulation in '{costmap_mode}' costmap mode "
                f"with '{lidar_mode}' lidar input on '{field}'"
            )
        ),
        LogInfo(msg=f"Generated parameter file: {generated_params}"),
        bridge,
        map_server,
        map_lifecycle,
        navigation,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="red_standard_robot1",
                description="Simulator robot namespace",
            ),
            DeclareLaunchArgument(
                "costmap_mode",
                default_value="combined",
                description="combined, static_only, or dynamic_only",
            ),
            DeclareLaunchArgument(
                "lidar_mode",
                default_value="single",
                description="single (official simulator image) or dual (custom dual-lidar image)",
            ),
            DeclareLaunchArgument(
                "field",
                default_value="rmuc_2025",
                description="Static map selection: rmuc_2025 or rmuc_2026",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the team navigation configuration",
            ),
            DeclareLaunchArgument(
                "relay_cmd_vel",
                default_value="true",
                description="Relay the team controller output to the simulated chassis",
            ),
            DeclareLaunchArgument(
                "publish_debug_visualizations",
                default_value="false",
                description="Publish high-rate MINCO debug Path messages",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
