import copy
import math
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


VALID_COSTMAP_MODES = ("combined", "static_only", "dynamic_only", "split")
VALID_LIDAR_MODES = ("single", "dual")
VALID_FIELDS = ("rmuc_2025", "rmuc_2026", "rmuc_2027")


def _dynamic_map_info(map_yaml):
    """Read PGM geometry only; never copy its interior obstacle pixels."""
    with open(map_yaml, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = (float(value) for value in metadata["origin"][:2])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Map resolution must be finite and positive")
    if not all(math.isfinite(value) for value in (origin_x, origin_y)):
        raise ValueError("Map origin must be finite")
    image_path = os.path.join(os.path.dirname(map_yaml), metadata["image"])
    with open(image_path, "rb") as stream:
        header = []
        while len(header) < 3:
            line = stream.readline()
            if not line:
                raise ValueError("Incomplete PGM header")
            header.extend(line.split(b"#", 1)[0].split())
    if header[0] not in (b"P2", b"P5"):
        raise ValueError("Dynamic-only map geometry requires a P2 or P5 PGM")
    cells_x, cells_y = int(header[1]), int(header[2])
    if cells_x <= 0 or cells_y <= 0:
        raise ValueError("PGM dimensions must be positive")
    return metadata, cells_x, cells_y


def _dynamic_global_geometry(map_yaml):
    metadata, cells_x, cells_y = _dynamic_map_info(map_yaml)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = (float(value) for value in metadata["origin"][:2])
    return {
        "origin_x": origin_x,
        "origin_y": origin_y,
        "resolution": resolution,
        # Humble declares width/height as integer metres. Round UP so the
        # fixed map covers all PGM cells without adding a static-map plugin.
        "width": math.ceil(cells_x * resolution),
        "height": math.ceil(cells_y * resolution),
    }


def _make_dynamic_boundary_map(map_yaml, boundary_width=0.35, output_directory=None):
    """Free arena interior, occupied inward rim and integer-window padding.

    PGM rows run top to bottom, whereas OccupancyGrid Y runs bottom to top.
    Keep the original arena's cell bounds, not the rounded costmap's edges:
    the extra positive-X/Y padding must never become a route off the floor.
    The default rim adds 0.15 m inward on every side to the original 0.20 m.
    """
    metadata, cells_x, cells_y = _dynamic_map_info(map_yaml)
    resolution = float(metadata["resolution"])
    if not math.isfinite(boundary_width) or boundary_width <= 0:
        raise ValueError("Boundary width must be finite and positive")
    rim = math.ceil(boundary_width / resolution)
    if cells_x <= 2 * rim or cells_y <= 2 * rim:
        raise ValueError("Arena is too small for the occupied boundary rim")
    geometry = _dynamic_global_geometry(map_yaml)
    padded_x = math.ceil(geometry["width"] / resolution)
    padded_y = math.ceil(geometry["height"] / resolution)
    pixels = bytearray(padded_x * padded_y)  # Occupied rim and padding: black.
    free_row = bytes([255]) * (cells_x - 2 * rim)
    for y in range(rim, cells_y - rim):
        start = (padded_y - 1 - y) * padded_x + rim
        pixels[start:start + len(free_row)] = free_row

    directory = output_directory or tempfile.mkdtemp(prefix="hero_dynamic_boundary_")
    os.makedirs(directory, exist_ok=True)
    image_path = os.path.abspath(os.path.join(directory, "boundary_only.pgm"))
    yaml_path = os.path.abspath(os.path.join(directory, "boundary_only.yaml"))
    with open(image_path, "wb") as stream:
        stream.write(f"P5\n{padded_x} {padded_y}\n255\n".encode())
        stream.write(pixels)
    boundary_metadata = {
        "image": image_path,
        "mode": "trinary",
        "resolution": resolution,
        "origin": list(metadata["origin"]),
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(yaml_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(boundary_metadata, stream, sort_keys=False)
    return yaml_path


def _make_sim_params(
    source_file, map_yaml, behavior_tree_xml, mode, namespace, publish_debug_visualizations,
    perception_cloud_topic="cloud_registered",
    filter_high_columns=False, high_column_min_z=0.30,
):
    if not isinstance(filter_high_columns, bool):
        raise ValueError("filter_high_columns must be a boolean")
    high_column_min_z = float(high_column_min_z)
    if not math.isfinite(high_column_min_z) or high_column_min_z < 0:
        raise ValueError("high_column_min_z must be finite and nonnegative (absolute map Z, metres)")
    with open(source_file, "r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    dog_map = params["dog_map_node"]["ros__parameters"]
    dog_map["use_sim_time"] = True
    dog_map["odom_topic"] = "odometry/filtered"
    dog_map["cloud_topic"] = perception_cloud_topic
    dog_map["map_yaml_path"] = map_yaml
    dog_map["filter_high_columns"] = filter_high_columns
    dog_map["high_column_min_z"] = high_column_min_z

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
        "dynamic_only": ["boundary_layer", "fast_layer", "inflation_layer"],
        "split": ["boundary_layer", "fast_layer", "inflation_layer"],
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
        if mode in ("dynamic_only", "split"):
            # Standard Nav2 static layer, not the team's mutable static layer.
            # Only the generated free-interior/boundary map is served in this mode.
            costmap["boundary_layer"] = {
                "plugin": "nav2_costmap_2d::StaticLayer",
                "map_topic": robot_topic("map"),
                "map_subscribe_transient_local": True,
                "subscribe_to_updates": False,
                "use_maximum": True,
            }

    # Original PGM interior pixels never enter dynamic-only navigation.
    # BoundaryLayer adds only the arena rim; FastLayer supplies sensed obstacles.
    if mode in ("dynamic_only", "split"):
        global_costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
        global_costmap.update(_dynamic_global_geometry(map_yaml))
        global_costmap["rolling_window"] = False
        global_costmap["track_unknown_space"] = False

    if mode == "split":
        # Both normal Nav2 maps stay identical to dynamic_only. Only Smac's
        # input changes: a private, read-only PGM + inflation map. The standard
        # StaticLayer does not clear PGM walls using DOG free-space data.
        prior = copy.deepcopy(params["global_costmap"]["global_costmap"]["ros__parameters"])
        for unused in ("static_layer", "fast_layer", "boundary_layer"):
            prior.pop(unused, None)
        prior.update(
            plugins=["prior_static_layer", "inflation_layer"],
            track_unknown_space=True,
            prior_static_layer={
                "plugin": "nav2_costmap_2d::StaticLayer",
                "map_topic": robot_topic("planning_prior/map"),
                "map_subscribe_transient_local": True,
                "subscribe_to_updates": False,
                "footprint_clearing_enabled": False,
            },
        )
        params["static_prior_costmap"] = {"static_prior_costmap": {"ros__parameters": prior}}
        params["planner_server"]["ros__parameters"]["GridBased"]["plugin"] = (
            "hero_static_prior_planner/StaticPriorPlanner"
        )
        params["smoother_server"]["ros__parameters"]["costmap_topic"] = (
            "global_costmap/costmap_raw"
        )

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


def _launch_setup(context, perception_cloud_topic="cloud_registered", point_lio_navigation=False):
    namespace = LaunchConfiguration("namespace").perform(context)
    costmap_mode = LaunchConfiguration("costmap_mode").perform(context).lower()
    lidar_mode = LaunchConfiguration("lidar_mode").perform(context).lower()
    field = LaunchConfiguration("field").perform(context).lower()
    use_rviz = LaunchConfiguration("use_rviz")
    relay_cmd_vel = LaunchConfiguration("relay_cmd_vel")
    high_column_switch = LaunchConfiguration("filter_high_columns", default="false").perform(context).lower()
    if high_column_switch not in ("true", "false"):
        raise ValueError("filter_high_columns must be true or false")
    filter_high_columns = high_column_switch == "true"
    high_column_min_z = float(LaunchConfiguration("high_column_min_z", default="0.30").perform(context))
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
        filter_high_columns,
        high_column_min_z,
    )
    navigation_map_yaml = (
        _make_dynamic_boundary_map(map_yaml)
        if costmap_mode in ("dynamic_only", "split") else map_yaml
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
                "relay_cmd_vel": False if point_lio_navigation else relay_cmd_vel,
                **({"output_odom_topic": "odometry/truth_planar",
                    "output_cmd_vel_topic": "truth/unrelayed_cmd_vel"}
                   if point_lio_navigation else {}),
                "dual_lidar": lidar_mode == "dual",
            }
        ],
        remappings=[
            ("/tf", "truth/tf" if point_lio_navigation else "tf"),
            ("/tf_static", "truth/tf_static" if point_lio_navigation else "tf_static"),
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
        parameters=[{"use_sim_time": True, "yaml_filename": navigation_map_yaml}],
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

    reference_map_nodes = []
    reference_map_remappings = []
    if costmap_mode == "dynamic_only":
        # Display only: never replace the boundary map or its lifecycle manager.
        # A private namespace also keeps map/load_map services away from navigation.
        reference_namespace = f"{namespace.rstrip('/')}/visualization"
        reference_map_nodes = [
            Node(
                condition=IfCondition(use_rviz),
                package="nav2_map_server", executable="map_server",
                namespace=reference_namespace, name="reference_map_server",
                output="screen",
                parameters=[{"use_sim_time": True, "yaml_filename": map_yaml,
                             "frame_id": "map"}],
                remappings=[("map", "static_map")],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                namespace=reference_namespace, name="lifecycle_manager_reference_map",
                output="screen",
                parameters=[{"use_sim_time": True, "autostart": True,
                             "node_names": ["reference_map_server"]}],
            ),
        ]
        # Only RViz's existing Map background reads the original PGM. Its
        # Global/Local Costmap displays still read the real navigation costmaps.
        reference_map_remappings = [("map", "visualization/static_map")]

    if costmap_mode == "split":
        # A planning input, NOT an RViz-only node: needed with use_rviz:=false.
        # Its private map/load_map cannot overwrite the boundary-only /map.
        prior_namespace = f"{namespace.rstrip('/')}/planning_prior"
        reference_map_nodes = [
            Node(
                package="nav2_map_server", executable="map_server",
                namespace=prior_namespace, name="static_prior_map_server", output="screen",
                parameters=[{"use_sim_time": True, "yaml_filename": map_yaml,
                             "frame_id": "map"}],
            ),
            Node(
                package="nav2_lifecycle_manager", executable="lifecycle_manager",
                namespace=prior_namespace, name="lifecycle_manager_static_prior", output="screen",
                parameters=[{"use_sim_time": True, "autostart": True,
                             "node_names": ["static_prior_map_server"]}],
            ),
        ]
        reference_map_remappings = [("map", "planning_prior/map")]

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
        remappings=[
            ("/tf", "tf"), ("/tf_static", "tf_static"),
            # The shared display follows the actual DOG input: Point-LIO in
            # C1/C2, registered truth in the original truth-only entry.
            ("pointlio/cloud_registered_map", perception_cloud_topic),
        ] + reference_map_remappings,
    )

    return [
        LogInfo(
            msg=(
                f"Starting HERO perception simulation in '{costmap_mode}' costmap mode "
                f"with '{lidar_mode}' lidar input on '{field}'"
            )
        ),
        LogInfo(msg=f"Generated parameter file: {generated_params}"),
        LogInfo(msg=f"DOG high-column filter enabled={filter_high_columns}, "
                    f"minimum absolute map Z={high_column_min_z:.3f} m (not robot-relative)"),
        bridge,
        map_server,
        map_lifecycle,
        *reference_map_nodes,
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
                description="combined, static_only, dynamic_only, or split (static Smac / dynamic MINCO)",
            ),
            DeclareLaunchArgument(
                "lidar_mode",
                default_value="single",
                description="single (official simulator image) or dual (custom dual-lidar image)",
            ),
            DeclareLaunchArgument(
                "field",
                default_value="rmuc_2025",
                description="Static map selection: rmuc_2025, rmuc_2026, or rmuc_2027",
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
            DeclareLaunchArgument(
                "filter_high_columns", default_value="false",
                description="Experimental DOG output filter for columns entirely above map Z threshold",
            ),
            DeclareLaunchArgument(
                "high_column_min_z", default_value="0.30",
                description="Lowest voxel lower-face threshold, absolute map Z in metres; not relative to local ground",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
