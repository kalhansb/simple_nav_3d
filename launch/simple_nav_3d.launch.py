"""
Unified simple_nav_3d launch file.

Usage:
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=dscovox
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=rama  mode:=uav mapping:=dscovox

Arguments:
  robot   - Robot name (default: atlas). Sets namespace + frame prefix.
  mode    - "ugv" or "uav" (default: ugv). Selects pipeline, sensor config, body size.
  mapping - "dscovox", "dscovox_lidar", "scovox", or "none" (default: dscovox).
            Selects mapping backend.
            "dscovox" runs scovox_node (local persistent grid, rolling planning_map)
                      + dscovox_node (global merger). RGB-D + segmentation input.
            "dscovox_lidar" same topology, but scovox_node integrates the
                      /<robot>/velodyne_points lidar cloud instead (geometric
                      only, no semantics; sensor model from
                      scovox/config/scovox_lidar_geometric.yaml). Nav costmap
                      points input switches to the lidar too.
            "scovox"  runs scovox_node alone in persistent mode (no merger).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robot = LaunchConfiguration("robot").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    mapping = LaunchConfiguration("mapping").perform(context)

    # Comma-separated list of peer robot names (e.g. "rama,charlie") for the
    # multi-robot DSCovox topology. Empty (default) preserves single-robot
    # behaviour bit-for-bit: dscovox_node only subscribes to its own
    # /<robot>/scovox_node/scovox_bin. With peers set, the merger also
    # subscribes to /<peer>/scovox_node/scovox_bin for each peer, giving
    # this robot a per-robot fused view of the whole team's mapping.
    peers_raw = LaunchConfiguration("peers").perform(context)
    peers = [p.strip() for p in peers_raw.split(",") if p.strip()]

    # fine_band:=true layers the fine-TSDF refinement-band overlay
    # (scovox/config/scovox_fine_band.yaml) onto the scovox_node. Regions
    # arrive on /<robot>/scovox_node/refinement_region; the fine cloud is
    # published on /<robot>/scovox_node/fine_tsdf_pointcloud.
    fine_band = LaunchConfiguration("fine_band").perform(context).lower() \
        in ("true", "1", "yes")

    is_uav = mode == "uav"

    # ── Common parameters for all simple_nav_3d nodes ──────────────────
    nav_params = {
        "use_sim_time": True,
        "robot.name": robot,
        "robot.mode": mode,
        "robot.namespace": f"/{robot}",
        # Empty topics → namespace-based defaults via with_fallback() in parameters.cpp
        "topics.odom": "",
        "topics.points": "",
        "topics.cmd_vel": "",
        "topics.goal": "",
        "topics.active_goal": "",
        "topics.global_path": "",
        "topics.local_path": "",
        "topics.local_map": "",
        "topics.global_map": "",
        "topics.robot_body_polygon": "",
        # Frames
        "frames.map": "map",
        "frames.odom": f"{robot}/odom",
        "frames.base_link": f"{robot}/base_link",
        # Sensors (common)
        "sensors.type": "rgbd",
        "sensors.expected_rate_hz": 15.0,
        "sensors.sensor_timeout_sec": 1.5,
        "sensors.sync_window_sec": 1.0,
        "sensors.max_range_m": 10.0,
        "sensors.voxel_size_m": 0.10,
        # Pipeline rate
        "pipeline.replan_rate_hz": 2.0,
        # Mission
        "mission.goal_timeout_sec": 120.0,
        "mission.progress_timeout_sec": 8.0,
        "mission.progress_min_distance_m": 0.20,
        "mission.use_sim_time": True,
        # Safety
        "safety.cmd_vel_timeout_sec": 0.5,
        "safety.stop_on_tf_failure": True,
        # Ground filter
        "ground_filter.bin_size_m": 0.5,
        "ground_filter.tolerance_m": 0.10,
        "ground_filter.valid_margin_m": 0.30,
    }

    # ── Mode-specific overrides ────────────────────────────────────────
    if is_uav:
        nav_params.update({
            "pipeline.planner": "planner_3d",
            "pipeline.controller": "flight_controller_uav",
            "pipeline.safety": "safety_air",
            # Filter out self-visibility (quadrotor arms at ~0.3m)
            "sensors.min_range_m": 0.5,
            "sensors.sensor_mount_height_m": 0.0,
            # Widen obstacle height band for flight altitude
            "sensors.min_obstacle_height_m": -10.0,
            "sensors.max_obstacle_height_m": 10.0,
            # Smaller body for quadrotor (~0.3m frame)
            "robot.body_length_m": 0.6,
            "robot.body_width_m": 0.6,
            # UAV flight params
            "uav.max_vx_mps": 2.0,
            "uav.max_vy_mps": 2.0,
            "uav.max_vz_mps": 1.5,
            "uav.max_yaw_rate_rps": 1.5,
            "uav.goal_xyz_tol_m": 0.3,
            "uav.goal_yaw_tol_rad": 0.25,
            "uav.nominal_height_m": 1.5,
        })
    else:
        nav_params.update({
            "pipeline.planner": "planner_2d",
            "pipeline.controller": "local_controller_ugv",
            "pipeline.safety": "safety_ground",
            "sensors.min_range_m": 0.25,
            "sensors.sensor_mount_height_m": 0.272,
            "sensors.min_obstacle_height_m": 0.10,
            "sensors.max_obstacle_height_m": 1.0,
            "robot.body_length_m": 1.0,
            "robot.body_width_m": 0.6,
            # UGV drive params
            "ugv.max_linear_vel_mps": 1.0,
            "ugv.max_angular_vel_rps": 0.5,
            "ugv.goal_xy_tol_m": 0.2,
            "ugv.goal_yaw_tol_rad": 0.2,
            "ugv.heading_kp": 1.5,
            "ugv.linear_kp": 0.8,
            "ugv.avoidance_hard_stop_distance_m": 0.15,
            "ugv.avoidance_slowdown_distance_m": 0.8,
            "ugv.avoidance_max_range_m": 3.0,
            "ugv.avoidance_arc_half_angle_deg": 45.0,
            "ugv.avoidance_turn_gain": 0.9,
            "ugv.forward_blind_zone_m": 0.55,
            "ugv.local_plan_window_size_m": 8.0,
            "ugv.local_plan_resolution_m": 0.20,
            "ugv.local_plan_inflation_m": 0.20,
            "ugv.obstacle_persistence_sec": 2.0,
            "ugv.global_map_size_m": 80.0,
            "ugv.global_map_resolution_m": 0.20,
            "ugv.global_map_origin_x_m": -40.0,
            "ugv.global_map_origin_y_m": -40.0,
            "ugv.replan_cost_threshold_m": 6.5,
            "ugv.side_flip_cooldown_sec": 3.0,
            "ugv.global_map_min_hits": 10,
        })

    # ── Lidar-input overrides (dscovox_lidar mode) ─────────────────────
    # The nav costmap's sensor-based local_map must read the lidar instead of
    # the (absent) rgbd cloud. sensors.type stays "rgbd" — parameters.cpp
    # validates the literal and nothing else reads it.
    if mapping == "dscovox_lidar":
        nav_params.update({
            "topics.points": f"/{robot}/velodyne_points",
            "sensors.sensor_mount_height_m": 0.716,   # velodyne z on base_link
            "sensors.max_range_m": 20.0,
            "sensors.min_range_m": 0.8,               # reject self-hits
            "sensors.expected_rate_hz": 10.0,
        })

    nodes = []

    # ── Mapping nodes ──────────────────────────────────────────────────
    if mapping == "scovox":
        nodes.append(Node(
            package="scovox_mapping",
            executable="scovox_mapping_node",
            namespace=robot,
            name="scovox_node",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "mode": "persistent",
                "trace_no_return_rays": True,
                "depth_topic": "rgbd_camera_depth_image",
                "depth_info_topic": "rgbd_camera_info",
                "seg_topic": "segmentation/colored",
                "integration_frame": f"{robot}/odom",
                "base_frame": f"{robot}/base_link",
                "scovox_topic": "~/scovox",
                "pointcloud_topic": "~/pointcloud",
                "publish_planning_map": True,
                "planning_map_topic": "~/planning_map",
                "planning_map_resolution": 0.20,
                # 30x30 m exploration ROI centred on the world origin. Must
                # match the eig_exploration_planner roi_* params.
                "planning_map_size_m": 30.0,
                "planning_map_origin_x": -15.0,
                "planning_map_origin_y": -15.0,
                "planning_map_min_z": 0.05,
                "planning_map_max_z": 1.0,
                "planning_map_inflation_m": 1.5,
            }],
        ))

    elif mapping == "dscovox":
        # scovox_node: per-robot persistent local map.
        # - Publishes ScovoxMapBinary snapshots to the dscovox merger. Only
        #   voxels touched since the last publish are shipped (dirty-set
        #   tracking); a fresh dscovox connection triggers a full snapshot.
        # - planning_map is published as a 20x20m rolling crop centered on
        #   the robot pose (mode=rolling). The underlying voxel grid is fully
        #   persistent — only the publication is windowed.
        nodes.append(Node(
            package="scovox_mapping",
            executable="scovox_mapping_node",
            namespace=robot,
            name="scovox_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[{
                "use_sim_time": True,
                "mode": "rolling",
                "trace_no_return_rays": True,
                "depth_topic": "rgbd_camera_depth_image",
                "depth_info_topic": "rgbd_camera_info",
                "seg_topic": "segmentation/colored",
                "integration_frame": f"{robot}/odom",
                "base_frame": f"{robot}/base_link",
                "scovox_topic": "~/scovox",
                "pointcloud_topic": "~/pointcloud",
                "clear_dynamic_voxels": False,
                "robot_id": robot,
                "publish_planning_map": True,
                "planning_map_topic": "~/planning_map",
                "planning_map_resolution": 0.20,
                "planning_map_window_size_m": 20.0,   # 10 m radius around robot
                "planning_map_min_z": 0.05,
                "planning_map_max_z": 1.0,
                "planning_map_inflation_m": 1.5,
            }],
        ))

        # dscovox_node: merges per-robot scovox snapshots into a global map.
        # Stores one source grid per robot keyed by header.frame_id of incoming
        # binaries; rebuilds the fused grid by additive Beta-conjugate
        # consensus merge across sources at publish_rate_hz.
        # Multi-robot fused view: subscribe to every team member's scovox_bin
        # (self + peers). Each robot's dscovox_node is its own per-robot
        # consensus merger -- there is no central merger. With peers=[] the
        # input list collapses to the single-robot case.
        dscovox_inputs = [f"/{r}/scovox_node/scovox_bin" for r in [robot] + peers]
        nodes.append(Node(
            package="scovox_mapping",
            executable="dscovox_mapping_node",
            namespace=robot,
            name="dscovox_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[{
                "use_sim_time": True,
                "input_topics": dscovox_inputs,
                "pointcloud_topic": "~/pointcloud",
                "map_frame": "map",
                "publish_rate_hz": 1.0,
                "publish_planning_map": True,
                "planning_map_topic": "~/planning_map",
                "planning_map_resolution": 0.20,
                # 60x60 m exploration ROI centred on the world origin. Must
                # match the eig_exploration_planner roi_* params so the
                # global planner is constrained to the same area.
                "planning_map_size_m": 60.0,
                "planning_map_origin_x": -30.0,
                "planning_map_origin_y": -30.0,
                "planning_map_min_z": 0.05,
                "planning_map_max_z": 1.0,
                "planning_map_inflation_m": 1.5,
            }],
        ))

    elif mapping == "dscovox_lidar":
        # Same rolling-mapper + per-robot-merger topology as "dscovox", but
        # scovox_node integrates the lidar cloud (geometric Beta occupancy,
        # no semantics). Sensor model mirrors
        # scovox/config/scovox_lidar_geometric.yaml; share cadence mirrors
        # scovox_robot_share.yaml (2 Hz coalesced deltas on the wire).
        scovox_fine_extra = {}
        if fine_band:
            # Mirrors scovox/config/scovox_fine_band.yaml (base 0.10 m ->
            # fine 0.025 m, trunc 7.5 cm, slab brackets breast height).
            scovox_fine_extra = {
                "fine_ratio_log2": 2,
                "fine_sdf_trunc_voxels": 3,
                "fine_region_margin": 0.15,
                "fine_raw_returns": True,
                "fine_anchor_enable": True,
                "fine_anchor_min_points": 12,
                "fine_anchor_max_shift": 0.30,
                "fine_region_z_lo": 1.0,
                "fine_region_z_hi": 1.6,
                "publish_fine_tsdf_pointcloud": True,
            }
        scovox_lidar_params = [{
                "use_sim_time": True,
                "mode": "rolling",
                # THE input switch: non-empty pointcloud topic selects the
                # lidar path; fuse_lidar_rgbd=false drops every depth/seg sub.
                "input_pointcloud_topic": f"/{robot}/velodyne_points",
                "fuse_lidar_rgbd": False,
                # gz PointCloudPacked has no per-point time field; "off" also
                # skips the IMU subscription + lidar-imu extrinsic lookups.
                "deskew_mode": "off",
                "integration_frame": f"{robot}/odom",
                # On the non-fused lidar path base_frame IS the ray origin.
                "base_frame": f"{robot}/velodyne",
                "scovox_topic": "~/scovox",
                "pointcloud_topic": "~/pointcloud",
                "robot_id": robot,
                # Lidar sensor model (scovox_lidar_geometric.yaml)
                "resolution": 0.10,
                "w_occ": 8.0,
                "w_free": 4.0,
                "carve_band": -1.0,        # full-ray free-space carve
                "min_range": 1.0,
                "max_range": 20.0,
                "range_decay_length": -1.0,
                "grazing_angle_threshold": -1.0,
                "enable_tsdf": False,
                # Required for real-time full-ray carve on a dense scan.
                "downsample_voxel_size": 0.10,
                # Wire-stream cadence to the mergers (scovox_robot_share.yaml).
                # No share_roi_z clip: full vertical extent in the fused map,
                # matching the rgbd campaign.
                "share_change_gate": True,
                "share_rate_hz": 2.0,
                "publish_planning_map": True,
                "planning_map_topic": "~/planning_map",
                "planning_map_resolution": 0.20,
                "planning_map_window_size_m": 20.0,
                "planning_map_min_z": 0.05,
                "planning_map_max_z": 1.0,
                "planning_map_inflation_m": 1.5,
            }]
        if scovox_fine_extra:
            scovox_lidar_params.append(scovox_fine_extra)
        nodes.append(Node(
            package="scovox_mapping",
            executable="scovox_mapping_node",
            namespace=robot,
            name="scovox_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=scovox_lidar_params,
        ))

        # Per-robot merger, identical to the "dscovox" one (sensor-agnostic —
        # it fuses ScovoxMapBinary streams). The planning_map_* params the
        # rgbd block passes are undeclared no-ops in dscovox_node, dropped here.
        dscovox_inputs = [f"/{r}/scovox_node/scovox_bin" for r in [robot] + peers]
        nodes.append(Node(
            package="scovox_mapping",
            executable="dscovox_mapping_node",
            namespace=robot,
            name="dscovox_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[{
                "use_sim_time": True,
                "input_topics": dscovox_inputs,
                "pointcloud_topic": "~/pointcloud",
                "map_frame": "map",
                "publish_rate_hz": 1.0,
            }],
        ))

    # ── Navigation nodes (all namespaced under robot) ──────────────────

    # Costmap node:
    # - dscovox mode: the global planner subscribes directly to the dscovox
    #   planning_map and the local planner subscribes directly to the
    #   scovox_node planning_map, so the costmap does NOT forward any external
    #   map. It only builds the sensor-based local_map used by the controller
    #   for emergency stops.
    # - scovox mode: forward scovox_node's planning_map as the global planner's
    #   input (no separate local planner in this mode).
    costmap_extra = {}
    if mapping == "scovox":
        costmap_extra["topics.external_local_map"] = f"/{robot}/scovox_node/planning_map"

    nodes.append(Node(
        package="simple_nav_3d",
        executable="simple_nav_costmap_node",
        namespace=robot,
        name="simple_nav_costmap",
        output="screen",
        parameters=[nav_params, costmap_extra] if costmap_extra else [nav_params],
    ))

    # ── Global planner ────────────────────────────────────────────────
    # In dscovox mode the global planner reads dscovox's merged planning_map
    # directly (no costmap forwarding). For UAV it uses the 3D GetRegion
    # service instead of a 2D map. In other modes it falls back to the
    # costmap-built global map via topics.planning_map default.
    global_planner_extra = {"pipeline.role": "global"}
    if mapping in ("dscovox", "dscovox_lidar"):
        global_planner_extra["topics.planning_map"] = f"/{robot}/dscovox_node/planning_map"
        if is_uav:
            global_planner_extra["scovox_get_region_service"] = f"/{robot}/dscovox_node/get_region"

    nodes.append(Node(
        package="simple_nav_3d",
        executable="simple_nav_planner_node",
        namespace=robot,
        name="simple_nav_global_planner",
        output="screen",
        arguments=["--ros-args", "--log-level", "simple_nav_global_planner:=warn"],
        parameters=[nav_params, global_planner_extra],
    ))

    # ── Local planner (dscovox only, UGV only) ───────────────────────
    # Reads scovox_node's 20x20m rolling planning_map AND the global planner's
    # output path. On each replan it slices the global path to the segment
    # inside the local window, uses that exit point as its A* target, and
    # masks the local map to a corridor of half-width ugv.local_corridor_radius_m
    # around the slice. This refines global within the local window without
    # contradicting it. If the corridor is blocked (new obstacle on the global
    # path) the local planner retries with a free A*. Side-flip rejection is
    # still disabled here because the corridor already serves the same role.
    if mapping in ("dscovox", "dscovox_lidar") and not is_uav:
        local_planner_extra = {
            "pipeline.role": "local",
            "topics.local_planning_map": f"/{robot}/scovox_node/planning_map",
            "topics.local_path": f"/{robot}/simple_nav_3d/local_path",
            "ugv.side_flip_cooldown_sec": 0.0,
            "ugv.local_corridor_radius_m": 2.0,
        }
        nodes.append(Node(
            package="simple_nav_3d",
            executable="simple_nav_planner_node",
            namespace=robot,
            name="simple_nav_local_planner",
            output="screen",
            arguments=["--ros-args", "--log-level", "simple_nav_local_planner:=warn"],
            parameters=[nav_params, local_planner_extra],
        ))

    # ── Controller ────────────────────────────────────────────────────
    # In dscovox UGV mode the controller follows the local planner's output;
    # otherwise it follows the global planner's output.
    if mapping in ("dscovox", "dscovox_lidar") and not is_uav:
        controller_extra = {
            "pipeline.role": "local",
            "topics.local_path": f"/{robot}/simple_nav_3d/local_path",
        }
    else:
        controller_extra = {"pipeline.role": "global"}

    nodes.append(Node(
        package="simple_nav_3d",
        executable="simple_nav_controller_node",
        namespace=robot,
        name="simple_nav_controller",
        output="screen",
        arguments=["--ros-args", "--log-level", "simple_nav_controller:=warn"],
        parameters=[nav_params, controller_extra],
    ))

    nodes.append(Node(
        package="simple_nav_3d",
        executable="simple_nav_navigator_node",
        namespace=robot,
        name="simple_nav_navigator",
        output="screen",
        parameters=[nav_params],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="atlas",
                              description="Robot name (sets namespace and frame prefix)"),
        DeclareLaunchArgument("mode", default_value="ugv",
                              description="Robot mode: ugv or uav"),
        DeclareLaunchArgument("mapping", default_value="dscovox",
                              description="Mapping backend: dscovox, scovox, or none"),
        DeclareLaunchArgument("peers", default_value="",
                              description="Comma-separated peer robot names "
                              "for multi-robot DSCovox topology. Empty = "
                              "single-robot (self only)."),
        DeclareLaunchArgument("fine_band", default_value="false",
                              description="true = enable the fine-TSDF "
                              "refinement band on the scovox_node "
                              "(dscovox_lidar mode; mirrors "
                              "scovox_fine_band.yaml)."),
        OpaqueFunction(function=launch_setup),
    ])
