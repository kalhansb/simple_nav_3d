# Moved comments: doc/simple_nav_3d.launch_notes.md
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

    # Comma-separated peer robot names. Empty (default) keeps single-robot
    # behaviour: dscovox_node subscribes only to its own scovox_bin. With peers,
    # the merger also reads each peer's, giving a per-robot fused team view.
    # (notes: launch-peers)
    peers_raw = LaunchConfiguration("peers").perform(context)
    peers = [p.strip() for p in peers_raw.split(",") if p.strip()]

    # Topic the merger reads each PEER's scovox binary from; default is the
    # peer's own publisher. Under the comms emulator it must be the relayed rx
    # copy, or map sharing bypasses the radio model. Self is never patterned.
    # (notes: launch-peer-bin-topic-pattern)
    peer_bin_pattern = LaunchConfiguration(
        "peer_bin_topic_pattern").perform(context)

    # scovox voxel edge length (m) on the lidar path, default 0.10. Voxel count,
    # every map consumer's cost and the ScovoxMapBinary delta payload grow as
    # res^-3; calibrate comms severity at the resolution you run.
    # (notes: launch-voxel-resolution-cost)
    voxel_res = float(LaunchConfiguration("voxel_resolution_m").perform(context))

    def dscovox_input_topics():
        return [f"/{robot}/scovox_node/scovox_bin"] + [
            peer_bin_pattern.format(peer=p, robot=p, self=robot)
            for p in peers
        ]

    # Optional world-fixed map from scovox_node on ~/global_planning_map for the
    # exploration planner; ~/planning_map stays the local planner's rolling
    # crop. Side must cover the ROI side plus margin; 0 = off.
    # (notes: launch-global-planning-map-scovox)
    plan_glob_size = float(
        LaunchConfiguration("global_planning_map_size_m").perform(context))
    plan_glob_res = float(
        LaunchConfiguration("global_planning_map_resolution").perform(context))
    plan_glob_period = float(
        LaunchConfiguration("global_planning_map_period_sec").perform(context))
    scovox_global_plan_params = {}
    if plan_glob_size > 0.0:
        scovox_global_plan_params = {
            "publish_global_planning_map": True,
            "global_planning_map_topic": "~/global_planning_map",
            "global_planning_map_size_m": plan_glob_size,
            # Centred on the world origin, matching the explo_planner ROI
            # convention (roi_min/max are symmetric about the origin in sim).
            "global_planning_map_origin_x": -0.5 * plan_glob_size,
            "global_planning_map_origin_y": -0.5 * plan_glob_size,
            "global_planning_map_resolution": plan_glob_res,
            "global_planning_map_period_sec": plan_glob_period,
            # Same inflation as the local map: explo_planner does a single-cell
            # free check and relies on the map already carrying the body radius.
            "global_planning_map_inflation_m": 1.5,
        }

    # The same envelope from the merger over the fused team grid, the
    # exploration planner's map domain; keep it identical to the scovox copy.
    # Topic is ~/global_planning_map, never ~/planning_map (the local crop).
    # (notes: launch-global-planning-map-dscovox)
    dscovox_global_plan_params = {}
    if plan_glob_size > 0.0:
        dscovox_global_plan_params = {
            "publish_global_planning_map": True,
            "global_planning_map_topic": "~/global_planning_map",
            "global_planning_map_size_m": plan_glob_size,
            "global_planning_map_origin_x": -0.5 * plan_glob_size,
            "global_planning_map_origin_y": -0.5 * plan_glob_size,
            "global_planning_map_resolution": plan_glob_res,
            "global_planning_map_period_sec": plan_glob_period,
            "global_planning_map_inflation_m": 1.5,
            # Same body slab as the scovox planning maps, so canopy is not
            # projected down onto the floor as an obstacle.
            "global_planning_map_min_z": 0.05,
            "global_planning_map_max_z": 1.0,
        }

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
            # Recovery trigger clearance (m). Must sit inside the band the robot
            # still reaches under power, and strictly below
            # avoidance_slowdown_distance_m or the slowdown span goes
            # non-positive. (notes: launch-hard-stop-distance)
            "ugv.avoidance_hard_stop_distance_m": 0.4,
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
        # scovox_node: per-robot persistent map. Ships only voxels dirtied since
        # the last publish (full snapshot on a fresh dscovox connection);
        # planning_map is a 20x20 m rolling crop around the robot.
        # (notes: launch-scovox-node-dscovox)
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
                **scovox_global_plan_params,
            }],
        ))

        # dscovox_node: this robot's own consensus merger (no central merger)
        # over its and its peers' scovox_bin, one source grid per robot keyed by
        # header.frame_id, fused at publish_rate_hz. Empty peers is
        # single-robot. (notes: launch-dscovox-node-merger)
        dscovox_inputs = dscovox_input_topics()
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
                # Must equal comms_sim_params.yaml's rx_qos_depth: on reconnect
                # the relay releases an outage's backlog at once and the
                # shallower end silently drops the excess. Only a history bound
                # without the emulator. (notes: launch-bin-qos-depth-dscovox)
                "scovox_bin_qos_depth": 4000,
                **dscovox_global_plan_params,
                # No planning_map_* params here: dscovox_node does not declare
                # them. Its planning-map geometry comes from
                # dscovox_global_plan_params.
                # (notes: launch-dscovox-no-planning-map)
            }],
        ))

    elif mapping == "dscovox_lidar":
        # Same topology as dscovox, but scovox_node integrates the lidar cloud
        # (geometric Beta occupancy, no semantics). Sensor model mirrors
        # scovox_lidar_geometric.yaml; share cadence mirrors
        # scovox_robot_share.yaml. (notes: launch-dscovox-lidar-topology)
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
                "resolution": voxel_res,
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
                **scovox_global_plan_params,
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
        # it fuses ScovoxMapBinary streams).
        dscovox_inputs = dscovox_input_topics()
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
                **dscovox_global_plan_params,
                # Must equal comms_sim_params.yaml's rx_qos_depth: on reconnect
                # the relay releases an outage's backlog at once and the
                # shallower end silently drops the excess. Only a history bound
                # without the emulator. (notes: launch-bin-qos-depth-lidar)
                "scovox_bin_qos_depth": 4000,
            }],
        ))

    # ── Navigation nodes (all namespaced under robot) ──────────────────

    # dscovox modes: the planners read their maps directly, so the costmap only
    # builds the sensor local_map the controller uses for emergency stops.
    # scovox mode: it forwards scovox_node's planning_map to the global planner.
    # (notes: launch-costmap-node-role)
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
    # In dscovox modes reads the merger's ~/global_planning_map, never the
    # body-centred ~/planning_map (stale once the robot moves); UAV also gets
    # the GetRegion service. Other modes use the costmap's global map.
    # (notes: launch-global-planner-map)
    global_planner_extra = {"pipeline.role": "global"}
    if mapping in ("dscovox", "dscovox_lidar"):
        global_planner_extra["topics.planning_map"] = (
            f"/{robot}/dscovox_node/global_planning_map")
        if is_uav:
            global_planner_extra["scovox_get_region_service"] = f"/{robot}/dscovox_node/get_region"

    nodes.append(Node(
        package="simple_nav_3d",
        executable="simple_nav_planner_node",
        namespace=robot,
        name="simple_nav_global_planner",
        output="screen",
        # Logger names are namespaced, so the selector must carry the robot
        # prefix. Without it the selector matches nothing and the level is
        # silently ignored — which is why this node's INFO lines appeared in
        # the campaign nav logs despite the "warn" here.
        arguments=["--ros-args", "--log-level", f"{robot}.simple_nav_global_planner:=warn"],
        parameters=[nav_params, global_planner_extra],
    ))

    # ── Local planner (dscovox only, UGV only) ───────────────────────
    # Reads scovox_node's rolling planning_map and the global path; its A*
    # targets where the global path leaves the window, masked to a
    # ugv.local_corridor_radius_m corridor, else free A*. Side-flip rejection is
    # off. (notes: launch-local-planner-corridor)
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
            # Robot-prefixed, same reason as the global planner above: an
            # unprefixed selector matches no logger and is silently ignored, so
            # this node has been running at INFO all along despite saying warn.
            arguments=["--ros-args", "--log-level", f"{robot}.simple_nav_local_planner:=warn"],
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
        # Robot-prefixed (see the planners above). The controller's recovery
        # ENTRY/EXIT lines are fprintf to stderr, not rclcpp logging, so they
        # are unaffected by this level and stay greppable in the nav logs.
        arguments=["--ros-args", "--log-level", f"{robot}.simple_nav_controller:=warn"],
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
        DeclareLaunchArgument(
            "voxel_resolution_m", default_value="0.10",
            description="scovox voxel edge length (m) on the lidar path. "
                        "Dominates map size and therefore every consumer's "
                        "cost: free space is carved along the whole ray to "
                        "max_range, so the voxel count grows as res^-3. 0.20 "
                        "is ~8x cheaper than the 0.10 default and shrinks the "
                        "ScovoxMapBinary delta payload by about as much - "
                        "which changes what a comms model has to carry, so "
                        "calibrate severity at the resolution you will run."),
        DeclareLaunchArgument(
            "peer_bin_topic_pattern",
            default_value="/{peer}/scovox_node/scovox_bin",
            description="Topic pattern for each PEER's scovox binary feeding "
                        "this robot's dscovox merger. Placeholders: {peer} "
                        "(alias {robot}) = peer name, {self} = this robot. "
                        "Default subscribes to the peers directly (no comms "
                        "model). Under the comms emulator use "
                        "'/{self}/rx/{peer}/scovox_node/scovox_bin' so map "
                        "sharing goes through the modelled link. Self is "
                        "always subscribed directly and is unaffected."),
        DeclareLaunchArgument(
            "global_planning_map_size_m", default_value="0.0",
            description="Side length (m) of scovox_node's second, WORLD-FIXED "
                        "planning map on ~/global_planning_map, centred on the "
                        "world origin. 0 = disabled (default). This is the map "
                        "an exploration planner consumes; ~/planning_map stays "
                        "the local nav planner's rolling crop either way. Must "
                        "be at least the explo_planner ROI SIDE (2 x its half-"
                        "extent), plus margin, or candidates outside the "
                        "envelope are rejected as occupied and the coverage "
                        "check measures the wrong area."),
        DeclareLaunchArgument(
            "global_planning_map_resolution", default_value="0.40",
            description="Cell size (m) of ~/global_planning_map. Coarser than "
                        "the local map on purpose: it is inflated by the body "
                        "radius and only used for single-cell free/occupied "
                        "checks and a reachability flood."),
        DeclareLaunchArgument(
            "global_planning_map_period_sec", default_value="1.0",
            description="Min seconds between ~/global_planning_map publishes. "
                        "The projection + inflation run on scovox_node's "
                        "integration thread, so this is a real-time budget, "
                        "not just bandwidth. 0 = every integration frame."),
        DeclareLaunchArgument("fine_band", default_value="false",
                              description="true = enable the fine-TSDF "
                              "refinement band on the scovox_node "
                              "(dscovox_lidar mode; mirrors "
                              "scovox_fine_band.yaml)."),
        OpaqueFunction(function=launch_setup),
    ])
