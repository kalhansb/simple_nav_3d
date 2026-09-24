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

    # Where this robot's merger listens for each PEER's scovox binary. The
    # default is the peer's own publisher, i.e. today's behaviour unchanged.
    # Under the message-level comms emulator the peer streams arrive instead on
    # its relayed copies, and repointing the merger at those is the whole
    # mechanism by which map sharing obeys the radio model: left on the direct
    # topics, every robot merges every peer's map instantly and perfectly, and
    # the comms arm of an experiment silently degenerates into the control arm
    # while still producing a full set of plausible results.
    #
    # Placeholders: {peer} (or {robot}) = the peer being subscribed to, {self} =
    # this robot. The emulator's relay convention is
    #   "/{self}/rx/{peer}/scovox_node/scovox_bin".
    #
    # Self is deliberately NOT patterned. A robot's own binary never crosses a
    # radio link, and routing it through an rx topic would leave the robot
    # unable to see its own map whenever its own link was down — every arm would
    # then measure a mapping failure rather than a comms one.
    peer_bin_pattern = LaunchConfiguration(
        "peer_bin_topic_pattern").perform(context)

    # scovox voxel edge length (m) on the lidar path. Default 0.10 = the value
    # this launch has always used.
    #
    # It is the single biggest cost driver in a long run, and the cost is cubic:
    # the lidar path carves free space along the WHOLE ray (carve_band -1) out to
    # max_range 20 m, so every beam writes ~200 voxels at 0.10 m. Measured on a
    # 2-robot flatforest run, the fused map reached 12.7M voxels by t=550 s and
    # was still growing linearly with explored area. Everything that touches the
    # map scales with it -- dscovox integration, the full ScovoxMap publish, and
    # the planner's ingest plus whole-grid walk -- so past a few million voxels
    # the planner's map subscription simply stops keeping up. That failure is
    # silent and dangerous: one robot ran for three minutes on a frozen map,
    # still driving, still logging steps, its coverage curve flat while its
    # teammate's kept climbing.
    #
    # Raising this 0.10 -> 0.20 cuts the count ~8x. It also shrinks the
    # ScovoxMapBinary delta payload by about as much, which matters beyond
    # performance whenever the run is under a comms model: the deltas ARE what
    # the radio carries, so severity calibration must be done at the resolution
    # the campaign will actually run at, never carried over from another.
    voxel_res = float(LaunchConfiguration("voxel_resolution_m").perform(context))

    def dscovox_input_topics():
        return [f"/{robot}/scovox_node/scovox_bin"] + [
            peer_bin_pattern.format(peer=p, robot=p, self=robot)
            for p in peers
        ]

    # Second, world-fixed planning map published by scovox_node for an
    # EXPLORATION planner (explo_planner), on ~/global_planning_map.
    #
    # It cannot share ~/planning_map with the local nav planner: that one is a
    # 20 m robot-centred crop, and simple_nav_3d's local planner has no window
    # param of its own — the map extent IS its window, so widening it would put
    # the whole world through the local A*/corridor mask on the control path.
    # The exploration planner needs the opposite: a fixed envelope covering the
    # ROI, because it rejects candidates whose cell is out of bounds and
    # measures coverage termination over the ROI clipped to the grid. Hence two
    # publishers over the same voxel grid.
    #
    # Sizing: the envelope is world-fixed and centred on the world origin, and
    # so is the planner's ROI, so the side must be at least the ROI SIDE
    # (= 2 x roi half-extent) to cover it, plus margin for a robot that drifts
    # outside the ROI — its own cell must be in bounds or the reachability
    # flood starts nowhere. Default 0 = off, which is what every
    # non-exploration run wants.
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

    # The SAME world-fixed envelope, published by the MERGER over the FUSED
    # grid. scovox_node's copy above sees only what this robot measured itself;
    # dscovox_node's sees the team map, which is the domain the exploration
    # planner's candidates and its coverage-termination test actually live in.
    # Pointing the planner at the local map while it plans over the fused one
    # is a map-domain mismatch: a candidate in ground the PARTNER surveyed is
    # unknown-and-therefore-unreachable on the local map, so it is rejected.
    # Identical envelope and resolution to the scovox copy on purpose — the
    # two are meant to be comparable cell-for-cell, and the planner ROI is
    # sized against this side length.
    #
    # NOTE the topic: ~/global_planning_map, never ~/planning_map. Both the
    # exploration planner and — since the generation-5 fix — the nav global
    # planner subscribe to this name. ~/planning_map means the 20 m rolling
    # crop from scovox_node, which the LOCAL nav planner consumes; the two are
    # different maps with different jobs and must keep different names.
    dscovox_global_plan_params = {}
    if plan_glob_size > 0.0:
        dscovox_global_plan_params = {
            "publish_global_planning_map": True,
            "global_planning_map_topic": "~/global_planning_map",
            # Un-inflated twin of the above, published from the same tick. The
            # planner's DONE coverage test reads this one; the inflated grid
            # scores never-sensed cells as known. dscovox_node ONLY -- the
            # scovox_node block above does not declare this parameter and would
            # refuse to start with it.
            "global_coverage_map_topic": "~/global_coverage_map",
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
    # Pilot (fine-band on/off pair): the lattice ratio, the refined slab and
    # the anchor snap were hard-coded for a 0.10 m base map. At the campaign's
    # 0.20 m base, k=2 gives 5 cm; k=3 restores 2.5 cm.
    fine_ratio_log2 = int(LaunchConfiguration("fine_ratio_log2").perform(context))
    fine_z_lo = float(LaunchConfiguration("fine_region_z_lo").perform(context))
    fine_z_hi = float(LaunchConfiguration("fine_region_z_hi").perform(context))
    fine_anchor = LaunchConfiguration("fine_anchor_enable").perform(context).lower() \
        in ("true", "1", "yes")

    # Lookout experiment: the lidar renders to 100 m, and mapping + nav read
    # the 25 m-cropped copy (explo_planner/sim/lidar_crop.py) when this is set.
    # Empty (default) keeps /<robot>/velodyne_points.
    lidar_points = LaunchConfiguration("lidar_points_topic").perform(context) \
        or f"/{robot}/velodyne_points"
    # Nav global map side (m), centred on the world origin. 80 (default) is the
    # old hard-coded -40..40 m map; a lookout 90 m out needs a larger one.
    nav_map_size = float(LaunchConfiguration("nav_global_map_size_m").perform(context))

    is_uav = mode == "uav"

    # ── Common parameters for all simple_nav_3d nodes ──────────────────
    nav_params = {
        "use_sim_time": True,
        "robot.name": robot,
        "robot.mode": mode,
        "robot.namespace": f"/{robot}",
        # Empty topics → namespace-based defaults via with_fallback() in parameters.cpp
        # Pilot: "" = /<robot>/odom_ground_truth; the pose-noise pair points
        # the whole nav stack at /<robot>/odom_noisy instead.
        "topics.odom": LaunchConfiguration("odom_topic").perform(context),
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
            # Generation 5: raised 0.15 -> 0.4. At 0.15 the recovery state
            # machine was unreachable — it did not fire once in 72 campaign
            # robot-runs, including runs where a robot sat immobilised for ten
            # minutes, because the slowdown scaling above it
            #   scale = (clearance - hard_stop) / (slowdown - hard_stop)
            # drives commanded speed to zero as clearance approaches the
            # threshold, so the robot creeps to a halt just outside it and the
            # trigger is never crossed. 0.4 sits inside the band the robot can
            # still reach under power. It must stay strictly below
            # avoidance_slowdown_distance_m or the span goes non-positive.
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
            "ugv.global_map_size_m": nav_map_size,
            "ugv.global_map_resolution_m": 0.20,
            "ugv.global_map_origin_x_m": -0.5 * nav_map_size,
            "ugv.global_map_origin_y_m": -0.5 * nav_map_size,
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
            "topics.points": lidar_points,
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
                **scovox_global_plan_params,
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
                # Match the comms emulator's rx_qos_depth. On reconnect the
                # relay releases a whole outage's backlog in one pass; a
                # shallower reader here silently discards the excess and the
                # fused map is permanently holed with no counter recording it.
                # Harmless without the emulator — it is only a history bound.
                #
                # 500 -> 4000 (2026-08-16). At 500 this was the SHALLOW end of
                # the chain for the 250 stems/ha world: 861 s outages queue
                # ~1720 deltas at the ~2 Hz share rate, so all three dense cells
                # finished with their two merged maps 1.5-1.8 % apart and every
                # gate green. Keep this equal to comms_sim_params.yaml's
                # rx_qos_depth — raising only one end fixes nothing, because the
                # burst is discarded at whichever end is shallower.
                "scovox_bin_qos_depth": 4000,
                **dscovox_global_plan_params,
                # REMOVED: a planning_map_* block used to be passed here. Every
                # key in it was undeclared in dscovox_node, so ROS accepted the
                # values and nothing read them, while the nav global planner
                # subscribed to the ~/planning_map topic they described. Those
                # parameters made a dead topic look configured — the single
                # most misleading thing in this launch file. The global planner
                # now subscribes to dscovox's real ~/global_planning_map, whose
                # geometry comes from dscovox_global_plan_params above.
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
                "fine_ratio_log2": fine_ratio_log2,
                "fine_sdf_trunc_voxels": 3,
                "fine_region_margin": 0.15,
                "fine_raw_returns": True,
                "fine_anchor_enable": fine_anchor,
                "fine_anchor_min_points": 12,
                "fine_anchor_max_shift": 0.30,
                "fine_region_z_lo": fine_z_lo,
                "fine_region_z_hi": fine_z_hi,
                "publish_fine_tsdf_pointcloud": True,
            }
        scovox_lidar_params = [{
                "use_sim_time": True,
                "mode": "rolling",
                # THE input switch: non-empty pointcloud topic selects the
                # lidar path; fuse_lidar_rgbd=false drops every depth/seg sub.
                "input_pointcloud_topic": lidar_points,
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
                # Match the comms emulator's rx_qos_depth. On reconnect the
                # relay releases a whole outage's backlog in one pass; a
                # shallower reader here silently discards the excess and the
                # fused map is permanently holed with no counter recording it.
                # Harmless without the emulator — it is only a history bound.
                #
                # 500 -> 4000 (2026-08-16). At 500 this was the SHALLOW end of
                # the chain for the 250 stems/ha world: 861 s outages queue
                # ~1720 deltas at the ~2 Hz share rate, so all three dense cells
                # finished with their two merged maps 1.5-1.8 % apart and every
                # gate green. Keep this equal to comms_sim_params.yaml's
                # rx_qos_depth — raising only one end fixes nothing, because the
                # burst is discarded at whichever end is shallower.
                "scovox_bin_qos_depth": 4000,
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
    # In dscovox mode the global planner reads dscovox's merged planning map
    # directly (no costmap forwarding). For UAV it uses the 3D GetRegion
    # service instead of a 2D map. In other modes it falls back to the
    # costmap-built global map via topics.planning_map default.
    #
    # The map is ~/global_planning_map, NOT ~/planning_map. dscovox publishes
    # two grids and only the former is usable for global planning: the latter
    # is body-centred, so its origin moves with the robot and a plan is stale
    # the moment the robot drives. Pointing this node at ~/planning_map was a
    # silent no-op — nothing has ever published that name — and it left the
    # global planner inert for the whole campaign history while the local
    # planner drove alone on a 20 m horizon. That is the direct cause of the
    # local-minimum traps in sections 28 and 32.9. See the starvation warning
    # in simple_nav_planner_node.cpp, which now makes the same mistake loud.
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
        DeclareLaunchArgument("lidar_points_topic", default_value="",
                              description="Lidar cloud for mapping and the "
                              "nav costmap (dscovox_lidar); empty = "
                              "/<robot>/velodyne_points."),
        DeclareLaunchArgument("nav_global_map_size_m", default_value="80.0",
                              description="Nav global map side in m, centred "
                              "on the origin (80 = the old -40..40 map)."),
        DeclareLaunchArgument("odom_topic", default_value="",
                              description="Nav odometry topic; empty = "
                              "/<robot>/odom_ground_truth."),
        DeclareLaunchArgument("fine_ratio_log2", default_value="2",
                              description="Fine lattice: res_fine = "
                              "voxel_resolution_m / 2^k."),
        DeclareLaunchArgument("fine_region_z_lo", default_value="1.0",
                              description="Refined slab bottom, above each "
                              "region's base_z (m)."),
        DeclareLaunchArgument("fine_region_z_hi", default_value="1.6",
                              description="Refined slab top, above each "
                              "region's base_z (m)."),
        DeclareLaunchArgument("fine_anchor_enable", default_value="true",
                              description="Per-scan snap of in-region hits "
                              "onto the registered cylinder."),
        OpaqueFunction(function=launch_setup),
    ])
