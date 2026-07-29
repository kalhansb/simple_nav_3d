# Parameter Reference

Parameters are declared in `load_and_validate_params` and generally shared across nodes.

## 1. Robot and pipeline

- `robot.name` (string, default `robot1`)
- `robot.mode` (string, `ugv` or `uav`)
- `robot.namespace` (string, absolute namespace)
- `pipeline.role` (string, `global` or `local`)
- `pipeline.planner` (string)
- `pipeline.controller` (string)
- `pipeline.safety` (string)

Validation rules:

- UGV mode requires:
  - `pipeline.planner=planner_2d`
  - `pipeline.controller=local_controller_ugv`
  - `pipeline.safety=safety_ground`
- UAV mode requires:
  - `pipeline.planner=planner_3d`
  - `pipeline.controller=flight_controller_uav`
  - `pipeline.safety=safety_air`

## 2. Topic parameters

- `topics.odom`
- `topics.points`
- `topics.cmd_vel`
- `topics.goal`
- `topics.active_goal`
- `topics.global_path`
- `topics.local_path`
- `topics.local_map`
- `topics.external_local_map`
- `topics.global_map`
- `topics.planning_map`
- `topics.local_planning_map`
- `topics.robot_body_polygon`

Notes:

- Empty values fall back to namespace-derived defaults for most topics.
- `topics.local_planning_map` has no fallback and is required for planner local role.
- Topic strings are validated as absolute paths.

## 3. Frame parameters

- `frames.map` (default `map`)
- `frames.odom` (default `odom`)
- `frames.base_link` (default `base_link`)

## 4. Sensor and mapping input

- `sensors.type` (must be `rgbd`)
- `sensors.expected_rate_hz`
- `sensors.sensor_timeout_sec`
- `sensors.sync_window_sec`
- `sensors.min_range_m`
- `sensors.max_range_m`
- `sensors.voxel_size_m`
- `sensors.sensor_mount_height_m`
- `sensors.min_obstacle_height_m`
- `sensors.max_obstacle_height_m`

Validation highlights:

- `max_range_m > min_range_m`
- `max_obstacle_height_m > min_obstacle_height_m`

## 5. Robot body geometry

- `robot.body_length_m`
- `robot.body_width_m`

Used for:

- map inflation radius
- obstacle clearance checks
- UAV voxel inflation radius

## 6. UGV parameters

Velocity and goal tracking:

- `ugv.max_linear_vel_mps`
- `ugv.max_angular_vel_rps`
- `ugv.goal_xy_tol_m`
- `ugv.heading_kp`
- `ugv.linear_kp`

Controller avoidance and recovery behavior:

- `ugv.avoidance_hard_stop_distance_m`
- `ugv.avoidance_slowdown_distance_m`
- `ugv.avoidance_max_range_m`
- `ugv.avoidance_arc_half_angle_deg`
- `ugv.avoidance_turn_gain`
- `ugv.forward_blind_zone_m`

Local planner/map behavior:

- `ugv.local_plan_window_size_m`
- `ugv.local_plan_resolution_m`
- `ugv.local_plan_inflation_m`
- `ugv.obstacle_persistence_sec`
- `ugv.local_corridor_radius_m`

Global map accumulation behavior:

- `ugv.global_map_size_m`
- `ugv.global_map_resolution_m`
- `ugv.global_map_origin_x_m`
- `ugv.global_map_origin_y_m`
- `ugv.global_map_min_hits`

Global replanning behavior:

- `ugv.replan_cost_threshold_m`
- `ugv.side_flip_cooldown_sec`

Validation highlights:

- slowdown distance must be greater than hard-stop distance
- several parameters must be strictly positive
- some parameters allow zero/non-negative values (for example inflation/corridor/replan threshold)

## 7. UAV parameters

- `uav.max_vx_mps`
- `uav.max_vy_mps`
- `uav.max_vz_mps`
- `uav.max_yaw_rate_rps`
- `uav.goal_xyz_tol_m`
- `uav.goal_yaw_tol_rad`
- `uav.nominal_height_m`

Used by:

- 3D planner (goal interpretation and height preference)
- flight controller command limits

## 8. Ground filter parameters

- `ground_filter.bin_size_m`
- `ground_filter.tolerance_m`
- `ground_filter.valid_margin_m`

Used by local mapper's ground filtering pipeline.

## 9. Launch overrides and mode presets

`simple_nav_3d.launch.py` sets practical mode-dependent defaults and overrides for many parameters at launch time, including:

- pipeline plugin names
- sensor ranges/heights
- body dimensions
- UGV/UAV dynamic limits
- local/global planning map topics based on selected mapping backend

Refer to launch file for exact values used in each mode/backend combination.
