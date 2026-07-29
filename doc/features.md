# Feature Catalog

This document describes all implemented features in `simple_nav_3d`.

## 1. Navigation architecture

The package is split into four ROS 2 nodes:

- `simple_nav_navigator_node`
  - Accepts incoming goal poses.
  - Republishes a persistent active goal for downstream nodes.
  - Clears the active goal when the robot reaches tolerance.
- `simple_nav_costmap_node`
  - Builds a local occupancy map from odometry + RGB-D point cloud.
  - Publishes both inflated and raw local maps.
  - Maintains/publishes a global occupancy map when external map forwarding is not enabled.
- `simple_nav_planner_node`
  - Runs as global or local planner depending on `pipeline.role`.
  - Selects planner implementation by factory (`planner_2d` or `planner_3d`).
  - Publishes path output (`global_path` or `local_path`).
- `simple_nav_controller_node`
  - Selects control implementation by factory (`local_controller_ugv` or `flight_controller_uav`).
  - Follows global or local path based on `pipeline.role`.
  - Publishes `cmd_vel`.

## 2. Runtime modes

### 2.1 UGV mode

- Planner: `planner_2d` (A* on 2D occupancy map).
- Controller: `local_controller_ugv`.
- Safety profile: `safety_ground`.
- Goal tolerance: XY distance.

UGV-specific implemented behavior:

- Obstacle-aware endpoint selection when requested goal cell is blocked.
- Line-of-sight path pruning.
- Chaikin smoothing + uniform path resampling.
- Side-flip rejection/cooldown in the global planner to reduce oscillation between homotopies.
- Local planner corridor refinement in `dscovox` mode (local path constrained around global slice).
- Deterministic recovery state machine in controller:
  - Phase 1: straight backup
  - Phase 2: +/-90 degree turn based on side clearance
- Front-arc obstacle clearance based slowdown and hard stop.

### 2.2 UAV mode

- Planner: `planner_3d`.
- Controller: `flight_controller_uav`.
- Safety profile: `safety_air`.
- Goal tolerance: XYZ distance.

UAV-specific implemented behavior:

- Periodic `GetRegion` service fetch from scovox/dscovox backend.
- Conversion of sparse voxels to inflated 3D occupancy grid.
- 3D A* with 26-connected neighbors.
- Height preference penalty toward nominal cruise altitude with smooth start/goal blending.
- 3D line-of-sight pruning.
- Fallback to straight-line path if voxel grid is unavailable or no 3D path is found.
- Controller supports 3-axis velocity and yaw-rate command generation.
- Directional local-map-based XY obstacle slowdown/hard-stop cone.

## 3. Mapping and map handling features

### 3.1 Local map construction

From point cloud + odometry, the local mapper performs:

- Range filtering by `sensors.min_range_m` and `sensors.max_range_m`.
- Transform from robot frame to odom frame.
- Ground segmentation using PCL RANSAC perpendicular-plane model.
- Ray-based clearing for non-ceiling points.
- Obstacle persistence over time (sticky obstacle memory with expiry).
- Sticky free-space evidence.
- Occupancy inflation using body footprint + extra inflation radius.
- Robot-centered free bubble to avoid immediate self-trapping.
- Obstacle-band debug point cloud publication.

### 3.2 Global map behavior

Costmap node can run in two map publication behaviors:

- Internal global map accumulation:
  - Integrates local maps into a fixed-size global grid.
  - Uses hit-count confirmation (`ugv.global_map_min_hits`) before promoting occupied cells.
  - Does not clear confirmed occupied cells with free observations.
- External map forwarding:
  - When `topics.external_local_map` is set, incoming map is forwarded to global planner topic.
  - In launch usage, this is used for `scovox` planning map forwarding.

## 4. Launch-time topology features

`simple_nav_3d.launch.py` supports dynamic topology from arguments:

- `robot`: namespace + frame prefix source.
- `mode`: `ugv` or `uav` parameter set.
- `mapping`: `dscovox`, `scovox`, or `none`.

Topology decisions:

- `dscovox`:
  - starts `scovox_mapping_node` and `dscovox_mapping_node`
  - global planner reads merged planning map
  - optional local planner instance (UGV only) reads rolling local planning map
  - controller follows local path (UGV local role) or global path otherwise
- `scovox`:
  - starts `scovox_mapping_node` only
  - costmap forwards scovox planning map as global planner input
- `none`:
  - runs only simple_nav_3d nodes

## 5. Planner robustness features

- Goal-change cache reset when active goal moves beyond reset threshold.
- Replan failure fallback to previous valid path when possible.
- Planner stale-goal protection (clears state when navigator stops publishing).
- Local role fallback from corridor-constrained planning to free local planning if corridor is blocked.

## 6. Parameter safety and validation features

`load_and_validate_params` enforces:

- Valid mode and matching pipeline triplets.
- `pipeline.role` in `{global, local}`.
- Absolute topic naming for configured topics.
- Positive/valid numeric bounds on sensor, planner, and controller parameters.
- Cross-parameter constraints (for example, slowdown distance > hard-stop distance).

## 7. Current scope and non-implemented placeholders

Implemented extension points today:

- planner factory: `planner_2d`, `planner_3d`
- controller factory: `local_controller_ugv`, `flight_controller_uav`

Placeholder directories exist but currently have no implementations:

- `include/simple_nav_3d/checkers/`
- `include/simple_nav_3d/recoveries/`
- `src/recoveries/`
