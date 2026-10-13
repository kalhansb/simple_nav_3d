# Interfaces

This document lists node interfaces for `simple_nav_3d`.

## 1. Topic defaults

When topic parameters are left empty, defaults are generated under `robot.namespace`:

- odom: `/<robot>/odom_ground_truth`
- points: `/<robot>/rgbd_points`
- cmd_vel: `/<robot>/cmd_vel`
- goal: `/<robot>/goal_pose`
- active_goal: `/<robot>/simple_nav_3d/active_goal`
- goal_route: `/<robot>/goal_route`
- active_route: `/<robot>/simple_nav_3d/active_route`
- global_path: `/<robot>/simple_nav_3d/global_path`
- local_path: `/<robot>/simple_nav_3d/local_path`
- local_map: `/<robot>/simple_nav_3d/local_map`
- local_map_raw: `<local_map>_raw`
- global_map: `/<robot>/simple_nav_3d/global_map`
- robot_body_polygon: `/<robot>/simple_nav_3d/robot_body_polygon`

## 2. Node interfaces

### 2.1 simple_nav_navigator_node

Subscribes:

- `topics.goal` (`geometry_msgs/PoseStamped`)
- `topics.odom` (`nav_msgs/Odometry`)
- `topics.goal_route` (`nav_msgs/Path`, map frame): an ordered route of waypoints w0..w_last. `header.stamp` is the route id.

Publishes:

- `topics.active_goal` (`geometry_msgs/PoseStamped`)
- `topics.active_route` (`nav_msgs/Path`): the remaining waypoints w_k..w_last, under the held route's header (so it carries the route id). Published with `active_goal` on every 50 ms tick while a route is active, once more ([w_last]) on the tick it completes, so a pass of the second-to-last waypoint in that tick still reaches explo, and nothing after it ends.

Behavior:

- De-duplicates identical incoming goal positions.
- Periodically republishes active goal.
- Clears active goal when within final tolerance.

Route following (option C of the route planner design, `DESIGN_route_planner.md` §6 in the campaign folder, cited as RP in the code). With no `goal_route` ever received, every path above runs exactly as before.

- **Intake.** A `goal_route` whose id (`sec`·1e9 + `nanosec`) equals the **last accepted** id is a keep-alive and is ignored, even after that route finished or was preempted, so a late keep-alive cannot revive it. A new id replaces any held route and sets k = 0. An empty route, or one with a non-finite x/y/z, is ignored with a WARN: it does not clear the held route and its id is not recorded. There is no clear-on-empty; a `goal_pose` (the brake) is the stop.
- **Active goal.** While a route is active, `active_goal` is w_k, with the route's header (frame and stamp = route id, constant across legs).
- **Pass.** On accept (when odom has arrived) and on every tick before the arrival test: while w_k is not the last waypoint and the robot is within `route.pass_radius_m` of it, k advances. Only w_k is tested (no skip-ahead), and several waypoints can pass in one tick; only the last of them is published. Distance is 2D for UGV, 3D for UAV, as for arrival.
- **End.** The last waypoint uses the normal final-tolerance arrival; the navigator then stops publishing both topics. Because the pass radius must exceed the final tolerance, arrival can only fire on the last waypoint.
- **Preempt.** A `goal_pose` ends any active route *before* the same-goal test, so even a goal equal to w_k ends the route (the robot then carries on to w_k as a single goal).
- **Log lines** (INFO; `id` is `sec.nanosec`, `wp` and `at_wp` are the 0-based index k over the route size n):
  - `Navigator accepted route id=<id> wps=<n> first=(x= y=)`
  - `Navigator route leg: id=<id> wp=<k>/<n> passed=<m> (x= y= z= yaw=)`, one per tick that passed any waypoint; the pose is the new w_k.
  - `Navigator route end: id=<id> reason=<completed|goal_pose|replaced> at_wp=<k>/<n> passes=<total> forced_replans=<count>`. `forced_replans` counts the ticks (and the accept) whose pass moved w_k by more than max(0.20, 2·goal tolerance), a copy of the planner's goal-change reset test; chained passes in one tick count once.

### 2.2 simple_nav_costmap_node

Subscribes:

- `topics.odom` (`nav_msgs/Odometry`)
- `topics.points` (`sensor_msgs/PointCloud2`)
- optional `topics.external_local_map` (`nav_msgs/OccupancyGrid`, transient local)

Publishes:

- `topics.local_map` (`nav_msgs/OccupancyGrid`, transient local)
- `topics.local_map_raw` (`nav_msgs/OccupancyGrid`, transient local)
- `topics.global_map` (`nav_msgs/OccupancyGrid`, transient local)
- `<topics.local_map>_obstacle_cloud` (`sensor_msgs/PointCloud2`)

Behavior:

- Enforces odom/points freshness timeout and sync window.
- Builds local map from sensor data.
- Publishes global map from either:
  - integrated local observations, or
  - forwarded external map.

### 2.3 simple_nav_planner_node

Subscribes:

- `topics.odom` (`nav_msgs/Odometry`)
- map input topic (`nav_msgs/OccupancyGrid`, transient local):
  - global role: `topics.planning_map`
  - local role: `topics.local_planning_map`
- `topics.active_goal` (`geometry_msgs/PoseStamped`)
- local role only: `topics.global_path` (`nav_msgs/Path`)

Publishes:

- path output (`nav_msgs/Path`):
  - global role: `topics.global_path`
  - local role: `topics.local_path`

Services used:

- UAV mode only: `scovox_get_region_service` (`scovox_msgs/srv/GetRegion` client)

Behavior:

- Supports global/local role selection.
- Supports UGV and UAV planner implementations.
- Caches and can reuse previous paths on replan failure.

### 2.4 simple_nav_controller_node

Subscribes:

- `topics.odom` (`nav_msgs/Odometry`)
- `topics.local_map` (`nav_msgs/OccupancyGrid`, transient local)
- `topics.local_map_raw` (`nav_msgs/OccupancyGrid`, transient local)
- path input (`nav_msgs/Path`):
  - global role: `topics.global_path`
  - local role: `topics.local_path`

Publishes:

- `topics.cmd_vel` (`geometry_msgs/Twist`)

Behavior:

- Path-following command generation.
- Obstacle-aware slowdown/hard stop behavior per controller type.

## 3. Launch-level external mapping interfaces

When mapping is enabled, launch file can start mapping nodes from `scovox_mapping` package.

### 3.1 scovox mapping node (external)

Launched executable:

- `scovox_mapping_node`

Used planning map topic examples:

- `/<robot>/scovox_node/planning_map`

### 3.2 dscovox mapping node (external)

Launched executable:

- `dscovox_mapping_node`

Used planning map topic examples:

- `/<robot>/dscovox_node/planning_map`

Used service example for UAV:

- `/<robot>/dscovox_node/get_region`

## 4. QoS notes

- Map subscriptions/publications are configured as reliable + transient local in planner/costmap/controller where map latching behavior is needed.
- Sensor point cloud subscription in costmap uses sensor-data QoS.
- Path/goal/odom interfaces use standard queue depth subscriptions.
