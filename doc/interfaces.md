# Interfaces

This document lists node interfaces for `simple_nav_3d`.

## 1. Topic defaults

When topic parameters are left empty, defaults are generated under `robot.namespace`:

- odom: `/<robot>/odom_ground_truth`
- points: `/<robot>/rgbd_points`
- cmd_vel: `/<robot>/cmd_vel`
- goal: `/<robot>/goal_pose`
- active_goal: `/<robot>/simple_nav_3d/active_goal`
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

Publishes:

- `topics.active_goal` (`geometry_msgs/PoseStamped`)

Behavior:

- De-duplicates identical incoming goal positions.
- Periodically republishes active goal.
- Clears active goal when within final tolerance.

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
