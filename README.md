# simple_nav_3d

Simple Nav2-like navigation skeleton for a single robot in Gazebo.

## Stage constraints

- Single robot runtime.
- One selected mode per run: `ugv` or `uav`.
- RGB-D point cloud is the only obstacle sensor.
- `cmd_vel` is the output command topic for both modes.
- Internal control path is separate for UGV and UAV.

## Package contents

- `src/simple_nav_navigator_node.cpp`: accepts goal and publishes active goal.
- `src/simple_nav_costmap_node.cpp`: builds and publishes local costmap.
- `src/simple_nav_planner_node.cpp`: computes and publishes global path.
- `src/simple_nav_controller_node.cpp`: tracks path and publishes `cmd_vel`.
- `launch/simple_nav_3d.launch.py`: unified launch (all params inline, no yaml).

## Documentation

Detailed package documentation is available in `doc/`:

- `doc/README.md`: documentation index and quick summary.
- `doc/features.md`: all implemented features and behaviors.
- `doc/interfaces.md`: topics, QoS, and service interfaces.
- `doc/parameters.md`: parameter groups, validation, and launch override notes.
- `doc/extension_guide.md`: planner/controller extension and scaffolding notes.

## Launch

```bash
# UGV (atlas) with dscovox (scovox local + dscovox merger)
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=dscovox

# UAV (rama) with dscovox
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=rama mode:=uav mapping:=dscovox

# UGV with scovox-only (no merger)
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=scovox

# Nav only, no mapping backend
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=none
```

Arguments:

- `robot` — Robot name (default: `atlas`). Sets namespace and frame prefix.
- `mode` — `ugv` or `uav` (default: `ugv`). Selects pipeline, sensor config, body size.
- `mapping` — `dscovox`, `scovox`, or `none` (default: `dscovox`). Selects mapping backend.

## Published visualization topics

- `topics.global_path` (`nav_msgs/Path`)
- `topics.local_path` (`nav_msgs/Path`)
- `topics.local_map` (`nav_msgs/OccupancyGrid`)
- `topics.robot_body_polygon` (`geometry_msgs/PolygonStamped`)

## Build

```bash
colcon build --packages-select simple_nav_3d
source install/setup.bash
```
