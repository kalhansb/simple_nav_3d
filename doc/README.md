# simple_nav_3d Documentation

This folder contains detailed documentation for the `simple_nav_3d` package.

## Documentation index

- [Feature Catalog](features.md)
- [Interfaces (Topics, QoS, Services)](interfaces.md)
- [Parameter Reference](parameters.md)
- [Extension Guide](extension_guide.md)

## Package summary

`simple_nav_3d` is a Nav2-like ROS 2 navigation skeleton for a single robot in simulation.

Supported runtime modes:

- `ugv`: 2D planning + ground controller
- `uav`: 3D planning + flight controller

Supported mapping integrations:

- `dscovox`: local `scovox` + merged global `dscovox`
- `scovox`: `scovox` only
- `none`: no mapping backend nodes launched

Core nodes in this package:

- `simple_nav_navigator_node`
- `simple_nav_costmap_node`
- `simple_nav_planner_node`
- `simple_nav_controller_node`

## Quick start

Example launches:

```bash
# UGV + dscovox
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=dscovox

# UAV + dscovox
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=rama mode:=uav mapping:=dscovox

# UGV + scovox only
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=scovox

# Nav only, no mapping backend
ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv mapping:=none
```

Build:

```bash
colcon build --packages-select simple_nav_3d
source install/setup.bash
```
