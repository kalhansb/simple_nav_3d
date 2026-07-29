# Extension Guide

This package already has factory-based extension points for planners and controllers.

## 1. Current extension points

Planner factory:

- selector function: `create_planner(...)`
- base interface: `PlannerBase`
- built-in implementations:
  - `UgvPlanner` as `planner_2d`
  - `UavPlanner` as `planner_3d`

Controller factory:

- selector function: `create_controller(...)`
- base interface: `ControllerBase`
- built-in implementations:
  - `UgvController` as `local_controller_ugv`
  - `UavController` as `flight_controller_uav`

## 2. Add a new planner

1. Add class header/source implementing `PlannerBase`.
2. Register planner name in planner factory.
3. Ensure mode/parameter validation allows the new planner name where needed.
4. Add launch parameter overrides if this planner needs custom defaults.
5. Build and test with planner node in both role combinations if relevant.

Minimal requirements for planner implementation:

- set `PlannerOutput.has_path`
- populate `PlannerOutput.path` with correct frame and timestamp
- handle invalid/empty map or goal input safely

## 3. Add a new controller

1. Add class header/source implementing `ControllerBase`.
2. Register controller name in controller factory.
3. Update parameter validation for mode-to-controller compatibility if needed.
4. Add launch defaults for controller-specific parameters.
5. Validate stale-input behavior (return safe zero command on missing data).

Minimal requirements for controller implementation:

- consume odometry and path robustly
- obey configured velocity limits
- return zero command when input state is not actionable

## 4. Add checker/recovery modules

Scaffold folders exist:

- `include/simple_nav_3d/checkers/`
- `include/simple_nav_3d/recoveries/`
- `src/recoveries/`

No checker/recovery plugin framework is currently wired into factories.

Recommended approach:

1. Define abstract interfaces (for example `CheckerBase`, `RecoveryBase`).
2. Add factory registration similar to planner/controller factories.
3. Inject these modules into planner/controller node execution flow.
4. Add parameters and launch controls for selecting checker/recovery strategies.

## 5. Design conventions observed in this package

- Parameters are centrally declared and validated in `load_and_validate_params`.
- Runtime behavior is often switched by `mode` and `pipeline.role`.
- Map topics use transient-local QoS when latched behavior is required.
- Safety fallback behavior favors safe stop or path reuse over risky motion.

Following these conventions will keep new features consistent with existing behavior.
