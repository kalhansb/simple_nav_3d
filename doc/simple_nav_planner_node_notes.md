# simple_nav_planner_node.cpp — design notes and history

The long comments of `src/simple_nav_planner_node.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [slice_global_path_for_local](#slice_global_path_for_local) — 1
- [apply_corridor_mask](#apply_corridor_mask) — 1
- [SimpleNavPlannerNode](#simplenavplannernode) — 3
- [SimpleNavPlannerNode — declarations](#simplenavplannernode--declarations) — 1
- [on_tick](#on_tick) — 9
- [SimpleNavPlannerNode — declarations (part 2)](#simplenavplannernode--declarations-part-2) — 2

## slice_global_path_for_local

### planner-slice-global-path

**Slicing the global path for local** — attached to `bool slice_global_path_for_local(` (line 168)

```text
Pick the slice of the global path that we want the local planner to refine.

Walks the global path from the waypoint nearest the robot forward, keeping
every waypoint that lies inside the local map. Stops at the first one that
exits the window — that exit point becomes the local target. If the global
path's actual endpoint is inside the window we use that endpoint instead so
the local planner finishes the goal cleanly.

Returns false if the global path has no waypoint inside the local window
(e.g., the global path is entirely outside the rolling crop, or empty).
```

## apply_corridor_mask

### planner-corridor-mask

**The local corridor mask** — attached to `MapSnapshot apply_corridor_mask(` (line 216)

```text
Stamp a corridor of half-width `radius_m` around the centerline into the
local map by marking every cell *outside* the corridor as occupied. This
hard-constrains the local A* to refine within global's homotopy. The
corridor is rasterized by walking the centerline at sub-cell resolution
and unioning the open cells around each sample point.

Cells that are already occupied stay occupied — the corridor mask never
frees a previously-blocked cell. The robot's own start cell is always
included so A* can start.
```

## SimpleNavPlannerNode

### planner-local-follows-global-path

**Local planner input from global path** — attached to `if (is_local_role_) {` (line 449)

```text
When running as the local planner, also listen to the global planner's
output. We use it both to pick the local target (the global path's exit
point from the rolling window) and to mask the local A* into a corridor
around global's chosen homotopy. If no global path is published the
local planner falls back to the user goal and a free A*.
```

### planner-drop-global-on-goal-change

**Dropping the global path on goal change** — attached to `has_global_path_ = false;` (line 480)

```text
Drop the global path too. It was planned to the OLD goal, and
the global planner publishes nothing when it fails to plan — so
without this the local planner would keep masking its A* into a
corridor around a route to a goal we have already left. That
silently biases every local plan toward the wrong homotopy for
as long as global keeps failing.
```

### planner-startup-banner-logger

**Startup banner on the liveness logger** — attached to `RCLCPP_INFO(` (line 520)

```text
On liveness_logger_, NOT get_logger(). The launch file sets this node's
own logger to warn, so on get_logger() this banner is invisible in every
campaign log — and it is the only line that says which map topic the
planner is watching. That is precisely how a global planner pointed at a
topic nobody publishes went unnoticed for the whole campaign history.
liveness_logger_ is a separate logger name and is deliberately not
covered by the per-node selectors, so this line and the periodic
"global plan ok" heartbeat always survive together: one says what was
wired, the other proves it produced work.
```

## SimpleNavPlannerNode — declarations

### planner-clear-plan-severity

**Severity of clearing the plan** — attached to `void clear_plan_state(const char * reason, bool anomalous = false)` (line 602)

```text
`anomalous` picks the severity, and the distinction matters more than it
looks: the campaign log level for this node is WARN, so an INFO line here
is not written to the per-cell nav log at all. Clearing on "goal reached"
is routine and stays INFO. Clearing because the navigator stopped sending
goals is the robot silently coasting to a halt with an empty path, and at
INFO that produced a nav log with no entry for it whatsoever — the run
looked idle rather than broken.
```

## on_tick

### planner-starvation-warning

**The planner starvation warning** — attached to `if (has_goal_ && (!has_odom_ || !map_ready)) {` (line 658)

```text
Starvation is the one waiting-state that is never normal. A planner
with a goal and a pose but no map has been asked to work and cannot,
and if its map topic has no publisher it will wait forever — silently,
because the line above is DEBUG and campaign logs run at WARN. That is
exactly how this node stayed inert across every run of the project
while still printing a healthy startup banner.

Grace period first: dscovox publishes nothing until its first fused
frame and only once a subscriber exists, so a few seconds of no map at
startup is expected and must not train anyone to ignore this.
The condition is "a goal is pending and something needed to serve it is
missing", not specifically the map. The map was the input that failed
historically, but scoping the warning to it left the symmetric case —
goal and map present, no ODOM — in exactly the silence this check
exists to end: an odom remap regression produces a nav log with two
startup banners and nothing after, while the exploration planner times
every goal out and the run reads as difficult terrain.

`has_goal_` still gates it. A planner with no goal is idle by design
between explore steps, and warning on that would fire on every healthy
run until nobody read the line.
```

### planner-starvation-clear-when-idle

**Clearing starvation when the goal goes** — attached to `clear_starvation();` (line 690)

```text
No goal pending: idle by design, not starving. Clear the state here
rather than leaving it to the full-readiness path below, which is
only reached once all three inputs are present. Without this, a
starvation that ends because the GOAL was withdrawn keeps its
counter, and the next fully-ready tick prints a recovery line
inflated by however long the planner sat idle — attributing a goal
gap to an input recovery. A liveness line that reports the wrong
cause is worse than none: it is how these checks stop being read.
```

### planner-global-replan-decimation

**Global replan decimation** — attached to `const bool is_global_ugv = !is_uav_ && !is_local_role_;` (line 722)

```text
Global-role replan decimation.

The global instance plans A* over the 150 m fused grid (375x375 cells at
0.40 m). Its input map only changes at 1 Hz — dscovox republishes
global_planning_map on a 1.0 s timer — so planning at the 10 Hz tick rate
recomputes the same answer nine times out of ten, on the biggest grid in
the system, for both robots, inside a cell budget that is already
1.149 x sim-time.

The gate is stamped on every ATTEMPT, not on every success. A failing
plan publishes nothing and leaves has_prev_path_ false, so a
success-stamped gate would degenerate to full tick rate in exactly the
case that costs the most: a blocked goal cell makes best_goal_endpoint_cell
run its full relaxation scan before giving up.

Two things still preempt the period: a goal change (force_replan_), and a
latched path that the newest map has just invalidated — deferring either
would mean steering along a route we already know is wrong.
```

### planner-local-homotopy-refine

**Local refinement inside global's homotopy** — attached to `geometry_msgs::msg::PoseStamped goal_for_planner = latest_goal_;` (line 761)

```text
For role=local, refine inside global's chosen homotopy:
  1. Slice the global path to the segment that lies in our local map.
  2. Use the slice's exit point as the A* target (instead of the user
     goal, which is usually outside the rolling window anyway).
  3. Mask the local map down to a corridor of half-width
     ugv.local_corridor_radius_m around the slice. The corridor
     forces the local A* to stay on global's side of every obstacle.
  4. If A* fails inside the corridor (the slice runs through a freshly
     observed obstacle), retry without the mask so the local planner
     can dodge. The retry will likely pick a different homotopy than
     global; the global planner will catch up on its next replan.
```

### planner-side-flip-global-only

**Side-flip rejection is global-only** — attached to `if (has_prev_path_ && !is_uav_ && !is_local_role_) {` (line 809)

```text
Side-flip rejection is a global-planner concern: it prevents the
robot from committing to one homotopy and then flipping to a worse
alternative. The local planner runs at higher rate over a smaller
window where the cost gradient is more reliable, so the cooldown
would only lock in stale paths. Skip it entirely for role=local.
```

### planner-goal-snap-append

**Appending the true goal to the path** — attached to `constexpr double kGoalSnapMaxM = 0.6;` (line 847)

```text
D2. Close the quantisation gap between the A* endpoint and the goal,
then stamp the goal orientation onto whatever waypoint ends up last.

Four roundings stack up between the planner's goal and where the robot
physically stops: the global A* endpoint snaps to a 0.40 m cell centre,
that endpoint becomes the local target, the local A* snaps it again to a
0.20 m cell centre, and the controller stops within its waypoint
tolerance of that. None of those steps is wrong on its own and none is
worth removing, but they compose: the median arrival across the campaign
parked 0.269 m from the commanded point. The planner upstream then
measures its own arrival against goal_xy_tolerance and is entitled to
call a stop short of that a failure -- and failGoal() blacklists the
position the robot is standing on, so the cost of a 27 cm shortfall is
not a retry, it is a poisoned cell.

The fix is to put the real goal back on the end of the path when it is
close enough that the gap is quantisation rather than a genuine
truncation, and only when the straight run to it is clear. The three
guards are all load-bearing:
  - !is_uav_: segment_free is a 2D test and would silently ignore z.
    The UAV planner keeps the behaviour it has today.
  - dg > kGoalSnapEpsM: A* already landed on the goal; appending a
    duplicate point would give the controller a zero-length final
    segment to compute a heading from.
  - dg <= kGoalSnapMaxM: past this the shortfall is not rounding. It
    means A* could not reach the goal (blocked cell, corridor mask, map
    edge) and extending the path would be inventing a route through
    terrain nothing has searched.
The snapshot used is the UNMASKED map_snapshot, never map_for_planner:
the corridor mask marks everything outside the corridor occupied, so
testing against it would refuse appends purely for being off-centreline.
```

### planner-stamp-goal-orientation

**Stamping goal orientation on the last waypoint** — attached to `if (!out.path.poses.empty()) {` (line 899)

```text
Stamp the goal orientation onto the last waypoint so the controller
can rotate in place to face the desired direction after reaching it.
Only stamp if the last waypoint is near the actual goal — the local
planner's path ends at an intermediate target, not the final goal.
After a successful append dg is 0, so this always fires on the new tip.
```

### planner-global-plan-heartbeat

**The global plan ok heartbeat** — attached to `if (is_global_ugv) {` (line 915)

```text
Positive heartbeat for the global instance. The whole failure this
release fixes was a planner that looked healthy because it only ever
printed a startup banner: absence of output was indistinguishable from
absence of work. A periodic line that can ONLY be printed after a real
plan makes "global planner is alive" checkable from the campaign logs
instead of inferable from silence.
```

### planner-goal-snap-counters

**Logging the goal-snap counters** — attached to `if (!is_uav_ && (goal_snap_appends_ > 0 || goal_snap_blocked_ > 0)) {` (line 929)

```text
D2. Both counters, on both roles, or the rule is unfalsifiable.

An append that never fires and an append that fires on every plan look
identical from the outside -- the path just ends where it ends. The
blocked count is the half that matters most: it is the only evidence
that the segment test is doing work rather than rubber-stamping, and if
it stays at zero across a campaign the guard is not a guard.

role= is in the line because BOTH planner instances log under the same
`<robot>.nav_liveness` name (that name is what survives the launch's
per-logger :=warn selector), so without it the global and local
instances' lines are indistinguishable in the campaign log.
```

## SimpleNavPlannerNode — declarations (part 2)

### planner-liveness-logger

**Why liveness uses a separate logger** — attached to `rclcpp::Logger liveness_logger_{rclcpp::get_logger("nav_liveness")};` (line 1012)

```text
Liveness lines go to a SEPARATE logger, and that is not cosmetic.

The launch file runs this node at `<robot>.simple_nav_global_planner:=warn`
— deliberately, because the planner's INFO chatter is per-replan. But the
positive half of the "did the global planner ever plan?" gate is an INFO
heartbeat, and a heartbeat that the log level eats is a check that has
stopped checking: it reads as PASS whether or not the planner works. That
is the same class of defect as the topic bug it exists to catch.

Escalating it to WARN would work but poisons the WARN stream that the
campaign failure counts are read from. `<robot>.nav_liveness` sits outside
the `<robot>.simple_nav_global_planner` subtree the selector names, so it
keeps the default level and the line survives at its honest severity.
```

### planner-replan-attempt-stamp

**Replan decimation state** — attached to `rclcpp::Time last_plan_attempt_{0, 0, RCL_ROS_TIME};` (line 1027)

```text
Replan decimation. Timestamped on every ATTEMPT, not on every success:
a planner that is failing must be rate-limited too, and failure publishes
nothing, so gating on a stored path would leave the failing case running
at the full 10 Hz tick rate — the one case where each attempt is most
expensive, because a blocked goal cell triggers the full endpoint
relaxation scan.
```
