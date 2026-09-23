# simple_nav_3d — code comment notes

Long comments from files in the `simple_nav_3d` package, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word. Files with many moved comments have their own notes doc next to this one.

One section per source file, in file order. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [CMakeLists.txt](#cmakeliststxt) — 1
- [include/simple_nav_3d/controllers/controller_base.hpp](#includesimple_nav_3dcontrollerscontroller_basehpp) — 2
- [include/simple_nav_3d/controllers/ugv_controller.hpp](#includesimple_nav_3dcontrollersugv_controllerhpp) — 1
- [include/simple_nav_3d/grid_utils.hpp](#includesimple_nav_3dgrid_utilshpp) — 1
- [include/simple_nav_3d/parameters.hpp](#includesimple_nav_3dparametershpp) — 3
- [src/controllers/ugv_controller.cpp](#srccontrollersugv_controllercpp) — 6
- [src/parameters.cpp](#srcparameterscpp) — 1
- [src/planners/uav_planner.cpp](#srcplannersuav_plannercpp) — 1
- [src/planners/ugv_planner.cpp](#srcplannersugv_plannercpp) — 3
- [src/simple_nav_navigator_node.cpp](#srcsimple_nav_navigator_nodecpp) — 4
- [test/test_goal_identity.cpp](#testtest_goal_identitycpp) — 2
- [test/test_ugv_planner.cpp](#testtest_ugv_plannercpp) — 5

## CMakeLists.txt

### cmake-gtest-no-target-guard

**No if(TARGET) guard on gtests** — in `Top level`, attached to `ament_add_gtest(test_ugv_planner test/test_ugv_planner.cpp)` (line 105)

```text
No if(TARGET ...) guard, deliberately, and matching explo_planner's
CMakeLists. ament_add_gtest skips target creation when gtest is missing,
so the guard turned "the test suite was never built" into a clean build
with zero tests — the failure mode is a green result that checked nothing.
Unguarded, the same situation is a hard CMake error naming the target.
```

## include/simple_nav_3d/controllers/controller_base.hpp

### ctrl-on-path-cleared

**The on_path_cleared hook contract** — in `ControllerBase — declarations`, attached to `virtual void on_path_cleared() {}` (line 24)

```text
Called on every tick where the global path is empty, i.e. the goal was
reached, cleared, or withdrawn.

It exists because the node does not call compute_command() at all on an
empty path, so a controller cannot observe that transition from inside
compute_command() — a check there is unreachable code. Anything a
controller latched *for the goal that just went away* has to be dropped
here or it silently carries into the next goal.

Called repeatedly while the path stays empty, so implementations must be
idempotent and must not log unconditionally.
```

### ctrl-has-pending-maneuver

**Why has_pending_maneuver exists** — in `ControllerBase — declarations`, attached to `virtual bool has_pending_maneuver() const {return false;}` (line 37)

```text
True while the controller is executing a self-directed manoeuvre that owns
the wheels and must not be interrupted part-way through (the UGV's
back-up-and-turn recovery is the only one today).

D2. It exists because the node gained a branch -- rotate-to-goal-yaw --
that can fire while the path is still non-empty, and that branch returns
before compute_command(). Without this query the node would silently
freeze a half-finished recovery: the manoeuvre's state is tick-counted, so
it would neither advance nor time out, and it would resume mid-stride
however many seconds later the rotation ended. Retiring the recovery
instead is not an option either -- on_path_cleared() is the only exit and
it logs `PATH CLEARED`, which would be false here and would corrupt the
entry/exit pairing that line exists to support.

Default false: a controller with no interruptible state is always
preemptible, which is the safe answer for the node's purposes.
```

## include/simple_nav_3d/controllers/ugv_controller.hpp

### ugv-recovery-tick-bound

**Why recovery is bounded by ticks** — in `UgvController — declarations`, attached to `int recovery_ticks_{0};` (line 56)

```text
Ticks spent inside the current recovery sequence, counted only on ticks
where compute_recovery_command actually ran.

WHY A TICK COUNT AND NOT A CLOCK. Both recovery phases exit on a physical
condition — BACKUP on distance travelled or a blocked rear, TURN on yaw
error — and neither can be reached by a robot that cannot move. A robot
wedged with a clear rear arc (rear_clr stays above kRearMinClearance
because the thing holding it is not in the map) commands reverse forever,
and nothing else in the stack can preempt it: this class already hoists
`recovery_active_` above the front-arc scan, which used to be the
accidental way out. So the sequence needs a bound of its own.

A tick count is that bound and not a timestamp because the guard must not
depend on odom.header.stamp being populated — a zero stamp would make an
elapsed-time cap read 0 s forever and silently disable the very check
that exists to stop a silent hang.

This used to note that a suspended recovery "should not age", since the
node only calls in on a fresh non-empty path. That reasoning was sound for
the tick unit but wrong about suspension: a recovery is no longer allowed
to survive an empty path at all (see on_path_cleared), so the only gaps a
tick count can now skip are stale-odom and stale-path ticks, where the
robot is not executing the recovery either.
```

## include/simple_nav_3d/grid_utils.hpp

### goal-identity-tolerances

**Goal identity and its tolerances** — in `same_goal_pose`, attached to `inline bool same_goal_pose(` (line 40)

```text
Is `b` the same navigation goal as `a`, to within the tolerances a
re-publish has to survive?

THE DEFECT THIS EXISTS TO FIX. The navigator's goal intake used to compare
`pose.position` only, with `==` on the three doubles, and ignore
`pose.orientation` entirely. An upstream goal revision that changed ONLY the
heading was therefore discarded at the intake and never reached the
controller — while the controller, two nodes downstream, documents the
opposite contract in its active-goal callback: "A change in ORIENTATION alone
is not [a new destination], because the tick below re-reads the target yaw
out of active_goal_ every time -- an in-place yaw revision is simply
tracked." That promise could not be kept, because the revision was filtered
out before it arrived.

It was LATENT, not observed: with an omnidirectional sensor model the
exploration planner drops its yaw arrival term outside EXPLOIT, its EXPLOIT
re-anchor publishes position and yaw together, and its homing arrival test is
distance-only — so no shipped path currently waits on a yaw-only update. It is
fixed anyway, because the next code that revises a heading in place would
fail silently and in a way that looks like a controller bug.

WHY TOLERANCES RATHER THAN `==`. The old exact comparison meant a goal
republished through any float round-trip (a different message, a transform,
a serialisation) read as a NEW goal, re-arming the navigator and restarting
its accept/reached cycle.

WHY 1e-5 m AND NOT 1e-6. The positional tolerance was 1e-6 m, justified as
"far above any round-trip noise". That is true near the origin and FALSE at
the coordinates this campaign actually publishes, because float32 spacing
scales with magnitude: a half-ULP at |x| = 12 m is 4.8e-7 m, but at |x| = 75 m
it is 3.8e-6 m and at |x| = 100 m it is still 3.8e-6 m — several times OVER
the old tolerance. The shipped ROI is x in [-51.3, 100.9], y in [-38.7, 74.5],
so the old guard covered roughly the innermost 13 m of a 150 m box and let a
pure round-trip re-arm the navigator everywhere else. It was not caught
because the round-trip test picked x = 12.345678, one of the coordinates where
the check cannot bite.

1e-5 m holds for |coordinate| < 256 m (half-ULP there is 7.6e-6 m; the next
binade up, at 256 m, costs 1.5e-5 m and would break it). That is 2.5x the
largest shipped ROI bound. A world larger than +/-256 m needs this raised —
test/test_goal_identity.cpp pins that precondition so it fails loudly rather
than silently degrading into the bug above.

THE YAW TOLERANCE STAYS AT 1e-6 rad, and that is measured, not inherited: the
worst yaw error over a full -pi..pi sweep through a float32 quaternion is
8.3e-8 rad, a 12x margin. Quaternion components are bounded by 1 regardless of
where the robot is, so the yaw term has no magnitude dependence to correct —
which is exactly why only the positional half was wrong.

Neither tolerance is a "close enough" judgement. 1e-5 m is 10 micrometres and
1e-6 rad is 0.2 arc seconds; both are far below anything a planner means to
express. The controller owns proximity, with ugv.goal_xy_tol_m.

The yaw term compares through normalize_angle so that -pi and +pi are the
same heading, which they are. An `==` on the raw quaternion would call them
different and re-arm on a goal that did not move at all.
```

## include/simple_nav_3d/parameters.hpp

### params-pipeline-role

**Pipeline role and its topics** — in `NodeParameters — declarations`, attached to `std::string pipeline_role;` (line 18)

```text
Pipeline role: "global" or "local". Drives which input map / output path
topic the planner uses, and which path topic the controller subscribes to.
- "global": planner reads `planning_map_topic`, publishes `global_path_topic`;
            controller subscribes to `global_path_topic`.
- "local":  planner reads `local_planning_map_topic`, publishes `local_path_topic`;
            controller subscribes to `local_path_topic`.
Two planner instances can run in parallel with different roles.
```

### params-goal-yaw-tol-declared

**Why ugv.goal_yaw_tol_rad is declared** — in `NodeParameters — declarations`, attached to `double ugv_goal_yaw_tol_rad;` (line 67)

```text
D2. Declared because launch/simple_nav_3d.launch.py has been SETTING
`ugv.goal_yaw_tol_rad` since the dscovox UGV pipeline was written, and an
undeclared parameter passed to a node that does not allow undeclared
overrides is dropped without a word. Every campaign cell therefore ran the
controller's rotate-to-goal branch against a hardcoded 0.15 rad while the
launch file said 0.2 -- a discrepancy no log line would ever have shown.
```

### params-local-corridor-radius

**Local planner corridor half-width** — in `NodeParameters — declarations`, attached to `double ugv_local_corridor_radius_m;` (line 93)

```text
Half-width of the corridor the local planner builds around the global
path. Cells outside the corridor are masked off so the local planner
refines within global's chosen homotopy. Used only when role==local and
a global path is available; if A* fails inside the corridor, the local
planner falls back to a free A* over the whole local map.
```

## src/controllers/ugv_controller.cpp

### ugv-arc-clearance

**compute_arc_clearance and its body frame** — in `compute_arc_clearance`, attached to `double compute_arc_clearance(` (line 87)

```text
Min bounding-box clearance to occupied cells inside an angular arc
centred on `center_angle_body` (body frame: 0 = forward, ±π = rear,
+π/2 = left, -π/2 = right). Used by the recovery state machine to check
rear clearance during BACKUP and to pick a turn direction when entering
recovery. Generalisation of compute_front_arc_stats.
```

### ugv-recovery-ends-on-empty-path

**An empty path ends recovery** — in `UgvController::on_path_cleared`, attached to `void UgvController::on_path_cleared()` (line 162)

```text
An empty global path ends any recovery in progress, and says so. Two reasons.

Correctness: the path empties when the goal is reached, cleared, or
withdrawn, and both pieces of recovery state were chosen for the OLD goal —
the backup distance is measured from the pose at entry, and the turn target
is an ABSOLUTE yaw computed once from the obstacle that was blocking then.
Previously the recovery merely suspended (the node stops calling
compute_command on an empty path) and resumed on the next non-empty path.
That resume steers toward a heading nothing has re-measured, from a
displacement origin the robot may have already left, and it owns the command
stream for up to the tick cap while doing it. Worse, a nav-budget expiry is
one of the most likely ways for a goal to be withdrawn, and a robot in
recovery is precisely a robot whose goal is about to time out — so this is
not a corner case, it is the common exit from a recovery.

Pairing: a suspended recovery also produced a `-> recovery:` entry with no
`recovery EXIT:`, which in a grep is indistinguishable from a recovery that
never terminated — the one failure mode the entry/exit pairing check exists
to detect. It reported a fault that had not happened while masking the one
that had.

This lives here, and not in compute_command's empty-path guard, because the
node does not call compute_command at all when the path is empty: a check
there is unreachable code that reads as if it works.
```

### ugv-recovery-before-front-arc

**Recovery checked before the front arc** — in `UgvController::compute_command`, attached to `if (recovery_active_) {` (line 280)

```text
Recovery owns the command stream until both phases complete. Checked here,
ahead of the front-arc scan, because the scan's no-obstacle early return
below would otherwise drop out of an in-progress recovery: a robot that has
just backed away from the obstacle that triggered recovery often sees a
clear arc, which used to abandon the sequence before the turn ever ran.
```

### ugv-proximity-trigger

**The recovery proximity trigger** — in `UgvController::compute_command`, attached to `if (clearance < params_.ugv_avoidance_hard_stop_distance_m) {` (line 320)

```text
The proximity trigger. It did not fire once in 72 campaign robot-runs at
the old 0.15 m threshold, including runs where a robot sat immobilised for
ten minutes, because the slowdown scaling immediately above decays the
commanded speed toward zero *before* clearance reaches the threshold: the
robot asymptotes into a creep and never crosses it. Generation 5 raises
ugv.avoidance_hard_stop_distance_m to 0.4 m so the trigger sits inside the
band the robot can actually reach while still commanding motion.

The cost is that it will also fire on genuinely tight-but-passable gaps.
That is a known and accepted trade, not an oversight — an unreachable
recovery is worth less than one that occasionally fires early, and the
pilot gate measures the entry rate per robot-run before any full campaign
commits to it.
```

### ugv-two-phase-recovery

**The two-phase recovery sequence** — in `UgvController::compute_recovery_command`, attached to `geometry_msgs::msg::Twist UgvController::compute_recovery_command(` (line 380)

```text
Two-phase deterministic recovery:
  BACKUP: drive straight backwards until we've travelled kBackupDistance
          OR rear clearance gets too tight (give up the backup early).
  TURN:   rotate in place to the absolute target yaw chosen at recovery
          entry (current yaw ± 90 deg).
On TURN completion, clears recovery_active_ so normal control resumes
the next tick. The phase progression terminates deterministically — no
time-based cooldown, no oscillation hysteresis needed.
```

### ugv-recovery-timeout-abandon

**Abandoning a timed-out recovery** — in `UgvController::compute_recovery_command`, attached to `fprintf(stderr,` (line 414)

```text
Abandon rather than hang. Handing the robot back to normal control does
not pretend the obstacle is gone: if it is still inside the hard-stop
range the next tick re-enters recovery, which is a bounded, logged,
observable cycle instead of a silent permanent reverse. The escalation
beyond that belongs to the exploration planner, which times the goal out
on nav_max_timeout_sec and blacklists it.
```

## src/parameters.cpp

### nav-ugv-goal-yaw-tol-default

**Why the UGV goal yaw tolerance default** — in `load_and_validate_params`, attached to `p.ugv_goal_yaw_tol_rad = node.declare_parameter<double>("ugv.goal_yaw_tol_rad", 0.2);` (line 126)

```text
D2. Default matches the UAV counterpart in spirit but is set to the value
the launch file was already trying to pass, so declaring it changes nothing
for the dscovox pipeline and gives every other caller the same number the
controller used to hardcode (rounded from 0.15 to the launch's 0.2).
```

## src/planners/uav_planner.cpp

### uav-height-penalty

**UAV nominal height penalty** — in `UavPlanner::compute_plan`, attached to `const double nominal_z = params_.uav_nominal_height_m;` (line 256)

```text
Nominal height penalty: penalize cells that deviate from the preferred
flight altitude. This biases the path toward the nominal height while
still allowing altitude changes when obstacles require it.
The penalty blends between start Z, nominal Z, and goal Z based on
progress along the path, so start/goal altitudes are respected.
```

## src/planners/ugv_planner.cpp

### ugv-goal-endpoint-fast-path

**Goal endpoint fast path** — in `best_goal_endpoint_cell`, attached to `if (occupied[flatten(goal_raw, width)] == 0 && reachable[flatten(goal_raw, width)] != 0) {` (line 315)

```text
Fast path: the requested goal cell is itself free and reachable.

This is provably the same answer the scan below would produce. The scan
minimises heuristic(c, goal_raw), and the goal cell scores 0 — no other
cell can beat it. It clears clear_toward_goal() trivially (zero-length
ray), and it clears the min-progress filter by construction: that filter
is either 0, or max(3, 0.2*d) with d > 6, and the goal cell's distance
from the start IS d.

Worth a special case only because of what it skips. The scan is a full
width*height sweep running a Bresenham ray per candidate cell; on the
375x375 global grid that is on the order of 1e7 cell visits, every replan,
for the common case where the goal is plainly reachable.
```

### ugv-carve-escape-corridor

**Carving an escape corridor from a halo** — in `carve_escape_corridor`, attached to `void carve_escape_corridor(` (line 409)

```text
When the robot is parked inside an inflated obstacle halo (e.g. next to
a tree), the start cell ends up surrounded by occupied cells and A* can't
find any path out — flood fill returns {start} only and the planner emits
an empty path. We can't safely free the entire halo (the inflation exists
for a reason), but we *can* carve a minimum-length corridor from the start
to the nearest originally-free cell. The robot is physically *here*, so
any cells it passes through to escape were always passable.

Strategy: BFS from start that walks *through* occupied cells, recording
parent pointers. The first originally-free cell we touch is the escape
breakthrough; we then walk the parent chain back and free only those
cells. Bounded to max_carve_cells so the planner can't free arbitrary
chunks of the map if the robot is genuinely deep inside an obstacle.
```

### ugv-escape-carve-budget

**Escape carve budget** — in `UgvPlanner::compute_plan`, attached to `const int max_carve_cells = std::max(` (line 591)

```text
If the robot is parked inside an inflated obstacle halo (next to a tree,
pinned against a wall, etc.), all neighbours of the start cell are also
occupied and A* would return an empty path. Carve the minimum corridor
out so the planner has somewhere to escape to. Budget the carve to
~2 m so we never free arbitrary chunks of the map.
```

## src/simple_nav_navigator_node.cpp

### nav-ignore-same-goal

**Ignoring a re-publish of the same goal** — in `SimpleNavNavigatorNode`, attached to `if (has_active_goal_ && same_goal_pose(active_goal_.pose, msg->pose)) {` (line 28)

```text
Ignore a re-publish of the goal already being driven. ORIENTATION IS
PART OF THE COMPARISON: this test used to be position-only, which
silently discarded every yaw-only goal revision before the controller
could see it. See same_goal_pose() for why that mattered and why it
was latent rather than observed.
```

### nav-new-goal-vs-rearm

**New goal versus re-arm** — in `SimpleNavNavigatorNode`, attached to `const bool is_rearm =` (line 36)

```text
Distinguish a genuinely NEW goal from a re-arm of the one just
finished. Both are accepted — re-arming is how a goal published while
this node was down gets picked up at all — but they are not equally
interesting, and conflating them made the robot log unreadable: after
arrival `has_active_goal_` is false, so an upstream that re-publishes
an unchanged goal (the planner's keep-alive, or its EXPLOIT re-anchor
at tick rate while the controller rotates) produced an
accepted/reached INFO pair per re-publish — up to 10 per second, for
as long as the rotation lasted.
```

### nav-repeat-arrival-log

**Repeat arrivals log at DEBUG** — in `on_tick`, attached to `const bool repeat = reached_goal_valid_ &&` (line 95)

```text
Same reasoning as the re-arm branch above: an upstream that keeps
re-publishing an unchanged goal after arrival makes this fire once per
re-publish. The FIRST arrival on a given goal is the interesting one
and stays at INFO; the repeats drop to DEBUG. `active_goal_` is still
intact here (only the flag was cleared), so the comparison is against
the goal that was just reached.
```

### nav-had-goal-ever

**Why had_goal_ever_ exists** — in `SimpleNavNavigatorNode — declarations`, attached to `bool had_goal_ever_{false};` (line 127)

```text
A goal has been accepted at some point, so `active_goal_` holds a real
pose. Distinct from has_active_goal_, which is cleared on arrival: the
re-arm test below needs "what was the last goal" AFTER it stopped being
active, and reading a default-constructed active_goal_ before the first
goal would make the origin-with-identity-orientation compare equal to a
real goal at the origin.
```

## test/test_goal_identity.cpp

### test-goal-identity-background

**Goal identity tests background** — in `File scope`, attached to `#include <gtest/gtest.h>` (line 4)

```text
THE DEFECT. The intake used to compare `pose.position` with `==` on three
doubles and ignore `pose.orientation` outright, so a goal revision that
changed only the heading was discarded before the controller could see it —
while the controller, two nodes downstream, documents the opposite contract:
"A change in ORIENTATION alone is not [a new destination], because the tick
below re-reads the target yaw out of active_goal_ every time -- an in-place
yaw revision is simply tracked." The controller was written to a promise the
intake broke.

It was LATENT, and that is stated here so the fix is not later misread as
having changed a result: with the omnidirectional sensor model the
exploration planner drops its yaw arrival term outside EXPLOIT, its EXPLOIT
re-anchor publishes position and yaw together in one message, and its homing
arrival test is distance-only. No shipped path waits on a yaw-only update.
The fix removes a trap for the next one that does.

The second half of these tests is about the OTHER direction — a goal that did
NOT change must compare equal even after a float round-trip, because the old
exact `==` made any round-trip read as a new goal and re-armed the navigator.
```

### test-round-trip-world-scale

**Round-trip test at world scale** — in `SameGoalPose.FloatRoundTripNoiseIsNotARevision`, attached to `TEST(SameGoalPose, FloatRoundTripNoiseIsNotARevision)` (line 132)

```text
A goal that survived a float round-trip is STILL the same goal. The old
intake used `==`, so any re-serialisation read as new, re-armed the navigator
and restarted its accept/reached cycle on a goal nobody had revised.

THE COORDINATES ARE THE POINT. This test used to run at x = 12.345678, where a
half-ULP of float32 is 4.8e-7 m and the then-default 1e-6 m tolerance had room
to spare. The shipped ROI is x in [-51.3, 100.9], y in [-38.7, 74.5], and
float32 spacing scales with magnitude: at x = 75.4 the same round trip costs
1.5e-6 m and at y = -88.37 it costs 2.7e-6 m, both OVER the old tolerance. The
test passed for the whole time the predicate was broken across ~90% of the
world, because the fixture sat in the one region where the check could not
bite. It now runs where the robots actually drive.
```

## test/test_ugv_planner.cpp

### ugv-planner-test-purpose

**Why the UGV planner tests exist** — in `File scope`, attached to `#include <gtest/gtest.h>` (line 3)

```text
WHY THIS FILE EXISTS. For 66 out of 66 robot-logs in the mr1 campaign the
nav global planner never produced a single path: it subscribed to a
`planning_map` topic nothing published, so `latest_map_` stayed empty and
every tick returned before A* ran. That defect is fixed in the LAUNCH file
(the global planner now reads dscovox's `global_planning_map`), and a
launch-wiring defect cannot be caught by a unit test — the runtime gate for
it is the new "global plan ok:" heartbeat in the nav log, paired with the
"no map" starvation WARN, so presence and absence are both observable in the
same binary.

What CAN be pinned down here is the thing the fix hands the planner: given a
map, does the planner actually plan, and does the generation-5 endpoint fast
path return the same answers the full scan did? These are the regression
guards for that.
```

### ugv-test-snapshot-occupancy

**Read occupancy from the snapshot** — in `cell_occupied`, attached to `bool cell_occupied(const MapSnapshot & snap, int cx, int cy)` (line 100)

```text
Read occupancy back out of the snapshot the planner will actually see, not
out of the OccupancyGrid we wrote. The two are separated by
snapshot_from_occupancy_grid's >= 50 threshold and its own indexing, and a
test that verifies its own fixture against the wrong one of those verifies
nothing. Out-of-bounds counts as occupied: the map edge is a wall.
```

### ugv-test-seal-box-cell-space

**Sealing the box in cell space** — in `seal_box`, attached to `void seal_box(nav_msgs::msg::OccupancyGrid & g, int x0, int y0, int x1, int y1, int thickness)` (line 123)

```text
Seal a rectangular region by filling every cell of a `thickness`-cell ring
around the closed cell range [x0,x1] x [y0,y1].

In CELL space, deliberately. The previous fixture drew each side by stepping
along it in metres at half the resolution, which looks like the safer choice
and is not: it fills the four sides and silently misses the four corner
cells, so what it built was a box with open corners. Cells are what A*
searches, so the fixture has to be written in the same units the property is
about — and the negative control below then checks the whole ring rather
than four lines that happen not to meet.
```

### ugv-test-enclosed-goal-controls

**Enclosed-goal test needs both controls** — in `UgvGlobalPlanner.FullyEnclosedGoalYieldsNoPath`, attached to `TEST(UgvGlobalPlanner, FullyEnclosedGoalYieldsNoPath)` (line 274)

```text
A goal walled off from the robot has no reachable endpoint that makes
progress; the planner must say so rather than invent one behind the wall.

This test used to be vacuous in BOTH directions and is worth spelling out,
because it is the shape a lot of guards in this project decayed into. Its
only assertion sat inside `if (out.has_path)`, so a planner that never
planned at all — precisely the mr1 failure this file exists to guard —
passed it silently. And nothing checked that the box was actually sealed, so
a coordinate or resolution slip that left a gap would also pass, by planning
straight through the hole into the goal. It therefore needs both a NEGATIVE
control (the seal is real) and a POSITIVE one (the planner is alive on this
exact map), or "no path" means nothing.
```

### ugv-test-relaxed-path-check

**A relaxed path must stay outside** — in `TEST`, attached to `for (const auto & ps : out.path.poses) {` (line 329)

```text
Relaxation is allowed — the endpoint just has to be outside the seal.
Assert on every waypoint, not only the last: a path that tunnels through
the wall and comes back out would satisfy an endpoint-only check.
Tested in cell space against the interior the fixture actually sealed;
a metre-space bounding box of the OUTER ring also covers the free ground
just outside the corners, and flags a legal path as a violation.
```
