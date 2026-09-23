# simple_nav_controller_node.cpp — design notes and history

The long comments of `src/simple_nav_controller_node.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [SimpleNavControllerNode](#simplenavcontrollernode) — 1
- [on_tick](#on_tick) — 5
- [SimpleNavControllerNode — declarations](#simplenavcontrollernode--declarations) — 1

## SimpleNavControllerNode

### ctrl-yaw-from-active-goal

**Desired yaw comes from the active goal** — attached to `active_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(` (line 74)

```text
D2. The goal itself, not the path's last waypoint, is where the desired
yaw comes from now.

The path was never a sound channel for it. The planner stamps the goal
orientation onto the final waypoint only when that waypoint is already
near the goal, so the orientation is absent for most of the approach and
appears late; and the old latch here accepted it only if the quaternion
was non-identity, which makes a COMMANDED yaw of zero -- a perfectly
ordinary heading, and the one an unstamped waypoint also carries --
indistinguishable from "no yaw was requested". A goal facing +x therefore
never produced a rotation at all. Worse, the latch was never cleared when
the goal changed, so a rotation interrupted by a new goal stayed armed and
the robot would later turn to the OLD goal's heading on arriving at the
new one.

Reading the active goal directly removes all three problems: every pose on
this topic has a meaningful orientation, a new goal replaces the old one
by construction, and the yaw is available from the moment the goal is.
```

## on_tick

### ctrl-path-cleared-notice

**Telling the controller the goal is gone** — attached to `if (latest_path_.poses.empty()) {` (line 139)

```text
Tell the controller the goal is gone. It cannot see this for itself:
every path below either returns early or skips compute_command entirely
when the path is empty, so a controller only ever observes non-empty
paths and cannot drop state it latched for the goal that just ended.
Placed above the rotate-to-goal branch because that branch returns.
```

### ctrl-rotate-to-goal-trigger

**Rotate-to-goal trigger and goal latch** — attached to `const bool rotate_applicable =` (line 148)

```text
D2. Rotate-to-goal-yaw.

The trigger is proximity to the active goal, not an empty path. The old
condition worked only because the planner happens to publish one empty
path when it retires a goal; it therefore depended on that single message
arriving and on the controller's copy of the path being the one that went
empty. Proximity is the condition the upstream arrival test actually uses,
so triggering on it makes the two agree by construction instead of by
coincidence.

The empty path is NOT kept as a second trigger, and that is deliberate. A
path empties for reasons that have nothing to do with arriving: the
planner clears state when the navigator goes quiet, when a goal is
withdrawn, when its inputs starve. Rotating on any of those would spin the
robot in place wherever it happened to be standing -- possibly tens of
metres from the goal -- to face a heading that only means anything at the
goal. Proximity is not merely a better trigger than an empty path, it is
the precondition that makes the manoeuvre meaningful at all.

Nothing is lost by dropping it: rotation does not translate, so once the
robot is inside the radius it stays inside it, and the latched goal keeps
the test true for the whole episode even after the navigator falls silent.

WHY THE GOAL LATCH IS NEVER STALE-CHECKED: the navigator deliberately
STOPS publishing active_goal the moment the robot is within
final_goal_tolerance -- which is precisely when this branch needs the goal
most. Any freshness test on active_goal_ would therefore expire the target
in the middle of every rotation it is supposed to serve. The latch is
instead retired by events (a new goal position, alignment, or the backstop
below), and that is why the backstop has to exist.

UGV only. The test is planar and the rate limit it clamps against is the
UGV's; a UAV already servos its own yaw toward the path inside
UavController::compute_command, so there is nothing here for it to do.
```

### ctrl-rotate-defers-to-recovery

**Rotation waits for a recovery manoeuvre** — attached to `RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,` (line 199)

```text
Do not cut into a recovery. Rotating away mid-back-up would abandon
the manoeuvre in the state it was invoked to escape, and the recovery
is tick-counted so preempting it suspends rather than ends it. It
finishes or times out on its own within a bounded number of ticks,
and the rotation gets its turn afterwards.
```

### ctrl-rotate-backstop

**The rotate-to-goal backstop** — attached to `++rotate_backstops_;` (line 218)

```text
The latch is deliberately immune to staleness (see above), so
without this bound a rotation whose goal is never superseded would
be commanded forever. What ends an episode normally is a NEW goal
POSITION arriving on active_goal -- the planner's next goal, relayed
by the navigator. This fires when no such goal comes for 30 s.

THE WARN DOES NOT DIAGNOSE, and must not be read as if it did. Two
very different things reach this line and look identical from here:
  - the mission ended. The planner stops issuing goals, so nothing
    supersedes the latched one. This is HEALTHY and, in generation
    9, expected: the planner no longer gates arrival on yaw at all
    (`yaw_required = (phase_ == EXPLOIT) || !fov_is_omnidirectional_`
    in explo_planner_node.cpp is false once the FOV is
    omnidirectional and exploitation is off), so its own
    `failGoal("budget-rotate")` deadline is UNREACHABLE by
    construction and this backstop is the only rotate bound left;
  - the planner died mid-mission. A genuine fault.
Separate them with the planner's own liveness, never with this
count: a nonzero rotate_backstops_ at the end of a clean run is the
normal reading, not a defect.
```

### ctrl-raw-map-size-check

**Raw and inflated grids must match** — attached to `if (raw_snapshot.valid &&` (line 275)

```text
The two grids come from two independent publishers, so nothing in
the type system makes them the same size. Downstream, the UGV
controller indexes occupied_raw with an index it derived from
map_snapshot.width -- so a mismatch is not a wrong answer, it is a
read past the end of the vector. The grids are expected to agree
(same node, same rolling window) and a disagreement means a
configuration fault, so it is reported rather than papered over,
and the inflated grid is left in place as the fallback.
```

## SimpleNavControllerNode — declarations

### ctrl-rotate-state-backstop-sizing

**Rotate state and backstop sizing** — attached to `bool has_active_goal_{false};` (line 352)

```text
D2. Rotate-to-goal state. The target comes from the active goal, not from
the path; see the subscription for why the path was the wrong source.

active_goal_ is held without a freshness test on purpose -- the navigator
stops publishing it exactly when the rotation needs it. kRotateBackstopSec
is the price of that: the only bound left on a rotation whose upstream has
gone silent. 30 s sits far above the interval at which a working planner
supersedes a goal it has not finished with (goal_republish_sec is 5 s, and
a republish of the SAME pose does not end an episode -- only a new POSITION
does), so it does not preempt normal operation.

It is NOT sized against the planner's goal_rotate_timeout_sec of 15 s.
That deadline was the original justification and it is unreachable in
generation 9 (see the backstop branch), so this is a backstop with nothing
behind it. Reaching it is therefore not by itself a fault -- mission end
reaches it too. The WARN says so; do not re-tighten the constant toward 15
on the strength of a deadline that no longer fires.
```
