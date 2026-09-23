# occupancy_grid_utils.hpp — design notes and history

The long comments of `include/simple_nav_3d/mapping/occupancy_grid_utils.hpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [snapshot_from_occupancy_grid](#snapshot_from_occupancy_grid) — 1
- [segment_free](#segment_free) — 6

## snapshot_from_occupancy_grid

### grid-occupied-raw-is-inflated

**Why occupied_raw is a copy** — attached to `out.occupied_raw = out.occupied;` (line 37)

```text
The grid arriving over the wire is already inflated and there is no raw
counterpart to recover, so occupied_raw is a copy and every consumer of it
is really reading inflated cells.

This is not an omission that can be closed here. dscovox_node's
publishGlobalPlanningMap writes 100 for a true obstacle and 100 again for
each dilated cell around it, on one topic, so the distinction does not exist
upstream to be transported. Recovering it means a second topic from a third
package. Consumers that want "true obstacle distance" therefore get a
conservative answer -- they treat the inflation halo as solid -- and that is
the safe direction for every current use, but a caller that needs the real
distance (rather than a lower bound on it) cannot get it from here.
```

## segment_free

### segment-free-exact-traversal

**segment_free: exact walk and corner ties** — attached to `inline bool segment_free(` (line 57)

```text
D2. Its one caller is the planner node's goal-snap: the decision of whether it
is safe to put the true goal back on the end of a path whose A* endpoint
quantised short of it.

EXACT, NOT SAMPLED. The obvious implementation walks the segment at some
fraction of a cell and tests each sample, and it is wrong in a way that only
shows up on the cases that matter: a segment clipping the corner of an
occupied cell can produce a chord shorter than the sample step and be missed
entirely, so the guard silently authorises a path through an obstacle exactly
when the obstacle is barely in the way. No step size fixes that -- halving it
halves the chord it misses. This is instead an Amanatides-Woo grid traversal,
which enumerates the cells the segment actually passes through, so the
question "how fine is fine enough" does not arise.

CORNER CROSSINGS, and why the tie is its own branch. A segment passing exactly
through the point where four cells meet leaves the current cell across both
boundaries at the same t. The loop used to resolve that with a bare `else`,
which stepped y and left x for the following iteration -- so the walk entered
ONE of the two cells that touch the segment only at that corner, and which one
depended on the direction of travel.

That made the predicate ASYMMETRIC: segment_free(a, b) and segment_free(b, a)
disagreed whenever exactly one shoulder was occupied. It was not an exotic
case. A 45-degree segment between two cell centres ties at EVERY corner it
crosses, and on the shipped 0.2 m grid a segment from the centre of (10,10) to
the centre of (13,13) reported "blocked" forwards and "free" backwards with a
single cell occupied -- in both directions, depending on which shoulder was
marked. A geometric predicate on an unordered pair of points must not depend
on the order it is handed them.

The tie is now resolved by checking BOTH shoulders and refusing if either is
occupied, then stepping diagonally. That is strictly MORE conservative than
the old behaviour -- it can only turn a yes into a no, never the reverse, so
it cannot introduce a drive-into-obstacle. That is the direction this function
has already chosen everywhere else ("Refusing is always safe ... while a wrong
yes drives into a cell nothing looked at").

THAT FIX ALONE IS NOT ENOUGH, and the reason is worth keeping. An exact
geometric corner is not always an exact floating-point tie. The segment from
the centre of cell (8,11) to the centre of (26,25) passes exactly through two
cell corners, and at both of them the computed gap between the two crossing
parameters is 2.8e-17 and 4.4e-16 rather than zero -- with OPPOSITE SIGNS
forwards and backwards, because the parameters are computed from whichever
endpoint is x0. So `==` never fired, `<` picked opposite shoulders, and the
predicate stayed asymmetric on a segment with no 45-degree symmetry to hint at
it. It was found by a sweep over angles, not by reasoning about the tie.

Symmetry is therefore delivered structurally, by ordering the endpoints before
the walk starts (see CANONICAL ENDPOINT ORDER in the body), not by the tie
branch. Widening the tie into an epsilon instead would have been the wrong
tool twice over: the gap that needs swallowing scales with the coordinates and
the segment length, so the epsilon would be a tuning parameter, and it would
still only cover the near-ties someone thought to measure.

The permissive reading -- that a zero-length intersection is not a crossing,
so neither shoulder need be checked -- is defensible geometry, and it is what
the comment here used to claim. It is rejected on two grounds. It was never
what the code did, so the claim was untested prose. And its safety rested on
the grid being obstacle-inflated, which is a property of the CALLER that this
function cannot verify; the conservative reading needs no such assumption.

There is deliberately no epsilon on the tie. With the endpoints ordered, a
segment that misses the corner by one ULP takes a normal step and traverses
one shoulder -- which IS checked, and which is the shoulder the segment
genuinely passes through, by a margin of about 1e-16 m. Missing the other one
by that distance on a 0.2 m inflated grid is not a safety claim anyone needs.
A tolerance here would buy nothing and would start swallowing genuine
single-cell diagonal gaps.
```

### segment-free-bounds-and-unknown

**segment_free: bounds, unknown cells, grid** — attached to `inline bool segment_free(` (line 126)

```text
OUT OF BOUNDS BLOCKS, which is the OPPOSITE of path_still_valid()'s
convention, and the difference is the point. That function skips OFF-WINDOW
waypoints because its job is to keep an already-searched path alive, and a
waypoint that has rolled off the rolling window is unobserved rather than
known-bad. This function authorises a NEW segment that no search ever
examined, so an OFF-WINDOW cell must refuse. Refusing is always safe -- the
robot parks where it parks today -- while a wrong yes drives into a cell
nothing looked at.

Read "off-window" strictly. The paragraph above is about cells OUTSIDE the
grid and says nothing about in-bounds cells the sensor has not yet observed.
Those read as FREE, deliberately: snapshot_from_occupancy_grid maps int8 -1
to 0 above (it is not >= 50) and this function inherits that. The rule is
pinned by TEST(SegmentFree, UnknownCellsReadAsFree) in test_goal_snap.cpp.

It is also the PIPELINE-WIDE convention rather than a local choice, which is
why it is not quietly "fixed" here. Every traversability test in the UGV
planner reads the same collapsed array: ugv_planner.cpp's
line_of_sight_free (:46 zero-step, :57 the ray), world_point_free (:108),
the A* expansion itself (:650), and all four tiers of the goal-relocation
search best_goal_endpoint_cell (:328 direct accept, :338 :366 :392 the
widening scans). simple_nav_planner_node.cpp's path_still_valid (:154)
tests >= 50 on the raw grid directly, which collapses -1 the same way.
A* therefore routes through unobserved space as a matter of course. Making
this one guard refuse on in-bounds unknown would make the 0.6 m append
strictly stricter than the A* that produced the path it is appending to:
the search would plan 20 m through unobserved cells and this function would
then refuse the final 30 cm of the same route. Wanting unknown to refuse is
a coherent position, but it is a change to the whole UGV planner -- and it
would bite hardest at frontier goals, which sit on the observed/unobserved
boundary by construction.

WHICH GRID: callers pass the inflated `occupied`, because `occupied_raw` is a
copy of it (see above) and an un-inflated grid does not exist to pass. That
makes the test strictly conservative: the appends it refuses that a raw grid
would have allowed are exactly those where the goal lies inside an obstacle's
inflation halo, which is the case where driving the last few centimetres is
least justified anyway.
```

### segment-free-non-finite

**Non-finite endpoints refuse explicitly** — attached to `if (!std::isfinite(x0) || !std::isfinite(y0) ||` (line 172)

```text
NON-FINITE COORDINATES REFUSE, explicitly. A NaN or infinite endpoint was
already rejected before this line existed, but only by accident: cell_of()
casts floor(NaN) to int, which is UNDEFINED BEHAVIOUR, and the walk survived
because the value it happens to produce on this platform is out of bounds and
blocked() refuses out of bounds. That is not a guarantee, it is a coincidence
that a compiler is entitled to withdraw -- and it is real, not theoretical:
built with -fsanitize=float-cast-overflow, the version without this guard
reports "nan is outside the range of representable values of type 'int'" and
the version with it reports nothing. Refusing here makes it a rule, and it
costs four comparisons on a function that then walks dozens of cells.

It also makes the tie branch below provably unreachable with a non-finite
crossing parameter, which is why that branch does not re-check.
```

### segment-free-endpoint-order

**Canonical endpoint order in segment_free** — attached to `if (x1 < x0 || (x1 == x0 && y1 < y0)) {` (line 191)

```text
CANONICAL ENDPOINT ORDER. Everything below is a walk whose branch decisions
are floating-point comparisons of the two crossing parameters, and those are
computed from x0/y0 -- so running the same segment backwards is a DIFFERENT
computation that can round the other way. Ordering the endpoints first makes
the traversal a function of the unordered pair, which is what the predicate
claims to be. See "CORNER CROSSINGS" above for the case that forced it.

Free to do: both endpoints are blocked-checked below regardless of order,
and the question "is this segment clear" has no direction in it.
```

### segment-free-step-cap

**The segment_free step cap** — attached to `const long max_steps =` (line 246)

```text
A pure DDA takes exactly |dgx| + |dgy| steps, minus one for each crossing
that happens to land on a cell corner (the tie branch below advances both
axes in one iteration, which is what spends two of that budget at once).
The cap is therefore still an upper bound, not an exact count, and
exceeding it means the walk failed to converge on the end cell -- which can
only be a floating-point pathology, and is answered with a refusal rather
than a loop, because refusing is the safe direction.
```

### segment-free-exact-tie

**The exact corner-tie branch** — attached to `if (blocked(gx + step_x, gy)) return false;` (line 265)

```text
EXACT TIE: the segment passes through the point where four cells meet.
See the "CORNER CROSSINGS" note above the function for why both
shoulders are checked and why this branch cannot be folded into either
of the two above.

Reaching here with a non-finite parameter is impossible, and that is by
construction rather than by hope: the endpoints are finite (checked at
the top), so a crossing parameter is a finite difference over a nonzero
finite step. The two-infinity case -- both step_x and step_y zero, where
this branch would advance nothing and burn the step budget -- needs
x0 == x1 and y0 == y1, and that segment returns at the `gx == gx_end &&
gy == gy_end` test on the first iteration, before ever getting here.
```
