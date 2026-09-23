# test_goal_snap.cpp — design notes and history

The long comments of `test/test_goal_snap.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [File scope](#file-scope) — 2
- [SegmentFree.TinyCornerChordIsCaught](#segmentfreetinycornerchordiscaught) — 1
- [TEST](#test) — 1
- [File scope (part 2)](#file-scope-part-2) — 1
- [SegmentFree.TheCornerFixtureReallyTies](#segmentfreethecornerfixturereallyties) — 1
- [SegmentFree.ACornerCrossingIsSymmetric](#segmentfreeacornercrossingissymmetric) — 1
- [SegmentFree.ADiagonalSqueezeBetweenTwoCornersIsRefused](#segmentfreeadiagonalsqueezebetweentwocornersisrefused) — 1
- [SegmentFree.TheNearTieCornerIsSymmetric](#segmentfreetheneartiecornerissymmetric) — 1
- [SegmentFree.TheWalkConvergesOnTieHeavyAndDegenerateGeometry](#segmentfreethewalkconvergesontieheavyanddegenerategeometry) — 1
- [SegmentFree.NonFiniteCoordinatesRefuse](#segmentfreenonfinitecoordinatesrefuse) — 1
- [SegmentFree.UnknownCellsReadAsFree](#segmentfreeunknowncellsreadasfree) — 1

## File scope

### goal-snap-segment-free-oob-blocks

**Why segment_free is pinned here** — attached to `#include <gtest/gtest.h>` (line 4)

```text
WHY THIS FILE EXISTS. Across the ts1b campaign the median arrival parked
0.269 m from the commanded goal while the exploration planner judged arrival
against a 0.4 m tolerance -- and failGoal() blacklists the cell the robot is
standing on, so a shortfall is not a retry, it is a poisoned position. The
shortfall is quantisation: a 0.40 m global cell, a 0.20 m local cell, and a
controller stop tolerance, composed. D2 closes it by appending the true goal
to the end of the path, but only when the straight run to it is clear.

That "only when" is the whole safety argument, and it is one function. The
append itself lives in the planner NODE, which is an executable and not
linkable, which is exactly why segment_free was lifted into a header: the
guard is the part that must not be wrong, so it is the part that gets pinned.

The convention under test that is easiest to get backwards: out of bounds
BLOCKS here. path_still_valid() skips OOB cells because it is keeping an
already-searched path alive; this function authorises a segment nothing ever
searched, so unknown must refuse. A regression that "fixes" the OOB case to
match path_still_valid would let the planner extend paths off the edge of the
map, which is precisely the failure the guard exists to prevent.
```

### goal-snap-float-resolution

**Fixture resolution follows float widening** — attached to `constexpr double kResExact = static_cast<double>(static_cast<float>(kRes));` (line 46)

```text
The resolution the code under test actually divides by.

OccupancyGrid::info::resolution is a FLOAT, and snapshot_from_occupancy_grid
widens it to double, so the nominal 0.20 arrives as 0.20000000298023224. Cell
indices computed from the literal 0.20 therefore land one cell LOW at every
exact multiple of the resolution: floor(3.0 / 0.20) is 15, floor(3.0 /
0.20000000298) is 14. The first six versions of the tests below all failed for
this reason and the failure looked like a bug in the traversal rather than in
the fixture. The fixture must do the same widening the code does, so the cell
it marks is the cell the test is about.
```

## SegmentFree.TinyCornerChordIsCaught

### goal-snap-tiny-corner-chord

**A tiny corner chord must be caught** — attached to `TEST(SegmentFree, TinyCornerChordIsCaught)` (line 176)

```text
The property that motivated replacing the sampler with an exact walk: a
segment that clips an occupied cell over a chord far SHORTER than any sane
sample step is still caught.

The geometry is written in cell coordinates and the chord length is asserted,
not assumed, because the whole point is that the chord is tiny -- a test that
silently built a long chord would pass against the sampler too and would not
be testing anything.
```

## TEST

### goal-snap-corner-chord-geometry

**Geometry of the corner-chord cut** — attached to `const double lo_x = cell_lo(kBx);` (line 192)

```text
Cut down-right across the cell's lower-left corner. The segment enters
through the left edge 1 mm above the corner and leaves through the bottom
edge 1 mm right of it, so the chord inside the occupied cell is
hypot(1mm, 1mm) = 1.4 mm -- 35x shorter than the quarter-cell (50 mm) step
the sampler used, and no sample step fixes that because halving the step
only halves the chord it misses.

The sign convention matters and is easy to get backwards: a down-right
segment through this corner clips EITHER (kBx,kBy) or the diagonally
opposite (kBx-1,kBy-1) depending on which boundary it crosses first, so the
entry/exit geometry is asserted below rather than eyeballed.
```

## File scope (part 2)

### goal-snap-corner-crossing-defects

**Exact and near-tie corner crossings** — attached to `namespace` (line 245)

```text
Two separate defects, and they need two separate guards, so the negative
control for one does not cover the other.

(a) THE EXACT TIE. A 45-degree segment between two cell centres leaves its
    cell across both boundaries at the same t, at EVERY corner it crosses. The
    traversal resolved that with a bare `else` that stepped y, entering one
    shoulder cell and not the other. Guarded by ACornerCrossingIsSymmetric,
    whose real content is that BOTH shoulders now block.

(b) THE NEAR-TIE. An exact geometric corner is not always an exact
    floating-point tie: on the segment in TheNearTieCornerIsSymmetric the
    computed gap is 2.8e-17 and 4.4e-16 rather than zero, with opposite signs
    depending on which endpoint the parameters were computed from. No tie
    branch can catch that. It is fixed by ordering the endpoints before the
    walk, and guarded by SegmentFreeIsSymmetricOverASweep -- which is where it
    was found, by sweeping angles rather than by reasoning about the tie.

Because the endpoints are now ordered, every forward-vs-reverse assertion here
is true BY CONSTRUCTION. That is the point: these tests exist to fail if the
ordering is ever removed, and removing it does fail the sweep.
```

## SegmentFree.TheCornerFixtureReallyTies

### goal-snap-tie-calibration

**The corner fixture must really tie** — attached to `TEST(SegmentFree, TheCornerFixtureReallyTies)` (line 278)

```text
CALIBRATION, and it is not optional: every assertion below is about the tie
branch, so if this geometry does not actually tie, those tests are silently
exercising the ordinary one-axis step and proving nothing about the fix.
This recomputes the two crossing parameters with the same expressions the
function uses and demands they be BITWISE equal.
```

## SegmentFree.ACornerCrossingIsSymmetric

### goal-snap-shoulder-symmetry

**Corner shoulders block both ways** — attached to `TEST(SegmentFree, ACornerCrossingIsSymmetric)` (line 294)

```text
THE DEFECT, pinned. One occupied shoulder, and the answer must not depend on
which end you start from. Before the fix, blocking (10,11) gave blocked
forwards and free backwards, and blocking (11,10) gave the mirror image.

The EXPECT_FALSE is what carries this test, not the EXPECT_EQ: with the
endpoints ordered the two calls are the same computation, so equality is
structural. Both shoulders blocking is not -- if the tie branch had not run,
the walk would traverse exactly one of them and the other would read free.
Deleting the tie branch fails this test and only this test.
```

## SegmentFree.ADiagonalSqueezeBetweenTwoCornersIsRefused

### goal-snap-diagonal-squeeze

**Diagonal squeeze between corners is refused** — attached to `TEST(SegmentFree, ADiagonalSqueezeBetweenTwoCornersIsRefused)` (line 332)

```text
The conservative direction, stated as its own claim rather than left implicit:
a segment threading the gap between two diagonally-opposed occupied cells is
refused. The permissive reading -- a zero-length corner touch is not a
crossing -- would allow it, and would be authorising the robot to drive
between two obstacles on an already-inflated grid.
```

## SegmentFree.TheNearTieCornerIsSymmetric

### goal-snap-near-tie-witness

**The near-tie witness segment** — attached to `TEST(SegmentFree, TheNearTieCornerIsSymmetric)` (line 349)

```text
The specific segment the sweep found, pinned by name so the regression has a
witness that does not depend on the sweep's obstacle field or its bounds.
(8,11) -> (26,25) passes exactly through the corners at (13,15) and (22,22),
and before the endpoints were ordered it traversed (12,15) and (21,22) going
one way and (13,14) and (22,21) coming back -- the opposite shoulder of each
corner. Occupying one of those cells made the answer depend on argument order.
```

## SegmentFree.TheWalkConvergesOnTieHeavyAndDegenerateGeometry

### goal-snap-walk-convergence

**Walk convergence under the step cap** — attached to `TEST(SegmentFree, TheWalkConvergesOnTieHeavyAndDegenerateGeometry)` (line 410)

```text
The diagonal step changes how fast the walk spends its step budget, and the
budget is capped: a walk that fails to reach the end cell returns FALSE. That
failure mode is invisible in the ordinary tests, because "blocked" is also the
right answer when something is genuinely in the way -- a non-converging walk
would quietly refuse every goal-snap on open ground and look conservative
rather than broken. So convergence is asserted directly, on open ground, over
the cases where the tie branch fires hardest or where it must NOT fire at all.
```

## SegmentFree.NonFiniteCoordinatesRefuse

### goal-snap-non-finite-guard

**Non-finite coordinates refuse by rule** — attached to `TEST(SegmentFree, NonFiniteCoordinatesRefuse)` (line 446)

```text
Non-finite coordinates refuse. These already refused before the guard was
added, but only because casting floor(NaN) to int is undefined behaviour that
happened to land out of bounds on this platform. Pinned so the answer is a
rule rather than a property of this compiler.
```

## SegmentFree.UnknownCellsReadAsFree

### goal-snap-unknown-reads-free

**Unknown cells read as free** — attached to `TEST(SegmentFree, UnknownCellsReadAsFree)` (line 537)

```text
Unknown (-1) reads as free. That is snapshot_from_occupancy_grid's existing
behaviour -- int8 -1 is not >= 50 -- and the goal-snap rule inherits it, so
it is stated rather than left to be discovered. This is the one place the
rule is NOT conservative: an unobserved cell inside the map is treated as
drivable.

WHAT BOUNDS IT, stated carefully because the obvious argument is wrong. It is
NOT that the robot has already seen the ground under its own wheels: the
segment does not start under the wheels. At simple_nav_planner_node.cpp:881
the near end is `out.path.poses.back()` -- the LAST waypoint of the path,
i.e. the far end of the rolling window, where observation is thinnest. For
role=local the path ends at the slice exit point, which is at the local map's
edge by construction. The two bounds that do hold are kGoalSnapMaxM (0.6 m at
:878, so at most three cells on the 0.2 m grid) and the fact that the whole
pipeline already drives through unobserved space -- A* planned the 20 m that
got here on the same collapsed array. See the OUT OF BOUNDS paragraph in
occupancy_grid_utils.hpp for why that convention is not local to this guard.
```
