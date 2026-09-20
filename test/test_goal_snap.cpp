// Known-answer tests for segment_free, the authorisation test behind D2's
// goal-snap append.
//
// WHY THIS FILE EXISTS. Across the ts1b campaign the median arrival parked
// 0.269 m from the commanded goal while the exploration planner judged arrival
// against a 0.4 m tolerance -- and failGoal() blacklists the cell the robot is
// standing on, so a shortfall is not a retry, it is a poisoned position. The
// shortfall is quantisation: a 0.40 m global cell, a 0.20 m local cell, and a
// controller stop tolerance, composed. D2 closes it by appending the true goal
// to the end of the path, but only when the straight run to it is clear.
//
// That "only when" is the whole safety argument, and it is one function. The
// append itself lives in the planner NODE, which is an executable and not
// linkable, which is exactly why segment_free was lifted into a header: the
// guard is the part that must not be wrong, so it is the part that gets pinned.
//
// The convention under test that is easiest to get backwards: out of bounds
// BLOCKS here. path_still_valid() skips OOB cells because it is keeping an
// already-searched path alive; this function authorises a segment nothing ever
// searched, so unknown must refuse. A regression that "fixes" the OOB case to
// match path_still_valid would let the planner extend paths off the edge of the
// map, which is precisely the failure the guard exists to prevent.

#include <gtest/gtest.h>

#include <limits>
#include <utility>

#include <cmath>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "simple_nav_3d/mapping/occupancy_grid_utils.hpp"

using simple_nav_3d::MapSnapshot;
using simple_nav_3d::segment_free;
using simple_nav_3d::snapshot_from_occupancy_grid;

namespace
{

constexpr double kRes = 0.20;   // the local planning map's resolution
constexpr int kW = 50;          // 10 m
constexpr int kH = 50;

// The resolution the code under test actually divides by.
//
// OccupancyGrid::info::resolution is a FLOAT, and snapshot_from_occupancy_grid
// widens it to double, so the nominal 0.20 arrives as 0.20000000298023224. Cell
// indices computed from the literal 0.20 therefore land one cell LOW at every
// exact multiple of the resolution: floor(3.0 / 0.20) is 15, floor(3.0 /
// 0.20000000298) is 14. The first six versions of the tests below all failed for
// this reason and the failure looked like a bug in the traversal rather than in
// the fixture. The fixture must do the same widening the code does, so the cell
// it marks is the cell the test is about.
constexpr double kResExact = static_cast<double>(static_cast<float>(kRes));

int cell_of(double v)
{
  return static_cast<int>(std::floor(v / kResExact));
}

// Lower-left corner of a cell, in the same arithmetic the snapshot uses.
double cell_lo(int c)
{
  return c * kResExact;
}

// A 10x10 m grid at 0.20 m with origin at the origin, all free unless marked.
nav_msgs::msg::OccupancyGrid make_grid()
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.resolution = static_cast<float>(kRes);
  g.info.width = kW;
  g.info.height = kH;
  g.info.origin.position.x = 0.0;
  g.info.origin.position.y = 0.0;
  g.data.assign(static_cast<size_t>(kW * kH), 0);
  return g;
}

// Occupy the single cell containing (x, y). Written in CELL space after the
// conversion, so the fixture and the function under test agree on which cell
// that is rather than agreeing by arithmetic coincidence.
void occupy_cell(nav_msgs::msg::OccupancyGrid & g, int cx, int cy)
{
  ASSERT_GE(cx, 0);
  ASSERT_GE(cy, 0);
  ASSERT_LT(cx, kW);
  ASSERT_LT(cy, kH);
  g.data[static_cast<size_t>(cy * kW + cx)] = 100;
}

void occupy_at(nav_msgs::msg::OccupancyGrid & g, double x, double y)
{
  occupy_cell(g, cell_of(x), cell_of(y));
}

}  // namespace

// Fixture calibration. Everything below is a claim about a specific cell being
// free or occupied, so first prove the fixture can express both -- otherwise a
// test that passes because the grid is uniformly free proves nothing.
TEST(SegmentFree, FixtureCanMarkAndUnmarkACell)
{
  auto g = make_grid();
  auto snap = snapshot_from_occupancy_grid(g);
  ASSERT_TRUE(snap.valid);
  // Derived through the same widening the snapshot does, and cross-checked
  // against the snapshot's OWN resolution -- if those two ever disagree, every
  // claim below is about a different cell than the one it marked.
  EXPECT_DOUBLE_EQ(snap.resolution, kResExact);
  const int cx = cell_of(2.1);
  const int cy = cell_of(2.1);
  const size_t idx = static_cast<size_t>(cy * kW + cx);
  EXPECT_EQ(snap.occupied[idx], 0U);

  occupy_at(g, 2.1, 2.1);
  snap = snapshot_from_occupancy_grid(g);
  EXPECT_EQ(snap.occupied[idx], 1U);
}

TEST(SegmentFree, OpenGroundIsFree)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  EXPECT_TRUE(segment_free(snap, 1.0, 1.0, 1.5, 1.0));
  EXPECT_TRUE(segment_free(snap, 1.0, 1.0, 1.35, 1.35));
}

// The case the append is FOR: a 0.3 m gap across free ground.
TEST(SegmentFree, TypicalQuantisationGapIsFree)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  EXPECT_TRUE(segment_free(snap, 5.0, 5.0, 5.269, 5.0));
}

// The case the append must REFUSE: an obstacle sitting between the A* endpoint
// and the goal.
TEST(SegmentFree, ObstacleOnTheSegmentBlocks)
{
  auto g = make_grid();
  occupy_at(g, 5.3, 5.0);
  const auto snap = snapshot_from_occupancy_grid(g);
  EXPECT_FALSE(segment_free(snap, 5.1, 5.0, 5.5, 5.0));
}

// The obstacle is next to the segment, not on it. A guard that rejected this
// would refuse nearly every append in a forest and the fix would do nothing.
TEST(SegmentFree, ObstacleBesideTheSegmentDoesNotBlock)
{
  auto g = make_grid();
  occupy_at(g, 5.3, 5.6);   // three cells away in y
  const auto snap = snapshot_from_occupancy_grid(g);
  EXPECT_TRUE(segment_free(snap, 5.1, 5.0, 5.5, 5.0));
}

// Either endpoint being occupied is a refusal. The goal endpoint is the one
// that matters in practice -- it is the cell the explo planner asked for, and
// dscovox's inflation can mark it solid after the goal was chosen.
TEST(SegmentFree, OccupiedGoalEndpointBlocks)
{
  auto g = make_grid();
  occupy_at(g, 5.5, 5.0);
  const auto snap = snapshot_from_occupancy_grid(g);
  EXPECT_FALSE(segment_free(snap, 5.1, 5.0, 5.5, 5.0));
}

TEST(SegmentFree, OccupiedStartEndpointBlocks)
{
  auto g = make_grid();
  occupy_at(g, 5.1, 5.0);
  const auto snap = snapshot_from_occupancy_grid(g);
  EXPECT_FALSE(segment_free(snap, 5.1, 5.0, 5.5, 5.0));
}

// The property that motivated replacing the sampler with an exact walk: a
// segment that clips an occupied cell over a chord far SHORTER than any sane
// sample step is still caught.
//
// The geometry is written in cell coordinates and the chord length is asserted,
// not assumed, because the whole point is that the chord is tiny -- a test that
// silently built a long chord would pass against the sampler too and would not
// be testing anything.
TEST(SegmentFree, TinyCornerChordIsCaught)
{
  constexpr int kBx = 12;
  constexpr int kBy = 12;
  auto g = make_grid();
  occupy_cell(g, kBx, kBy);
  const auto snap = snapshot_from_occupancy_grid(g);

  // Cut down-right across the cell's lower-left corner. The segment enters
  // through the left edge 1 mm above the corner and leaves through the bottom
  // edge 1 mm right of it, so the chord inside the occupied cell is
  // hypot(1mm, 1mm) = 1.4 mm -- 35x shorter than the quarter-cell (50 mm) step
  // the sampler used, and no sample step fixes that because halving the step
  // only halves the chord it misses.
  //
  // The sign convention matters and is easy to get backwards: a down-right
  // segment through this corner clips EITHER (kBx,kBy) or the diagonally
  // opposite (kBx-1,kBy-1) depending on which boundary it crosses first, so the
  // entry/exit geometry is asserted below rather than eyeballed.
  const double lo_x = cell_lo(kBx);
  const double lo_y = cell_lo(kBy);
  const double a_x = lo_x - 0.010;
  const double a_y = lo_y + 0.011;
  const double b_x = lo_x + 0.011;
  const double b_y = lo_y - 0.010;

  // Neither endpoint is in the occupied cell, so a refusal cannot come from the
  // endpoint checks -- it has to come from the walk.
  ASSERT_EQ(cell_of(a_x), kBx - 1);
  ASSERT_EQ(cell_of(a_y), kBy);
  ASSERT_EQ(cell_of(b_x), kBx);
  ASSERT_EQ(cell_of(b_y), kBy - 1);

  // It crosses x = lo_x while still above lo_y, hence really does enter the
  // occupied cell rather than passing under it.
  const double dx = b_x - a_x;
  const double dy = b_y - a_y;
  const double t_at_left_edge = (lo_x - a_x) / dx;
  const double y_at_left_edge = a_y + t_at_left_edge * dy;
  ASSERT_GT(y_at_left_edge, lo_y);
  ASSERT_NEAR(std::hypot(0.001, 0.001), 0.00141, 1e-5);   // chord << 0.05 step

  EXPECT_FALSE(segment_free(snap, a_x, a_y, b_x, b_y));
}

// A long diagonal straight through an occupied cell -- the easy case, present so
// a regression that breaks the walk entirely is not masked by the hard case
// above happening to still refuse.
TEST(SegmentFree, DiagonalThroughAnOccupiedCellBlocks)
{
  auto g = make_grid();
  occupy_cell(g, 12, 12);
  const auto snap = snapshot_from_occupancy_grid(g);
  EXPECT_FALSE(
    segment_free(snap, cell_lo(11) + 0.1, cell_lo(11) + 0.1,
                 cell_lo(13) + 0.1, cell_lo(13) + 0.1));
}

// ---------------------------------------------------------------------------
// F19. Exact corner crossings.
//
// Two separate defects, and they need two separate guards, so the negative
// control for one does not cover the other.
//
// (a) THE EXACT TIE. A 45-degree segment between two cell centres leaves its
//     cell across both boundaries at the same t, at EVERY corner it crosses. The
//     traversal resolved that with a bare `else` that stepped y, entering one
//     shoulder cell and not the other. Guarded by ACornerCrossingIsSymmetric,
//     whose real content is that BOTH shoulders now block.
//
// (b) THE NEAR-TIE. An exact geometric corner is not always an exact
//     floating-point tie: on the segment in TheNearTieCornerIsSymmetric the
//     computed gap is 2.8e-17 and 4.4e-16 rather than zero, with opposite signs
//     depending on which endpoint the parameters were computed from. No tie
//     branch can catch that. It is fixed by ordering the endpoints before the
//     walk, and guarded by SegmentFreeIsSymmetricOverASweep -- which is where it
//     was found, by sweeping angles rather than by reasoning about the tie.
//
// Because the endpoints are now ordered, every forward-vs-reverse assertion here
// is true BY CONSTRUCTION. That is the point: these tests exist to fail if the
// ordering is ever removed, and removing it does fail the sweep.
// ---------------------------------------------------------------------------

namespace
{
// Centre of a cell, in the arithmetic the snapshot uses. Written as one
// multiply, matching how the boundary terms inside segment_free are formed, so
// the tie below is exact rather than exact-by-luck.
double cell_ctr(int c)
{
  return (c + 0.5) * kResExact;
}
}  // namespace

// CALIBRATION, and it is not optional: every assertion below is about the tie
// branch, so if this geometry does not actually tie, those tests are silently
// exercising the ordinary one-axis step and proving nothing about the fix.
// This recomputes the two crossing parameters with the same expressions the
// function uses and demands they be BITWISE equal.
TEST(SegmentFree, TheCornerFixtureReallyTies)
{
  const double x0 = cell_ctr(10), y0 = cell_ctr(10);
  const double x1 = cell_ctr(13), y1 = cell_ctr(13);
  const double t_next_x = ((10 + 1) * kResExact - x0) / (x1 - x0);
  const double t_next_y = ((10 + 1) * kResExact - y0) / (y1 - y0);
  ASSERT_EQ(t_next_x, t_next_y)
      << "the corner tests below are about the exact-tie branch; without a tie "
         "they exercise the ordinary step and assert nothing";
}

// THE DEFECT, pinned. One occupied shoulder, and the answer must not depend on
// which end you start from. Before the fix, blocking (10,11) gave blocked
// forwards and free backwards, and blocking (11,10) gave the mirror image.
//
// The EXPECT_FALSE is what carries this test, not the EXPECT_EQ: with the
// endpoints ordered the two calls are the same computation, so equality is
// structural. Both shoulders blocking is not -- if the tie branch had not run,
// the walk would traverse exactly one of them and the other would read free.
// Deleting the tie branch fails this test and only this test.
TEST(SegmentFree, ACornerCrossingIsSymmetric)
{
  const double x0 = cell_ctr(10), y0 = cell_ctr(10);
  const double x1 = cell_ctr(13), y1 = cell_ctr(13);

  for (const auto shoulder : {std::make_pair(10, 11), std::make_pair(11, 10)}) {
    auto g = make_grid();
    occupy_cell(g, shoulder.first, shoulder.second);
    const auto snap = snapshot_from_occupancy_grid(g);
    const bool fwd = segment_free(snap, x0, y0, x1, y1);
    const bool rev = segment_free(snap, x1, y1, x0, y0);
    EXPECT_EQ(fwd, rev) << "asymmetric with (" << shoulder.first << ","
                        << shoulder.second << ") occupied";
    EXPECT_FALSE(fwd) << "a corner shoulder is checked, not skipped";
  }
}

// The control for the test above: with nothing occupied the same segment is
// free both ways. Without this, a regression that refused everything would
// satisfy every EXPECT_FALSE above and look like a pass.
TEST(SegmentFree, TheCornerSegmentIsFreeOnOpenGround)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  EXPECT_TRUE(segment_free(snap, cell_ctr(10), cell_ctr(10),
                           cell_ctr(13), cell_ctr(13)));
  EXPECT_TRUE(segment_free(snap, cell_ctr(13), cell_ctr(13),
                           cell_ctr(10), cell_ctr(10)));
}

// The conservative direction, stated as its own claim rather than left implicit:
// a segment threading the gap between two diagonally-opposed occupied cells is
// refused. The permissive reading -- a zero-length corner touch is not a
// crossing -- would allow it, and would be authorising the robot to drive
// between two obstacles on an already-inflated grid.
TEST(SegmentFree, ADiagonalSqueezeBetweenTwoCornersIsRefused)
{
  auto g = make_grid();
  occupy_cell(g, 10, 11);
  occupy_cell(g, 11, 10);
  const auto snap = snapshot_from_occupancy_grid(g);
  EXPECT_FALSE(segment_free(snap, cell_ctr(10), cell_ctr(10),
                            cell_ctr(13), cell_ctr(13)));
  EXPECT_FALSE(segment_free(snap, cell_ctr(13), cell_ctr(13),
                            cell_ctr(10), cell_ctr(10)));
}

// The specific segment the sweep found, pinned by name so the regression has a
// witness that does not depend on the sweep's obstacle field or its bounds.
// (8,11) -> (26,25) passes exactly through the corners at (13,15) and (22,22),
// and before the endpoints were ordered it traversed (12,15) and (21,22) going
// one way and (13,14) and (22,21) coming back -- the opposite shoulder of each
// corner. Occupying one of those cells made the answer depend on argument order.
TEST(SegmentFree, TheNearTieCornerIsSymmetric)
{
  for (const auto shoulder : {std::make_pair(12, 15), std::make_pair(13, 14),
                              std::make_pair(21, 22), std::make_pair(22, 21)}) {
    auto g = make_grid();
    occupy_cell(g, shoulder.first, shoulder.second);
    const auto snap = snapshot_from_occupancy_grid(g);
    const double x0 = cell_ctr(8), y0 = cell_ctr(11);
    const double x1 = cell_ctr(26), y1 = cell_ctr(25);
    EXPECT_EQ(segment_free(snap, x0, y0, x1, y1),
              segment_free(snap, x1, y1, x0, y0))
        << "asymmetric with (" << shoulder.first << "," << shoulder.second
        << ") occupied";
  }
  // Control: the segment is free across this grid when nothing is occupied, so
  // the equalities above are not four copies of "blocked == blocked".
  const auto open = snapshot_from_occupancy_grid(make_grid());
  EXPECT_TRUE(segment_free(open, cell_ctr(8), cell_ctr(11),
                           cell_ctr(26), cell_ctr(25)));
}

// Symmetry as a property rather than as one worked example, over a spread of
// angles and a scattered obstacle field. A tie-break that is wrong at some other
// angle, or an off-by-one in the new shoulder indices, shows up here.
TEST(SegmentFree, SegmentFreeIsSymmetricOverASweep)
{
  auto g = make_grid();
  for (int k = 0; k < 40; ++k) {
    occupy_cell(g, 7 + (k * 13) % 30, 6 + (k * 7) % 30);
  }
  const auto snap = snapshot_from_occupancy_grid(g);

  int blocked_count = 0, free_count = 0;
  for (int ax = 8; ax <= 14; ++ax) {
    for (int ay = 8; ay <= 14; ++ay) {
      for (int bx = 20; bx <= 26; ++bx) {
        for (int by = 20; by <= 26; ++by) {
          const double x0 = cell_ctr(ax), y0 = cell_ctr(ay);
          const double x1 = cell_ctr(bx), y1 = cell_ctr(by);
          const bool fwd = segment_free(snap, x0, y0, x1, y1);
          const bool rev = segment_free(snap, x1, y1, x0, y0);
          ASSERT_EQ(fwd, rev)
              << "(" << ax << "," << ay << ") -> (" << bx << "," << by << ")";
          (fwd ? free_count : blocked_count)++;
        }
      }
    }
  }
  // The sweep has to contain both answers, or it is a symmetry proof about a
  // function that returned one constant.
  EXPECT_GT(blocked_count, 0) << "no segment was blocked; the obstacle field "
                                 "missed every line and the sweep proves nothing";
  EXPECT_GT(free_count, 0) << "every segment was blocked; same problem";
}

// The diagonal step changes how fast the walk spends its step budget, and the
// budget is capped: a walk that fails to reach the end cell returns FALSE. That
// failure mode is invisible in the ordinary tests, because "blocked" is also the
// right answer when something is genuinely in the way -- a non-converging walk
// would quietly refuse every goal-snap on open ground and look conservative
// rather than broken. So convergence is asserted directly, on open ground, over
// the cases where the tie branch fires hardest or where it must NOT fire at all.
TEST(SegmentFree, TheWalkConvergesOnTieHeavyAndDegenerateGeometry)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  const auto corner = [](int c) { return c * kResExact; };

  // Endpoints sitting exactly ON the point where four cells meet, which is where
  // cell_of()'s floor and the diagonal step have to agree on the end cell.
  EXPECT_TRUE(segment_free(snap, cell_ctr(10), cell_ctr(10), corner(14), corner(14)))
      << "segment ending exactly on a cell corner";
  EXPECT_TRUE(segment_free(snap, corner(14), corner(14), cell_ctr(20), cell_ctr(20)))
      << "segment starting exactly on a cell corner";
  EXPECT_TRUE(segment_free(snap, corner(10), corner(10), corner(14), corner(14)))
      << "both endpoints on cell corners";

  // A long 45-degree run: every single crossing is a tie, so the step budget is
  // spent two axes at a time for thirty cells.
  EXPECT_TRUE(segment_free(snap, cell_ctr(5), cell_ctr(5), cell_ctr(35), cell_ctr(35)))
      << "30-cell diagonal, tie at every corner";

  // Axis-aligned: ONE crossing parameter is infinity forever, and infinity is
  // not equal to a finite number, so the `<` wins and the tie branch is never
  // reached. Asserted anyway, because that argument is about the current
  // comparison order and a rewrite could get it wrong.
  EXPECT_TRUE(segment_free(snap, cell_ctr(5), cell_ctr(5), cell_ctr(35), cell_ctr(5)))
      << "horizontal";
  EXPECT_TRUE(segment_free(snap, cell_ctr(5), cell_ctr(5), cell_ctr(5), cell_ctr(35)))
      << "vertical";
}

// Non-finite coordinates refuse. These already refused before the guard was
// added, but only because casting floor(NaN) to int is undefined behaviour that
// happened to land out of bounds on this platform. Pinned so the answer is a
// rule rather than a property of this compiler.
TEST(SegmentFree, NonFiniteCoordinatesRefuse)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  const double inf_v = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(segment_free(snap, 2.0, 2.0, nan_v, 2.0));
  EXPECT_FALSE(segment_free(snap, 2.0, 2.0, 2.0, nan_v));
  EXPECT_FALSE(segment_free(snap, nan_v, nan_v, 2.0, 2.0));
  EXPECT_FALSE(segment_free(snap, 2.0, 2.0, inf_v, 2.0));
  EXPECT_FALSE(segment_free(snap, -inf_v, 2.0, 2.0, 2.0));
  // Control: the same call with finite coordinates is free, so the refusals
  // above are about the coordinates and not about this grid or this segment.
  EXPECT_TRUE(segment_free(snap, 2.0, 2.0, 2.4, 2.0));
}

// OOB refuses -- deliberately the opposite of path_still_valid(). See the file
// header: this is the convention a well-meaning "consistency" fix would break.
TEST(SegmentFree, OutOfBoundsBlocksUnlikePathStillValid)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  EXPECT_FALSE(segment_free(snap, 9.9, 5.0, 10.4, 5.0));   // runs off +x edge
  EXPECT_FALSE(segment_free(snap, 0.1, 5.0, -0.4, 5.0));   // runs off -x edge
  EXPECT_FALSE(segment_free(snap, 5.0, 5.0, 5.0, -0.2));   // runs off -y edge
}

// A start point already outside the map is refused too, not silently accepted
// by a loop that never finds an in-bounds sample to test.
TEST(SegmentFree, StartOutOfBoundsBlocks)
{
  const auto snap = snapshot_from_occupancy_grid(make_grid());
  EXPECT_FALSE(segment_free(snap, -1.0, 5.0, -0.5, 5.0));
}

// An invalid snapshot refuses. The caller reaches this with whatever snapshot
// the tick built, and "no map" must not read as "clear".
TEST(SegmentFree, InvalidSnapshotBlocks)
{
  MapSnapshot empty;   // valid == false
  EXPECT_FALSE(segment_free(empty, 0.0, 0.0, 0.1, 0.0));
}

// A snapshot whose occupancy vector is shorter than its declared dimensions
// refuses rather than indexing past the end. snapshot_from_occupancy_grid
// cannot produce this, but a hand-built MapSnapshot can, and the function is
// now public.
TEST(SegmentFree, TruncatedOccupancyVectorBlocks)
{
  MapSnapshot m;
  m.valid = true;
  m.resolution = kRes;
  m.width = kW;
  m.height = kH;
  m.origin_x = 0.0;
  m.origin_y = 0.0;
  m.occupied.assign(10, 0U);   // far short of kW*kH
  EXPECT_FALSE(segment_free(m, 1.0, 1.0, 1.2, 1.0));
}

// Degenerate segment: the caller guards against this with its dg > eps test,
// but the function must not divide by zero if that guard is ever relaxed.
TEST(SegmentFree, ZeroLengthSegmentTestsItsOwnCell)
{
  auto g = make_grid();
  const auto free_snap = snapshot_from_occupancy_grid(g);
  EXPECT_TRUE(segment_free(free_snap, 3.05, 3.05, 3.05, 3.05));

  occupy_at(g, 3.05, 3.05);
  const auto blocked_snap = snapshot_from_occupancy_grid(g);
  EXPECT_FALSE(segment_free(blocked_snap, 3.05, 3.05, 3.05, 3.05));
}

// The >= 50 threshold is snapshot_from_occupancy_grid's, and segment_free reads
// its output, so a cell at 49 is free and a cell at 50 is not. Pinned here
// because the goal-snap rule's conservatism argument assumes dscovox's 100 and
// would change meaning if the threshold moved.
TEST(SegmentFree, OccupancyThresholdIsFifty)
{
  auto g = make_grid();
  const int cx = cell_of(4.05);
  const int cy = cell_of(4.05);
  g.data[static_cast<size_t>(cy * kW + cx)] = 49;
  EXPECT_TRUE(segment_free(snapshot_from_occupancy_grid(g), 4.05, 4.05, 4.10, 4.05));

  g.data[static_cast<size_t>(cy * kW + cx)] = 50;
  EXPECT_FALSE(segment_free(snapshot_from_occupancy_grid(g), 4.05, 4.05, 4.10, 4.05));
}

// Unknown (-1) reads as free. That is snapshot_from_occupancy_grid's existing
// behaviour -- int8 -1 is not >= 50 -- and the goal-snap rule inherits it, so
// it is stated rather than left to be discovered. This is the one place the
// rule is NOT conservative: an unobserved cell inside the map is treated as
// drivable.
//
// WHAT BOUNDS IT, stated carefully because the obvious argument is wrong. It is
// NOT that the robot has already seen the ground under its own wheels: the
// segment does not start under the wheels. At simple_nav_planner_node.cpp:881
// the near end is `out.path.poses.back()` -- the LAST waypoint of the path,
// i.e. the far end of the rolling window, where observation is thinnest. For
// role=local the path ends at the slice exit point, which is at the local map's
// edge by construction. The two bounds that do hold are kGoalSnapMaxM (0.6 m at
// :878, so at most three cells on the 0.2 m grid) and the fact that the whole
// pipeline already drives through unobserved space -- A* planned the 20 m that
// got here on the same collapsed array. See the OUT OF BOUNDS paragraph in
// occupancy_grid_utils.hpp for why that convention is not local to this guard.
TEST(SegmentFree, UnknownCellsReadAsFree)
{
  auto g = make_grid();
  const int cx = cell_of(4.05);
  const int cy = cell_of(4.05);
  g.data[static_cast<size_t>(cy * kW + cx)] = -1;
  EXPECT_TRUE(segment_free(snapshot_from_occupancy_grid(g), 4.05, 4.05, 4.10, 4.05));
}
