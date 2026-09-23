// Known-answer tests for segment_free, the authorisation test behind D2's
// goal-snap append.
//
// segment_free authorises the goal-snap append. Out of bounds BLOCKS here,
// unlike path_still_valid(); do not change it to match, or the planner can
// extend paths off the map edge. (notes: goal-snap-segment-free-oob-blocks)
// Moved comments: doc/test_goal_snap_notes.md

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

// The resolution the code under test divides by: the grid resolution is a float
// widened to double, so indices from the literal 0.20 land one cell low at
// exact multiples. The fixture must widen the same way.
// (notes: goal-snap-float-resolution)
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

// A segment clipping an occupied cell over a chord far shorter than any sample
// step must still be refused. The geometry is written in cell coordinates so
// the chord really is tiny. (notes: goal-snap-tiny-corner-chord)
TEST(SegmentFree, TinyCornerChordIsCaught)
{
  constexpr int kBx = 12;
  constexpr int kBy = 12;
  auto g = make_grid();
  occupy_cell(g, kBx, kBy);
  const auto snap = snapshot_from_occupancy_grid(g);

  // Cuts down-right across the cell's lower-left corner, entering 1 mm above it
  // and leaving 1 mm right of it. Which cell it clips depends on the boundary
  // crossed first, so the geometry is asserted below.
  // (notes: goal-snap-corner-chord-geometry)
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
// Guards two corner defects: the exact tie (both shoulders must block) and the
// near-tie (fixed by ordering endpoints before the walk). Forward/reverse
// equality holds by construction; removing the ordering fails the sweep.
// (notes: goal-snap-corner-crossing-defects)
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

// Calibration: the corner tests below exercise the tie branch only if this
// geometry ties, so the two crossing parameters are recomputed as segment_free
// does and must be bitwise equal. (notes: goal-snap-tie-calibration)
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

// Each corner shoulder, occupied alone, must block in both directions. The
// EXPECT_FALSE carries the test: with ordered endpoints the EXPECT_EQ is
// structural, and deleting the tie branch fails the EXPECT_FALSE.
// (notes: goal-snap-shoulder-symmetry)
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

// Conservative by design: a segment threading between two diagonally opposed
// occupied cells is refused. Treating a zero-length corner touch as no crossing
// would let the robot pass between two obstacles.
// (notes: goal-snap-diagonal-squeeze)
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

// Near-tie witness from the sweep, pinned independently of its obstacle field:
// (8,11) -> (26,25) passes exactly through the corners at (13,15) and (22,22);
// each shoulder cell is occupied in turn. (notes: goal-snap-near-tie-witness)
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

// The walk's step budget is capped and a non-converging walk returns false,
// which looks like a conservative refusal. So convergence is asserted directly
// on open ground, for tie-heavy and tie-free geometry.
// (notes: goal-snap-walk-convergence)
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

// Non-finite coordinates refuse by explicit rule, not by the undefined
// behaviour of casting floor(NaN) to int. (notes: goal-snap-non-finite-guard)
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

// Unknown (-1) reads as free, inherited from the >= 50 threshold: the one
// non-conservative case. It is bounded by kGoalSnapMaxM (0.6 m), not by prior
// observation, since the segment starts at the path's last waypoint.
// (notes: goal-snap-unknown-reads-free)
TEST(SegmentFree, UnknownCellsReadAsFree)
{
  auto g = make_grid();
  const int cx = cell_of(4.05);
  const int cy = cell_of(4.05);
  g.data[static_cast<size_t>(cy * kW + cx)] = -1;
  EXPECT_TRUE(segment_free(snapshot_from_occupancy_grid(g), 4.05, 4.05, 4.10, 4.05));
}
