// Known-answer tests for the global UGV planner.
//
// Guards that, given a map, the planner plans, and that the endpoint fast path
// returns the same answers as the full scan. Launch wiring of the map topic
// cannot be caught by a unit test. (notes: ugv-planner-test-purpose)
// Moved comments: doc/simple_nav_3d_code_notes.md

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "simple_nav_3d/mapping/occupancy_grid_utils.hpp"
#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/planners/ugv_planner.hpp"

using simple_nav_3d::MapSnapshot;
using simple_nav_3d::NodeParameters;
using simple_nav_3d::UgvPlanner;

namespace
{

constexpr double kRes = 0.25;
constexpr int kW = 80;   // 20 m
constexpr int kH = 80;

// A 20x20 m grid at 0.25 m, origin at (-10,-10), all free unless marked.
nav_msgs::msg::OccupancyGrid make_grid()
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.resolution = static_cast<float>(kRes);
  g.info.width = kW;
  g.info.height = kH;
  g.info.origin.position.x = -10.0;
  g.info.origin.position.y = -10.0;
  g.data.assign(static_cast<size_t>(kW * kH), 0);
  return g;
}

void set_cell(nav_msgs::msg::OccupancyGrid & g, double x, double y, int8_t v)
{
  const int cx = static_cast<int>(std::floor((x - g.info.origin.position.x) / kRes));
  const int cy = static_cast<int>(std::floor((y - g.info.origin.position.y) / kRes));
  if (cx < 0 || cy < 0 || cx >= kW || cy >= kH) {
    return;
  }
  g.data[static_cast<size_t>(cy * kW + cx)] = v;
}

// A wall along x = wall_x spanning y in [y_lo, y_hi].
void add_wall(nav_msgs::msg::OccupancyGrid & g, double wall_x, double y_lo, double y_hi)
{
  for (double y = y_lo; y <= y_hi; y += kRes * 0.5) {
    set_cell(g, wall_x, y, 100);
    set_cell(g, wall_x + kRes, y, 100);
  }
}

nav_msgs::msg::Odometry odom_at(double x, double y)
{
  nav_msgs::msg::Odometry o;
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.orientation.w = 1.0;
  return o;
}

geometry_msgs::msg::PoseStamped goal_at(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

// compute_plan reads exactly two of these: odom_frame (only to stamp the path
// header) and ugv_local_plan_resolution_m (waypoint spacing). Everything else
// is value-initialised, so setting more would only invite drift.
NodeParameters make_params()
{
  NodeParameters p{};
  p.odom_frame = "odom";
  p.ugv_local_plan_resolution_m = 0.2;
  return p;
}

// Reads occupancy from the snapshot the planner sees (after the >= 50
// threshold), not from the written OccupancyGrid. Out of bounds counts as
// occupied: the map edge is a wall. (notes: ugv-test-snapshot-occupancy)
bool cell_occupied(const MapSnapshot & snap, int cx, int cy)
{
  if (cx < 0 || cy < 0 || cx >= snap.width || cy >= snap.height) {
    return true;
  }
  return snap.occupied[static_cast<size_t>(cy * snap.width + cx)] != 0U;
}

int cell_x(const MapSnapshot & snap, double x)
{
  return static_cast<int>(std::floor((x - snap.origin_x) / snap.resolution));
}

int cell_y(const MapSnapshot & snap, double y)
{
  return static_cast<int>(std::floor((y - snap.origin_y) / snap.resolution));
}

// Fills every cell of a thickness-cell ring around the closed cell range
// [x0,x1] x [y0,y1]. Written in cell space: stepping the sides in metres misses
// the corner cells. (notes: ugv-test-seal-box-cell-space)
void seal_box(nav_msgs::msg::OccupancyGrid & g, int x0, int y0, int x1, int y1, int thickness)
{
  for (int cy = y0 - thickness; cy <= y1 + thickness; ++cy) {
    for (int cx = x0 - thickness; cx <= x1 + thickness; ++cx) {
      if (cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1) {
        continue;  // interior stays free
      }
      if (cx < 0 || cy < 0 || cx >= kW || cy >= kH) {
        continue;
      }
      g.data[static_cast<size_t>(cy * kW + cx)] = 100;
    }
  }
}

double path_length(const nav_msgs::msg::Path & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    len += std::hypot(dx, dy);
  }
  return len;
}

}  // namespace

// The baseline claim the whole design-A fix rests on: WITH a map, this planner
// plans. Every mr1 robot-log failed at exactly this point, for want of a map.
TEST(UgvGlobalPlanner, PlansAcrossOpenGround)
{
  const auto grid = make_grid();
  const MapSnapshot snap = simple_nav_3d::snapshot_from_occupancy_grid(grid);
  ASSERT_TRUE(snap.valid);

  UgvPlanner planner(make_params());
  const auto out = planner.compute_plan(odom_at(-6.0, 0.0), goal_at(6.0, 0.0), snap);

  ASSERT_TRUE(out.has_path);
  ASSERT_GE(out.path.poses.size(), 2u);
  // Straight-line distance is 12 m; A* on an open grid must not be wildly
  // longer. The generous bound is deliberate: this asserts "planned sanely",
  // not a specific tie-break among equal-cost 8-connected paths.
  EXPECT_GT(path_length(out.path), 11.0);
  EXPECT_LT(path_length(out.path), 16.0);
  const auto & last = out.path.poses.back().pose.position;
  EXPECT_NEAR(last.x, 6.0, 0.6);
  EXPECT_NEAR(last.y, 0.0, 0.6);
}

// An empty map is the starvation case, and it must fail CLOSED (no path), not
// produce a straight line through unknown space.
TEST(UgvGlobalPlanner, NoMapMeansNoPath)
{
  MapSnapshot empty;  // valid == false
  UgvPlanner planner(make_params());
  const auto out = planner.compute_plan(odom_at(0.0, 0.0), goal_at(5.0, 0.0), empty);
  EXPECT_FALSE(out.has_path);
}

// A wall with a gap: the path must exist and must route through the gap rather
// than through the wall. This is the property the exploration planner depends
// on and could not get while the global planner was starved — its absence is
// what left every goal to greedy local navigation.
TEST(UgvGlobalPlanner, RoutesAroundAWallThroughTheGap)
{
  auto grid = make_grid();
  // The wall must reach both map edges. A wall stopping short of the boundary
  // leaves a legal detour round the end, and A* takes it — which is correct
  // behaviour and would make this test assert nothing about the gap.
  add_wall(grid, 0.0, -10.0, -2.0);
  add_wall(grid, 0.0, 2.0, 10.0);  // gap: y in (-2, 2)

  const MapSnapshot snap = simple_nav_3d::snapshot_from_occupancy_grid(grid);
  UgvPlanner planner(make_params());
  const auto out = planner.compute_plan(odom_at(-6.0, -6.0), goal_at(6.0, -6.0), snap);

  ASSERT_TRUE(out.has_path);
  // Every waypoint must be off the wall, and at least one must be inside the
  // gap band — a path that crossed the wall would be shorter and is exactly
  // what a broken endpoint relaxation would produce.
  bool used_gap = false;
  for (const auto & ps : out.path.poses) {
    const double x = ps.pose.position.x;
    const double y = ps.pose.position.y;
    if (std::fabs(x) < 0.25) {
      EXPECT_LT(std::fabs(y), 2.5) << "path crossed the wall at y=" << y;
      used_gap = true;
    }
  }
  EXPECT_TRUE(used_gap);
  // Detouring to the gap and back is much longer than the 12 m straight line.
  EXPECT_GT(path_length(out.path), 14.0);
}

// Generation-5 endpoint fast path. When the requested goal cell is itself free
// and reachable it is returned directly instead of scanning the whole grid.
// The claim being guarded is EQUIVALENCE, not a new behaviour: the path must
// still end at the requested goal.
TEST(UgvGlobalPlanner, FreeGoalCellIsPlannedToExactly)
{
  const auto grid = make_grid();
  const MapSnapshot snap = simple_nav_3d::snapshot_from_occupancy_grid(grid);
  UgvPlanner planner(make_params());

  for (const auto & g : {std::pair<double, double>{3.0, 3.0},
      std::pair<double, double>{-4.5, 7.0},
      std::pair<double, double>{8.0, -8.0}})
  {
    const auto out = planner.compute_plan(odom_at(0.0, 0.0), goal_at(g.first, g.second), snap);
    ASSERT_TRUE(out.has_path) << "no path to (" << g.first << ", " << g.second << ")";
    const auto & last = out.path.poses.back().pose.position;
    EXPECT_NEAR(last.x, g.first, kRes * 2.0);
    EXPECT_NEAR(last.y, g.second, kRes * 2.0);
  }
}

// A goal inside an obstacle must not kill the plan: the endpoint relaxation
// picks the nearest free reachable cell instead. The exploration planner does
// aim at cells that turn out to be occupied — that is the normal cost of
// planning on a map that is still filling in.
TEST(UgvGlobalPlanner, BlockedGoalRelaxesToANearbyFreeCell)
{
  auto grid = make_grid();
  for (double dx = -0.5; dx <= 0.5; dx += kRes * 0.5) {
    for (double dy = -0.5; dy <= 0.5; dy += kRes * 0.5) {
      set_cell(grid, 5.0 + dx, 0.0 + dy, 100);
    }
  }
  const MapSnapshot snap = simple_nav_3d::snapshot_from_occupancy_grid(grid);
  UgvPlanner planner(make_params());
  const auto out = planner.compute_plan(odom_at(-5.0, 0.0), goal_at(5.0, 0.0), snap);

  ASSERT_TRUE(out.has_path);
  const auto & last = out.path.poses.back().pose.position;
  // Close to the blocked goal, but not inside the blob.
  EXPECT_LT(std::hypot(last.x - 5.0, last.y - 0.0), 2.5);
  EXPECT_GT(std::hypot(last.x - 5.0, last.y - 0.0), 0.4);
}

// A walled-off goal must not get a path behind the wall. The test needs a
// negative control (the seal is closed) and a positive control (the planner
// plans on this map), or a refusal proves nothing.
// (notes: ugv-test-enclosed-goal-controls)
TEST(UgvGlobalPlanner, FullyEnclosedGoalYieldsNoPath)
{
  auto grid = make_grid();
  const MapSnapshot probe = simple_nav_3d::snapshot_from_occupancy_grid(grid);
  ASSERT_TRUE(probe.valid);
  // A ~3 m interior around the goal at (6,0), walled two cells thick.
  const int x0 = cell_x(probe, 4.75), x1 = cell_x(probe, 7.25);
  const int y0 = cell_y(probe, -1.25), y1 = cell_y(probe, 1.25);
  constexpr int kThickness = 2;
  seal_box(grid, x0, y0, x1, y1, kThickness);

  const MapSnapshot snap = simple_nav_3d::snapshot_from_occupancy_grid(grid);
  ASSERT_TRUE(snap.valid);

  // NEGATIVE CONTROL: the box is closed. Every cell of the ring, not four
  // lines that are assumed to meet at the corners — the first version of this
  // check walked the sides only and the corners were in fact open.
  for (int cy = y0 - kThickness; cy <= y1 + kThickness; ++cy) {
    for (int cx = x0 - kThickness; cx <= x1 + kThickness; ++cx) {
      const bool interior = cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1;
      if (interior) {
        ASSERT_FALSE(cell_occupied(snap, cx, cy))
          << "interior cell (" << cx << ", " << cy << ") is not free";
      } else {
        ASSERT_TRUE(cell_occupied(snap, cx, cy))
          << "door in the seal at cell (" << cx << ", " << cy << ")";
      }
    }
  }
  // ...and the goal itself is inside it, on free ground.
  ASSERT_FALSE(cell_occupied(snap, cell_x(snap, 6.0), cell_y(snap, 0.0)));

  // POSITIVE CONTROL: on this same map, with the same start, a goal outside the
  // box still plans. Without this the assertion below is satisfied by any
  // planner that has stopped working.
  UgvPlanner planner(make_params());
  const auto reachable =
    planner.compute_plan(odom_at(-6.0, 0.0), goal_at(-2.0, 4.0), snap);
  ASSERT_TRUE(reachable.has_path) << "planner is inert on this map; the sealed-box "
    "result below would prove nothing";

  const auto out = planner.compute_plan(odom_at(-6.0, 0.0), goal_at(6.0, 0.0), snap);
  if (out.has_path) {
    // Relaxation is allowed, but no waypoint may enter the sealed interior.
    // Checked in cell space against the interior actually sealed; a metre-space
    // box of the outer ring flags legal paths.
    // (notes: ugv-test-relaxed-path-check)
    for (const auto & ps : out.path.poses) {
      const double x = ps.pose.position.x;
      const double y = ps.pose.position.y;
      const int cx = cell_x(snap, x), cy = cell_y(snap, y);
      const bool inside = cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1;
      EXPECT_FALSE(inside) << "planned into the sealed region at (" << x << ", " << y << ")";
    }
  } else {
    SUCCEED() << "refused outright, which is the preferred answer";
  }
}
