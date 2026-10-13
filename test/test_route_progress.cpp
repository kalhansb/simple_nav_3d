// RouteProgress and goal_pose_is_repeat(): the navigator's route following
// (RP §6.2, §10 item 1). They pin the pass rule (only w_k, never the last, chained
// in one tick), the route-id rules (keep-alive, no revival after finish or
// preempt, invalid routes change nothing) and the goal_pose ordering (the route
// ends before the same-goal test).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "simple_nav_3d/grid_utils.hpp"
#include "simple_nav_3d/route_progress.hpp"

using simple_nav_3d::RouteProgress;
using simple_nav_3d::goal_pose_is_repeat;
using simple_nav_3d::route_id_ns;
using simple_nav_3d::same_goal_pose;
using Intake = simple_nav_3d::RouteProgress::Intake;

namespace
{

constexpr double kPass = 2.0;   // route.pass_radius_m (D4)
constexpr bool k2d = false;     // UGV metric

using Xy = std::pair<double, double>;

/// The four-waypoint route on the 2.5 m lattice used throughout.
const std::vector<Xy> kRoute = {{5.0, 0.0}, {7.5, 0.0}, {7.5, 2.5}, {10.0, 2.5}};

/// A map-frame route whose header.stamp (the route id) is (id_sec, id_nsec).
/// Every waypoint faces yaw 0.
nav_msgs::msg::Path path(int32_t id_sec, uint32_t id_nsec, const std::vector<Xy> & xy)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.header.stamp.sec = id_sec;
  p.header.stamp.nanosec = id_nsec;
  for (const auto & w : xy) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = p.header;
    ps.pose.position.x = w.first;
    ps.pose.position.y = w.second;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  return p;
}

geometry_msgs::msg::Point at(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Pose pose_at(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = at(x, y, z);
  p.orientation.w = std::cos(yaw * 0.5);
  p.orientation.z = std::sin(yaw * 0.5);
  return p;
}

/// Holding kRoute (id 100 s) advanced to index k by robot positions that pass
/// one waypoint each.
RouteProgress held_at(std::size_t k)
{
  RouteProgress r;
  EXPECT_EQ(r.accept(path(100, 0, kRoute)), Intake::kAccepted);
  for (std::size_t i = 0; i < k; ++i) {
    const auto & w = kRoute[i];
    EXPECT_EQ(r.advance(at(w.first, w.second - 0.1), kPass, k2d), 1u) << "setup pass " << i;
  }
  EXPECT_EQ(r.index(), k);
  return r;
}

void expect_waypoint(const geometry_msgs::msg::PoseStamped & ps, const Xy & w)
{
  EXPECT_DOUBLE_EQ(ps.pose.position.x, w.first);
  EXPECT_DOUBLE_EQ(ps.pose.position.y, w.second);
}

}  // namespace

// The fixture's own geometry, which the pass tests below lean on: it would be
// a different test if w2 sat within 2 m of the case 4 robot.
TEST(RouteProgress, TheFixtureGeometryIsAsStated)
{
  EXPECT_DOUBLE_EQ(RouteProgress::distance(at(3.1, 0.0), at(5.0, 0.0), k2d), 1.9);
  EXPECT_GT(RouteProgress::distance(at(6.25, 0.0), at(7.5, 2.5), k2d), kPass);
  EXPECT_EQ(route_id_ns(path(100, 7, kRoute).header.stamp), 100000000007LL);
}

// 1.
TEST(RouteProgress, AcceptStartsAtZero)
{
  RouteProgress r;
  EXPECT_FALSE(r.has_id());
  EXPECT_FALSE(r.active());
  ASSERT_EQ(r.accept(path(100, 5, kRoute)), Intake::kAccepted);
  EXPECT_TRUE(r.active());
  EXPECT_TRUE(r.has_id());
  EXPECT_EQ(r.id(), 100000000005LL);
  EXPECT_EQ(r.index(), 0u);
  EXPECT_EQ(r.size(), 4u);
  EXPECT_FALSE(r.at_last());
  const auto rem = r.remaining();
  ASSERT_EQ(rem.poses.size(), 4u);
  EXPECT_EQ(rem.header.stamp.sec, 100);
  EXPECT_EQ(rem.header.stamp.nanosec, 5u);
  const auto g = r.current_goal();
  EXPECT_EQ(g.header.frame_id, "map");
  EXPECT_EQ(g.header.stamp.sec, 100);
  EXPECT_EQ(g.header.stamp.nanosec, 5u);
  expect_waypoint(g, kRoute[0]);
}

// "Last accepted" means an id was accepted, not that the default id 0 is held:
// a zero stamp is a new id to a fresh navigator, and its keep-alive after that.
TEST(RouteProgress, ZeroIdIsNewToAFreshNavigator)
{
  RouteProgress r;
  EXPECT_EQ(r.accept(path(0, 0, kRoute)), Intake::kAccepted);
  EXPECT_TRUE(r.active());
  EXPECT_EQ(r.accept(path(0, 0, kRoute)), Intake::kSameId);
}

// 2.
TEST(RouteProgress, PassInsideRadius)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  // 1.9 m from w0, 4.4 m from w1.
  EXPECT_EQ(r.advance(at(3.1, 0.0), kPass, k2d), 1u);
  EXPECT_EQ(r.index(), 1u);
  expect_waypoint(r.current_goal(), kRoute[1]);
}

TEST(RouteProgress, NoPassOutside)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  EXPECT_EQ(r.advance(at(2.9, 0.0), kPass, k2d), 0u);  // 2.1 m from w0
  EXPECT_EQ(r.index(), 0u);
}

// The radius is exclusive, as written in RP §4 ("< 2.0 m"). Pinned so a rewrite
// chooses the side on purpose. 2.0 is exact here: hypot(2, 0).
TEST(RouteProgress, PassRadiusIsExclusive)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  EXPECT_EQ(r.advance(at(3.0, 0.0), kPass, k2d), 0u);
  EXPECT_EQ(r.index(), 0u);
}

// 3. Only w_k can be passed (RP §6.2). A later waypoint the robot is near
// stays in the route; explo's residual check drops it if covered.
TEST(RouteProgress, NoSkipAhead)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  const auto robot = at(7.5, 3.0);   // 0.5 m from w2, 3.9 m from w0
  ASSERT_GT(RouteProgress::distance(robot, at(5.0, 0.0), k2d), kPass);
  EXPECT_EQ(r.advance(robot, kPass, k2d), 0u);
  EXPECT_EQ(r.index(), 0u);
}

// 4. Waypoints all inside the radius pass in one call, so the planner sees one
// goal change, not several.
TEST(RouteProgress, ChainedPassesOneTick)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  EXPECT_EQ(r.advance(at(6.25, 0.0), kPass, k2d), 2u);  // 1.25 m from w0 and w1
  EXPECT_EQ(r.index(), 2u);
}

// 5. The last waypoint is never passed: the node's 0.2 m arrival test owns it.
TEST(RouteProgress, LastNeverPassed)
{
  RouteProgress r = held_at(3);
  EXPECT_TRUE(r.at_last());
  EXPECT_EQ(r.advance(at(10.0, 2.6), kPass, k2d), 0u);  // 0.1 m from w3
  EXPECT_EQ(r.index(), 3u);
  EXPECT_TRUE(r.at_last());
  EXPECT_TRUE(r.active());
  // finish() ends it without moving k, and an ended route is not "at last".
  r.finish();
  EXPECT_FALSE(r.active());
  EXPECT_FALSE(r.at_last());
  EXPECT_EQ(r.index(), 3u);
}

// 6.
TEST(RouteProgress, SingleWaypointRoute)
{
  RouteProgress r;
  ASSERT_EQ(r.accept(path(100, 0, {{5.0, 0.0}})), Intake::kAccepted);
  EXPECT_TRUE(r.at_last());
  EXPECT_EQ(r.advance(at(5.0, 0.0), kPass, k2d), 0u);
  EXPECT_EQ(r.advance(at(4.0, 0.5), kPass, k2d), 0u);
  EXPECT_EQ(r.advance(at(50.0, 50.0), kPass, k2d), 0u);
  EXPECT_EQ(r.index(), 0u);
  EXPECT_TRUE(r.at_last());
}

// 7. The invariant the node relies on (spec_nav §4.2): after advance(), either
// w_k is the last waypoint or it is at least the pass radius away. With the
// radius validated above the 0.2 m tolerance, the arrival test can then only
// fire on the last waypoint.
TEST(RouteProgress, NonLastBeyondRadiusAfterAdvance)
{
  int n_last = 0;
  int n_beyond = 0;
  for (int i = 0; i <= 48; ++i) {
    for (int j = 0; j <= 32; ++j) {
      const auto robot = at(0.25 * i, -3.0 + 0.25 * j);
      RouteProgress r;
      r.accept(path(100, 0, kRoute));
      r.advance(robot, kPass, k2d);
      if (r.at_last()) {
        ++n_last;
        continue;
      }
      ++n_beyond;
      EXPECT_GE(RouteProgress::distance(robot, r.current_goal().pose.position, k2d), kPass)
        << "robot (" << robot.x << ", " << robot.y << ") index " << r.index();
    }
  }
  // Both branches ran, or the property was checked on nothing.
  EXPECT_GT(n_last, 0);
  EXPECT_GT(n_beyond, 0);
}

// The same invariant along a drive: a robot that steps toward w_k each tick,
// with fixed-seed lateral jitter, passes every waypoint exactly once and ends
// within the arrival tolerance of the last one.
TEST(RouteProgress, DrivenRobotPassesEachWaypointOnce)
{
  std::mt19937 rng(20261012u);
  std::uniform_real_distribution<double> jitter(-0.02, 0.02);
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  auto robot = at(0.0, -1.0);
  std::size_t passes = 0;
  int ticks = 0;
  for (; ticks < 2000; ++ticks) {
    passes += r.advance(robot, kPass, k2d);
    const auto g = r.current_goal().pose.position;
    const double d = RouteProgress::distance(robot, g, k2d);
    if (!r.at_last()) {
      ASSERT_GE(d, kPass) << "tick " << ticks;
    }
    if (r.at_last() && d < 0.2) {
      break;
    }
    const double step = std::min(0.05, d);
    robot.x += step * (g.x - robot.x) / d + jitter(rng);
    robot.y += step * (g.y - robot.y) / d + jitter(rng);
  }
  EXPECT_LT(ticks, 2000) << "never arrived";
  EXPECT_EQ(passes, 3u);
  EXPECT_EQ(r.index(), 3u);
}

// The UAV metric is 3D, as the node's arrival test is.
TEST(RouteProgress, UavPassUsesThreeDimensions)
{
  RouteProgress r2;
  r2.accept(path(100, 0, kRoute));
  EXPECT_EQ(r2.advance(at(5.0, 0.0, 2.5), kPass, /*use_3d=*/ false), 1u);
  RouteProgress r3;
  r3.accept(path(100, 0, kRoute));
  EXPECT_EQ(r3.advance(at(5.0, 0.0, 2.5), kPass, /*use_3d=*/ true), 0u);
  EXPECT_EQ(r3.advance(at(5.0, 0.0, 1.5), kPass, /*use_3d=*/ true), 1u);
}

// 8. A keep-alive (the last accepted id) changes nothing, even if its content
// differs: same id means same content is explo's rule, not something to repair.
TEST(RouteProgress, KeepAliveSameIdIgnored)
{
  RouteProgress r = held_at(2);
  EXPECT_EQ(r.accept(path(100, 0, {{50.0, 50.0}, {60.0, 60.0}})), Intake::kSameId);
  EXPECT_TRUE(r.active());
  EXPECT_EQ(r.index(), 2u);
  EXPECT_EQ(r.size(), 4u);
  expect_waypoint(r.current_goal(), kRoute[2]);
}

// 9.
TEST(RouteProgress, NewIdResets)
{
  RouteProgress r = held_at(2);
  const std::vector<Xy> b = {{20.0, 0.0}, {22.5, 0.0}};
  EXPECT_EQ(r.accept(path(101, 0, b)), Intake::kAccepted);
  EXPECT_TRUE(r.active());
  EXPECT_EQ(r.index(), 0u);
  EXPECT_EQ(r.size(), 2u);
  EXPECT_EQ(r.id(), 101000000000LL);
  expect_waypoint(r.current_goal(), b[0]);
  // B is now the last accepted id.
  EXPECT_EQ(r.accept(path(101, 0, kRoute)), Intake::kSameId);
  EXPECT_EQ(r.size(), 2u);
}

// 10. Two publishes in the same sim second differ only in nanoseconds.
TEST(RouteProgress, NanosecondsDistinguishIds)
{
  RouteProgress r;
  EXPECT_EQ(r.accept(path(100, 1, kRoute)), Intake::kAccepted);
  EXPECT_EQ(r.accept(path(100, 2, kRoute)), Intake::kAccepted);
  EXPECT_EQ(r.id(), 100000000002LL);
  EXPECT_NE(
    route_id_ns(path(100, 1, kRoute).header.stamp),
    route_id_ns(path(100, 2, kRoute).header.stamp));
  // The id is 64-bit: a campaign-length stamp does not wrap.
  EXPECT_EQ(route_id_ns(path(7200, 999999999, kRoute).header.stamp), 7200999999999LL);
}

// 11. A keep-alive that lags the last-waypoint arrival must not re-arm the route.
TEST(RouteProgress, SameIdAfterFinishIgnored)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  r.finish();
  EXPECT_FALSE(r.active());
  EXPECT_EQ(r.accept(path(100, 0, kRoute)), Intake::kSameId);
  EXPECT_FALSE(r.active());
  EXPECT_TRUE(r.has_id());
}

// 12. Nor may one that lags a brake goal revive the route.
TEST(RouteProgress, PreemptRetainsId)
{
  RouteProgress none;
  EXPECT_FALSE(none.preempt());
  EXPECT_FALSE(none.has_id());

  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  EXPECT_TRUE(r.preempt());
  EXPECT_FALSE(r.active());
  EXPECT_FALSE(r.preempt()) << "a second preempt has nothing to end";
  EXPECT_EQ(r.accept(path(100, 0, kRoute)), Intake::kSameId);
  EXPECT_FALSE(r.active());
  // Preempted, a route is not advanced either.
  EXPECT_EQ(r.advance(at(5.0, 0.0), kPass, k2d), 0u);
  EXPECT_EQ(r.index(), 0u);
  EXPECT_EQ(r.accept(path(101, 0, kRoute)), Intake::kAccepted);
  EXPECT_TRUE(r.active());
}

// 13. An empty or non-finite route is ignored whole: it neither clears the
// held route nor becomes the last accepted id (no clear-on-empty, RP §6.1).
TEST(RouteProgress, EmptyIgnored)
{
  RouteProgress r = held_at(1);
  EXPECT_EQ(r.accept(path(102, 0, {})), Intake::kInvalid);
  EXPECT_TRUE(r.active());
  EXPECT_EQ(r.index(), 1u);
  EXPECT_EQ(r.id(), 100000000000LL);
  // Id 102 was not recorded, so a real route with it is still new.
  EXPECT_EQ(r.accept(path(102, 0, kRoute)), Intake::kAccepted);

  RouteProgress first;
  EXPECT_EQ(first.accept(path(100, 0, {})), Intake::kInvalid);
  EXPECT_FALSE(first.has_id());
  EXPECT_FALSE(first.active());
}

TEST(RouteProgress, NonFiniteWaypointIgnored)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    for (const double bad : {nan, inf}) {
      RouteProgress r = held_at(1);
      auto p = path(103, 0, kRoute);
      auto & q = p.poses[2].pose.position;   // not the first waypoint
      (axis == 0 ? q.x : axis == 1 ? q.y : q.z) = bad;
      EXPECT_EQ(r.accept(p), Intake::kInvalid) << "axis " << axis << " value " << bad;
      EXPECT_EQ(r.index(), 1u);
      EXPECT_EQ(r.id(), 100000000000LL);
    }
  }
}

// 14. active_route echoes w_k..w_last under the route's own header.
TEST(RouteProgress, RemainingIsSuffix)
{
  RouteProgress r = held_at(2);
  const auto rem = r.remaining();
  ASSERT_EQ(rem.poses.size(), 2u);
  expect_waypoint(rem.poses[0], kRoute[2]);
  expect_waypoint(rem.poses[1], kRoute[3]);
  EXPECT_EQ(rem.header.frame_id, "map");
  EXPECT_EQ(rem.header.stamp.sec, 100);
  EXPECT_EQ(rem.header.stamp.nanosec, 0u);
  expect_waypoint(r.current_goal(), kRoute[2]);
}

// 15. A goal_pose equal to w_k is still a goal_pose: the route ends first, and
// only then does the same-goal test ignore it (the robot carries on to w_k as a
// single goal).
TEST(GoalPoseIsRepeat, GoalPoseEqualToWaypointEndsRoute)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  const auto w0 = r.current_goal().pose;
  bool was = false;
  EXPECT_TRUE(goal_pose_is_repeat(r, true, w0, w0, &was));
  EXPECT_TRUE(was);
  EXPECT_FALSE(r.active());
}

TEST(GoalPoseIsRepeat, AnyGoalPoseEndsRoute)
{
  RouteProgress r;
  r.accept(path(100, 0, kRoute));
  const auto w0 = r.current_goal().pose;
  bool was = false;
  EXPECT_FALSE(goal_pose_is_repeat(r, true, w0, pose_at(30.0, 1.0, 0.0, 0.0), &was));
  EXPECT_TRUE(was);
  EXPECT_FALSE(r.active());
  EXPECT_FALSE(goal_pose_is_repeat(r, true, w0, pose_at(30.0, 1.0, 0.0, 0.0), &was));
  EXPECT_FALSE(was) << "only the goal_pose that ended the route reports it";
}

// 16. With no route held, the intake is exactly today's same-goal test.
TEST(GoalPoseIsRepeat, GoalPoseWithoutRouteUnchanged)
{
  std::mt19937 rng(97u);
  std::uniform_real_distribution<double> coord(-100.0, 100.0);
  std::uniform_real_distribution<double> yaw(-M_PI, M_PI);
  std::uniform_int_distribution<int> kind(0, 3);
  RouteProgress none;
  RouteProgress ended;
  ended.accept(path(100, 0, kRoute));
  ended.finish();
  int n_same = 0;
  for (int i = 0; i < 2000; ++i) {
    const auto a = pose_at(coord(rng), coord(rng), 0.35, yaw(rng));
    auto b = a;
    switch (kind(rng)) {
      case 0: break;                                  // identical
      case 1: b.position.x += 1e-3; break;            // moved
      case 2: b = pose_at(
          a.position.x, a.position.y, 0.35,
          simple_nav_3d::yaw_from_quaternion(a.orientation) + 0.1); break;  // yaw only
      default: b = pose_at(coord(rng), coord(rng), 0.35, yaw(rng)); break;
    }
    for (const bool has : {false, true}) {
      for (RouteProgress * r : {&none, &ended}) {
        bool was = true;
        EXPECT_EQ(goal_pose_is_repeat(*r, has, a, b, &was), has && same_goal_pose(a, b));
        EXPECT_FALSE(was);
        EXPECT_EQ(goal_pose_is_repeat(*r, has, a, b, nullptr), has && same_goal_pose(a, b));
      }
    }
    n_same += same_goal_pose(a, b) ? 1 : 0;
  }
  // Both outcomes were exercised.
  EXPECT_GT(n_same, 0);
  EXPECT_LT(n_same, 2000);
  EXPECT_FALSE(ended.active());
  EXPECT_TRUE(ended.has_id());
}
