// D2b. same_goal_pose(): the predicate the navigator's goal intake uses to
// decide whether an incoming goal is the one it is already driving.
//
// The tests pin both directions: a yaw-only revision is a new goal, and a goal
// changed only by float round-trip noise compares equal.
// (notes: test-goal-identity-background)
// Moved comments: doc/simple_nav_3d_code_notes.md

#include <gtest/gtest.h>

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "simple_nav_3d/grid_utils.hpp"

using simple_nav_3d::same_goal_pose;

namespace
{

/// A pose at (x, y, z) facing `yaw`. Built through the same quaternion the ROS
/// stack would carry, so the tests exercise the yaw extraction rather than a
/// shortcut around it.
geometry_msgs::msg::Pose pose_at(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = std::cos(yaw * 0.5);
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = std::sin(yaw * 0.5);
  return p;
}

}  // namespace

// The fixture itself has to be trustworthy before anything below means
// anything: if pose_at's quaternion did not round-trip through
// yaw_from_quaternion, every yaw assertion here would be about a heading other
// than the one it names.
TEST(SameGoalPose, TheFixtureRoundTripsYaw)
{
  for (const double yaw : {0.0, 0.3, 1.5707963, 2.9, -2.9, 3.0}) {
    EXPECT_NEAR(simple_nav_3d::yaw_from_quaternion(pose_at(1, 2, 3, yaw).orientation),
                yaw, 1e-12) << "yaw = " << yaw;
  }
}

TEST(SameGoalPose, IdenticalPosesAreTheSameGoal)
{
  const auto a = pose_at(3.0, -4.0, 0.5, 1.2);
  EXPECT_TRUE(same_goal_pose(a, a));
}

// THE FIX, pinned. Same position, different heading: a DIFFERENT goal. Before
// the fix this returned "same" and the revision was dropped at the intake.
TEST(SameGoalPose, AYawOnlyRevisionIsANewGoal)
{
  const auto a = pose_at(3.0, -4.0, 0.5, 0.0);
  const auto b = pose_at(3.0, -4.0, 0.5, 1.0);
  EXPECT_FALSE(same_goal_pose(a, b));
  EXPECT_FALSE(same_goal_pose(b, a)) << "the predicate must be symmetric";
}

// Even a small yaw revision counts, because the tolerance is a float-noise
// guard and not a "close enough heading" judgement — the controller owns that
// judgement, with ugv.goal_yaw_tol_rad, and it cannot apply it to a message it
// never receives.
TEST(SameGoalPose, ASmallYawRevisionStillCounts)
{
  const auto a = pose_at(1.0, 1.0, 0.0, 0.0);
  const auto b = pose_at(1.0, 1.0, 0.0, 0.01);   // 0.6 degrees
  EXPECT_FALSE(same_goal_pose(a, b));
}

// -pi and +pi are the same heading. An exact quaternion comparison would call
// them different and re-arm the navigator on a goal that did not move.
TEST(SameGoalPose, WrapAroundHeadingsAreEqual)
{
  const auto a = pose_at(2.0, 2.0, 0.0, M_PI);
  const auto b = pose_at(2.0, 2.0, 0.0, -M_PI);
  // Sanity: the two quaternions really are numerically different, so the
  // equality below is the predicate's work and not an accident of the fixture.
  // It is `z` that flips sign here, not `w` -- cos(+pi/2) and cos(-pi/2) are the
  // same ~6e-17, so asserting on `w` would assert nothing.
  ASSERT_NE(a.orientation.z, b.orientation.z);
  EXPECT_TRUE(same_goal_pose(a, b));
}

// Each position axis independently. z is included because the UAV path uses it
// and a copy-paste that compared x twice would otherwise pass.
TEST(SameGoalPose, EachPositionAxisIsCompared)
{
  const auto a = pose_at(1.0, 2.0, 3.0, 0.4);
  EXPECT_FALSE(same_goal_pose(a, pose_at(1.5, 2.0, 3.0, 0.4)));
  EXPECT_FALSE(same_goal_pose(a, pose_at(1.0, 2.5, 3.0, 0.4)));
  EXPECT_FALSE(same_goal_pose(a, pose_at(1.0, 2.0, 3.5, 0.4)));
}

namespace
{
/// One pass through 32-bit precision, which is what a message carrying float32
/// fields, or a transform composed in float, does to a pose.
geometry_msgs::msg::Pose round_tripped(geometry_msgs::msg::Pose p)
{
  p.position.x = static_cast<double>(static_cast<float>(p.position.x));
  p.position.y = static_cast<double>(static_cast<float>(p.position.y));
  p.position.z = static_cast<double>(static_cast<float>(p.position.z));
  p.orientation.z = static_cast<double>(static_cast<float>(p.orientation.z));
  p.orientation.w = static_cast<double>(static_cast<float>(p.orientation.w));
  return p;
}
}  // namespace

// A goal that survived a float round-trip is still the same goal. The fixture
// sits at large coordinates, where the round trip costs more than 1e-6 m; keep
// it there or the check cannot bite. (notes: test-round-trip-world-scale)
TEST(SameGoalPose, FloatRoundTripNoiseIsNotARevision)
{
  const auto a = pose_at(75.4, -88.37, 0.35, 0.7853981633974483);
  const auto b = round_tripped(a);
  // The round trip really did perturb EVERY axis — otherwise this test is
  // asserting that identical poses are identical, which is already covered, and
  // an axis that happens to be exactly representable (y = -88.25 is one, and was
  // the obvious choice here) would assert nothing at all.
  ASSERT_NE(a.position.x, b.position.x);
  ASSERT_NE(a.position.y, b.position.y);
  ASSERT_NE(a.position.z, b.position.z);
  ASSERT_NE(a.orientation.z, b.orientation.z);
  EXPECT_TRUE(same_goal_pose(a, b))
      << "float round-trip noise must not read as a goal revision";
}

// The negative control for the test above: at these coordinates the OLD 1e-6 m
// default does not survive the round trip. Without this, raising the default
// could be quietly reverted and the suite would stay green.
TEST(SameGoalPose, TheOldPositionalToleranceFailsAtWorldScale)
{
  const auto a = pose_at(75.4, -88.37, 0.35, 0.7853981633974483);
  const auto b = round_tripped(a);
  EXPECT_FALSE(same_goal_pose(a, b, /*pos_eps_m=*/1e-6))
      << "1e-6 m must be shown insufficient here, or the test above proves nothing";
  // And it is the position term that fails, not the yaw term: quaternion
  // components are bounded by 1 wherever the robot is, so yaw has no magnitude
  // dependence and 1e-6 rad is still correct.
  EXPECT_TRUE(same_goal_pose(a, b, /*pos_eps_m=*/1e-5, /*yaw_eps_rad=*/1e-6));
}

// The precondition the 1e-5 m default rests on, pinned so a larger world fails
// here rather than silently degrading back into the bug above. 1e-5 m covers
// |coordinate| < 256 m; past that binade a half-ULP is 1.5e-5 m and the guard
// stops guarding.
TEST(SameGoalPose, ThePositionalToleranceHoldsToTwoHundredFiftySixMetres)
{
  const auto near_bound = pose_at(255.9, 255.9, 0.35, 0.0);
  EXPECT_TRUE(same_goal_pose(near_bound, round_tripped(near_bound)))
      << "the default must hold across the whole binade it claims";
  // Past the bound it does not, and that is the documented limit rather than a
  // regression. If a world ever exceeds +/-256 m, raise pos_eps_m and move this.
  const auto past_bound = pose_at(300.7, 300.7, 0.35, 0.0);
  EXPECT_FALSE(same_goal_pose(past_bound, round_tripped(past_bound)))
      << "if this now passes, float32 got wider or the default was raised — "
         "update the bound in grid_utils.hpp's comment to match";
}

// The epsilons are parameters, and the default is not a licence to treat them
// as a proximity test. At the default a millimetre IS a new goal — which is
// correct: a planner that moves a goal by a millimetre meant to.
TEST(SameGoalPose, DefaultEpsilonIsFloatNoiseNotProximity)
{
  const auto a = pose_at(5.0, 5.0, 0.0, 0.0);
  const auto b = pose_at(5.001, 5.0, 0.0, 0.0);
  EXPECT_FALSE(same_goal_pose(a, b));
  // And the caller can widen it deliberately if it ever needs to.
  EXPECT_TRUE(same_goal_pose(a, b, /*pos_eps_m=*/0.01));
}

// The boundary is inclusive on yaw and exclusive-above on position, matching
// the implementation's `>` / `<=`. Pinned so a future rewrite has to choose the
// same side on purpose.
TEST(SameGoalPose, ToleranceBoundariesAreAsWritten)
{
  const auto a = pose_at(0.0, 0.0, 0.0, 0.0);
  // Exactly at the position epsilon: NOT greater than it, so still the same.
  EXPECT_TRUE(same_goal_pose(a, pose_at(1e-5, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(same_goal_pose(a, pose_at(2e-5, 0.0, 0.0, 0.0)));
  // Yaw: the quaternion construction is not exact at 1e-6, so this is asserted
  // against a comfortably-inside and a comfortably-outside value rather than on
  // the knife edge, where the assertion would be about std::cos and not about
  // the predicate.
  EXPECT_TRUE(same_goal_pose(a, pose_at(0.0, 0.0, 0.0, 1e-9)));
  EXPECT_FALSE(same_goal_pose(a, pose_at(0.0, 0.0, 0.0, 1e-4)));
}
