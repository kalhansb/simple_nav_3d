#ifndef SIMPLE_NAV_3D__GRID_UTILS_HPP_
#define SIMPLE_NAV_3D__GRID_UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace simple_nav_3d
{

struct GridIndex
{
  int x{0};
  int y{0};
};

inline int flatten(const GridIndex & idx, int width)
{
  return idx.y * width + idx.x;
}

inline bool in_bounds(const GridIndex & idx, int width, int height)
{
  return idx.x >= 0 && idx.y >= 0 && idx.x < width && idx.y < height;
}

inline double normalize_angle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/// Is `b` the same navigation goal as `a`, to within the tolerances a
/// re-publish has to survive?
///
/// THE DEFECT THIS EXISTS TO FIX. The navigator's goal intake used to compare
/// `pose.position` only, with `==` on the three doubles, and ignore
/// `pose.orientation` entirely. An upstream goal revision that changed ONLY the
/// heading was therefore discarded at the intake and never reached the
/// controller — while the controller, two nodes downstream, documents the
/// opposite contract in its active-goal callback: "A change in ORIENTATION alone
/// is not [a new destination], because the tick below re-reads the target yaw
/// out of active_goal_ every time -- an in-place yaw revision is simply
/// tracked." That promise could not be kept, because the revision was filtered
/// out before it arrived.
///
/// It was LATENT, not observed: with an omnidirectional sensor model the
/// exploration planner drops its yaw arrival term outside EXPLOIT, its EXPLOIT
/// re-anchor publishes position and yaw together, and its homing arrival test is
/// distance-only — so no shipped path currently waits on a yaw-only update. It is
/// fixed anyway, because the next code that revises a heading in place would
/// fail silently and in a way that looks like a controller bug.
///
/// WHY TOLERANCES RATHER THAN `==`. The old exact comparison meant a goal
/// republished through any float round-trip (a different message, a transform,
/// a serialisation) read as a NEW goal, re-arming the navigator and restarting
/// its accept/reached cycle.
///
/// WHY 1e-5 m AND NOT 1e-6. The positional tolerance was 1e-6 m, justified as
/// "far above any round-trip noise". That is true near the origin and FALSE at
/// the coordinates this campaign actually publishes, because float32 spacing
/// scales with magnitude: a half-ULP at |x| = 12 m is 4.8e-7 m, but at |x| = 75 m
/// it is 3.8e-6 m and at |x| = 100 m it is still 3.8e-6 m — several times OVER
/// the old tolerance. The shipped ROI is x in [-51.3, 100.9], y in [-38.7, 74.5],
/// so the old guard covered roughly the innermost 13 m of a 150 m box and let a
/// pure round-trip re-arm the navigator everywhere else. It was not caught
/// because the round-trip test picked x = 12.345678, one of the coordinates where
/// the check cannot bite.
///
/// 1e-5 m holds for |coordinate| < 256 m (half-ULP there is 7.6e-6 m; the next
/// binade up, at 256 m, costs 1.5e-5 m and would break it). That is 2.5x the
/// largest shipped ROI bound. A world larger than +/-256 m needs this raised —
/// test/test_goal_identity.cpp pins that precondition so it fails loudly rather
/// than silently degrading into the bug above.
///
/// THE YAW TOLERANCE STAYS AT 1e-6 rad, and that is measured, not inherited: the
/// worst yaw error over a full -pi..pi sweep through a float32 quaternion is
/// 8.3e-8 rad, a 12x margin. Quaternion components are bounded by 1 regardless of
/// where the robot is, so the yaw term has no magnitude dependence to correct —
/// which is exactly why only the positional half was wrong.
///
/// Neither tolerance is a "close enough" judgement. 1e-5 m is 10 micrometres and
/// 1e-6 rad is 0.2 arc seconds; both are far below anything a planner means to
/// express. The controller owns proximity, with ugv.goal_xy_tol_m.
///
/// The yaw term compares through normalize_angle so that -pi and +pi are the
/// same heading, which they are. An `==` on the raw quaternion would call them
/// different and re-arm on a goal that did not move at all.
inline bool same_goal_pose(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b,
  double pos_eps_m = 1e-5, double yaw_eps_rad = 1e-6)
{
  if (std::abs(b.position.x - a.position.x) > pos_eps_m) return false;
  if (std::abs(b.position.y - a.position.y) > pos_eps_m) return false;
  if (std::abs(b.position.z - a.position.z) > pos_eps_m) return false;
  const double dyaw =
    normalize_angle(yaw_from_quaternion(b.orientation) -
                    yaw_from_quaternion(a.orientation));
  return std::abs(dyaw) <= yaw_eps_rad;
}

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__GRID_UTILS_HPP_
