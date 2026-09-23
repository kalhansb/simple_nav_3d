// Moved comments: doc/simple_nav_3d_code_notes.md
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

/// Same navigation goal, position and yaw, to within float32 round-trip noise.
/// The pos_eps_m default 1e-5 m holds only for |coordinate| < 256 m (pinned by
/// test_goal_identity.cpp). Yaw compares via normalize_angle.
/// (notes: goal-identity-tolerances)
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
