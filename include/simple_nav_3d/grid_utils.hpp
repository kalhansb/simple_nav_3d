#ifndef SIMPLE_NAV_3D__GRID_UTILS_HPP_
#define SIMPLE_NAV_3D__GRID_UTILS_HPP_

#include <cmath>

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

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__GRID_UTILS_HPP_
