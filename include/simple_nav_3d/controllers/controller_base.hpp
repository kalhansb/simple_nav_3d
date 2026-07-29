#ifndef SIMPLE_NAV_3D__CONTROLLERS__CONTROLLER_BASE_HPP_
#define SIMPLE_NAV_3D__CONTROLLERS__CONTROLLER_BASE_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "simple_nav_3d/mapping/map_snapshot.hpp"

namespace simple_nav_3d
{

class ControllerBase
{
public:
  virtual ~ControllerBase() = default;
  virtual std::string name() const = 0;
  virtual geometry_msgs::msg::Twist compute_command(
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::Path & global_path,
    const MapSnapshot & map_snapshot) = 0;
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLERS__CONTROLLER_BASE_HPP_
