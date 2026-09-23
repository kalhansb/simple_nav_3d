// Moved comments: doc/simple_nav_3d_code_notes.md
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

  /// Called every tick the global path is empty (goal reached, cleared or
  /// withdrawn); compute_command() is not called then, so drop per-goal latched
  /// state here. Must be idempotent and must not log unconditionally.
  /// (notes: ctrl-on-path-cleared)
  virtual void on_path_cleared() {}

  /// True while a self-directed manoeuvre (the UGV recovery) owns the wheels
  /// and must not be interrupted; the node's rotate-to-goal-yaw branch checks
  /// it. Do not end it via on_path_cleared(): its PATH CLEARED log would be
  /// false. (notes: ctrl-has-pending-maneuver)
  virtual bool has_pending_maneuver() const {return false;}
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLERS__CONTROLLER_BASE_HPP_
