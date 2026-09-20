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

  /// Called on every tick where the global path is empty, i.e. the goal was
  /// reached, cleared, or withdrawn.
  ///
  /// It exists because the node does not call compute_command() at all on an
  /// empty path, so a controller cannot observe that transition from inside
  /// compute_command() — a check there is unreachable code. Anything a
  /// controller latched *for the goal that just went away* has to be dropped
  /// here or it silently carries into the next goal.
  ///
  /// Called repeatedly while the path stays empty, so implementations must be
  /// idempotent and must not log unconditionally.
  virtual void on_path_cleared() {}

  /// True while the controller is executing a self-directed manoeuvre that owns
  /// the wheels and must not be interrupted part-way through (the UGV's
  /// back-up-and-turn recovery is the only one today).
  ///
  /// D2. It exists because the node gained a branch -- rotate-to-goal-yaw --
  /// that can fire while the path is still non-empty, and that branch returns
  /// before compute_command(). Without this query the node would silently
  /// freeze a half-finished recovery: the manoeuvre's state is tick-counted, so
  /// it would neither advance nor time out, and it would resume mid-stride
  /// however many seconds later the rotation ended. Retiring the recovery
  /// instead is not an option either -- on_path_cleared() is the only exit and
  /// it logs `PATH CLEARED`, which would be false here and would corrupt the
  /// entry/exit pairing that line exists to support.
  ///
  /// Default false: a controller with no interruptible state is always
  /// preemptible, which is the safe answer for the node's purposes.
  virtual bool has_pending_maneuver() const {return false;}
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLERS__CONTROLLER_BASE_HPP_
