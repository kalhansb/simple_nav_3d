#ifndef SIMPLE_NAV_3D__CONTROLLERS__UGV_CONTROLLER_HPP_
#define SIMPLE_NAV_3D__CONTROLLERS__UGV_CONTROLLER_HPP_

#include <string>

#include "simple_nav_3d/controllers/controller_base.hpp"
#include "simple_nav_3d/parameters.hpp"

namespace simple_nav_3d
{

class UgvController : public ControllerBase
{
public:
  explicit UgvController(const NodeParameters & params);

  std::string name() const override;
  geometry_msgs::msg::Twist compute_command(
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::Path & global_path,
    const MapSnapshot & map_snapshot) override;

private:
  // Deterministic 2-phase recovery: drive straight backwards into known
  // free space, then rotate 90 deg to break out of the dead end. Replaces
  // the old sector-scoring recovery, which could oscillate near tree bases.
  enum class RecoveryPhase { BACKUP, TURN, DONE };

  geometry_msgs::msg::Twist compute_recovery_command(
    const nav_msgs::msg::Odometry & odom,
    const MapSnapshot & map_snapshot);

  geometry_msgs::msg::Twist enter_recovery(
    const nav_msgs::msg::Odometry & odom,
    const MapSnapshot & map_snapshot,
    const char * trigger,
    double desired_linear,
    double clearance);

  NodeParameters params_;
  bool recovery_active_{false};
  RecoveryPhase recovery_phase_{RecoveryPhase::DONE};
  // Pose snapshot at the moment recovery was entered. Backup distance is
  // measured against (recovery_start_x_, recovery_start_y_); the turn target
  // is an absolute yaw computed once at entry so accumulated wraparound
  // doesn't matter.
  double recovery_start_x_{0.0};
  double recovery_start_y_{0.0};
  double recovery_target_yaw_{0.0};

  // Ticks spent inside the current recovery sequence, counted only on ticks
  // where compute_recovery_command actually ran.
  //
  // WHY A TICK COUNT AND NOT A CLOCK. Both recovery phases exit on a physical
  // condition — BACKUP on distance travelled or a blocked rear, TURN on yaw
  // error — and neither can be reached by a robot that cannot move. A robot
  // wedged with a clear rear arc (rear_clr stays above kRearMinClearance
  // because the thing holding it is not in the map) commands reverse forever,
  // and nothing else in the stack can preempt it: this class already hoists
  // `recovery_active_` above the front-arc scan, which used to be the
  // accidental way out. So the sequence needs a bound of its own.
  //
  // A tick count is that bound and not a timestamp because the guard must not
  // depend on odom.header.stamp being populated — a zero stamp would make an
  // elapsed-time cap read 0 s forever and silently disable the very check
  // that exists to stop a silent hang. Ticks are also the honest unit here:
  // the node only calls into the controller when it has a fresh, non-empty
  // path, so a suspended recovery should not age.
  int recovery_ticks_{0};

};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLERS__UGV_CONTROLLER_HPP_
