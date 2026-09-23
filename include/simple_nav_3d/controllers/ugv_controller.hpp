// Moved comments: doc/simple_nav_3d_code_notes.md
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

  void on_path_cleared() override;
  // D2. The back-up-and-turn recovery is tick-counted, so a node that stops
  // calling compute_command() mid-recovery suspends it rather than ending it.
  bool has_pending_maneuver() const override {return recovery_active_;}

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

  // Ticks spent in the current recovery, counted only when
  // compute_recovery_command runs; bounds a recovery that cannot physically
  // complete. A tick count, not a clock, so a zero odom.header.stamp cannot
  // disable the bound. (notes: ugv-recovery-tick-bound)
  int recovery_ticks_{0};

};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLERS__UGV_CONTROLLER_HPP_
