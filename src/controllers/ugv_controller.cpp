#include "simple_nav_3d/controllers/ugv_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "simple_nav_3d/grid_utils.hpp"

using simple_nav_3d::GridIndex;
using simple_nav_3d::flatten;
using simple_nav_3d::in_bounds;
using simple_nav_3d::normalize_angle;
using simple_nav_3d::yaw_from_quaternion;

namespace
{

struct FrontArcStats
{
  // Minimum clearance from the robot's bounding-box edge to the nearest obstacle.
  double min_clearance{std::numeric_limits<double>::infinity()};
};

FrontArcStats compute_front_arc_stats(
  const simple_nav_3d::MapSnapshot & map_snapshot,
  const nav_msgs::msg::Odometry & odom,
  double half_body_length,
  double half_body_width,
  double max_range,
  double half_angle_rad)
{
  FrontArcStats stats;
  if (!map_snapshot.valid || map_snapshot.width <= 0 || map_snapshot.height <= 0) {
    return stats;
  }

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (int gy = 0; gy < map_snapshot.height; ++gy) {
    for (int gx = 0; gx < map_snapshot.width; ++gx) {
      GridIndex idx{gx, gy};
      if (!in_bounds(idx, map_snapshot.width, map_snapshot.height)) {
        continue;
      }
      if (map_snapshot.occupied_raw[static_cast<size_t>(flatten(idx, map_snapshot.width))] == 0) {
        continue;
      }

      const double wx = map_snapshot.origin_x + (static_cast<double>(gx) + 0.5) * map_snapshot.resolution;
      const double wy = map_snapshot.origin_y + (static_cast<double>(gy) + 0.5) * map_snapshot.resolution;

      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      const double forward = cos_yaw * dx + sin_yaw * dy;
      const double lateral = -sin_yaw * dx + cos_yaw * dy;

      if (forward <= 0.0) {
        continue;
      }

      const double distance = std::hypot(forward, lateral);
      if (distance > max_range) {
        continue;
      }

      const double angle = std::atan2(lateral, forward);
      if (std::abs(angle) > half_angle_rad) {
        continue;
      }

      // Distance from obstacle to nearest point on robot's bounding box.
      const double forward_gap = std::max(0.0, forward - half_body_length);
      const double lateral_gap = std::max(0.0, std::abs(lateral) - half_body_width);
      const double clearance = std::hypot(forward_gap, lateral_gap);

      stats.min_clearance = std::min(stats.min_clearance, clearance);
    }
  }

  return stats;
}

// Min bounding-box clearance to occupied cells inside an angular arc
// centred on `center_angle_body` (body frame: 0 = forward, ±π = rear,
// +π/2 = left, -π/2 = right). Used by the recovery state machine to check
// rear clearance during BACKUP and to pick a turn direction when entering
// recovery. Generalisation of compute_front_arc_stats.
double compute_arc_clearance(
  const simple_nav_3d::MapSnapshot & map_snapshot,
  const nav_msgs::msg::Odometry & odom,
  double half_body_length,
  double half_body_width,
  double max_range,
  double center_angle_body,
  double half_angle_rad)
{
  double min_clearance = std::numeric_limits<double>::infinity();
  if (!map_snapshot.valid || map_snapshot.width <= 0 || map_snapshot.height <= 0) {
    return min_clearance;
  }

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (int gy = 0; gy < map_snapshot.height; ++gy) {
    for (int gx = 0; gx < map_snapshot.width; ++gx) {
      GridIndex idx{gx, gy};
      if (!in_bounds(idx, map_snapshot.width, map_snapshot.height)) continue;
      if (map_snapshot.occupied_raw[
            static_cast<size_t>(flatten(idx, map_snapshot.width))] == 0) {
        continue;
      }

      const double wx =
        map_snapshot.origin_x + (static_cast<double>(gx) + 0.5) * map_snapshot.resolution;
      const double wy =
        map_snapshot.origin_y + (static_cast<double>(gy) + 0.5) * map_snapshot.resolution;
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      const double forward = cos_yaw * dx + sin_yaw * dy;
      const double lateral = -sin_yaw * dx + cos_yaw * dy;

      const double range = std::hypot(forward, lateral);
      if (range > max_range) continue;

      const double angle = std::atan2(lateral, forward);
      const double angle_diff = normalize_angle(angle - center_angle_body);
      if (std::abs(angle_diff) > half_angle_rad) continue;

      const double forward_gap = std::max(0.0, std::abs(forward) - half_body_length);
      const double lateral_gap = std::max(0.0, std::abs(lateral) - half_body_width);
      const double clearance = std::hypot(forward_gap, lateral_gap);
      min_clearance = std::min(min_clearance, clearance);
    }
  }

  return min_clearance;
}

}  // namespace

namespace simple_nav_3d
{

UgvController::UgvController(const NodeParameters & params)
: params_(params)
{
}

std::string UgvController::name() const
{
  return "local_controller_ugv";
}

geometry_msgs::msg::Twist UgvController::compute_command(
  const nav_msgs::msg::Odometry & odom,
  const nav_msgs::msg::Path & global_path,
  const MapSnapshot & map_snapshot)
{
  geometry_msgs::msg::Twist cmd;

  if (global_path.poses.empty()) {
    return cmd;
  }

  const auto & current = odom.pose.pose.position;

  // Follow a near-future waypoint on the planned path instead of always chasing the final goal.
  size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < global_path.poses.size(); ++i) {
    const auto & p = global_path.poses[i].pose.position;
    const double d = std::hypot(p.x - current.x, p.y - current.y);
    if (d < nearest_dist) {
      nearest_dist = d;
      nearest_idx = i;
    }
  }
  constexpr double kLookaheadDistM = 0.8;
  size_t lookahead_idx = nearest_idx;
  double acc_dist = 0.0;
  for (size_t i = nearest_idx + 1; i < global_path.poses.size(); ++i) {
    const auto & a = global_path.poses[i - 1].pose.position;
    const auto & b = global_path.poses[i].pose.position;
    acc_dist += std::hypot(b.x - a.x, b.y - a.y);
    lookahead_idx = i;
    if (acc_dist >= kLookaheadDistM) {
      break;
    }
  }
  const auto & target = global_path.poses[lookahead_idx].pose.position;

  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  const double distance = std::hypot(dx, dy);

  // Do not reuse global goal tolerance for intermediate path tracking points.
  // A looser tolerance here can cause premature zero-cmd near local plan end.
  const double waypoint_stop_tol_m = std::max(0.03, 0.25 * params_.ugv_goal_xy_tol_m);
  if (distance < waypoint_stop_tol_m) {
    return cmd;
  }

  const double target_heading = std::atan2(dy, dx);
  const double current_heading = yaw_from_quaternion(odom.pose.pose.orientation);
  const double heading_error = normalize_angle(target_heading - current_heading);

  const double linear = params_.ugv_linear_kp * distance;
  const double angular = params_.ugv_heading_kp * heading_error;

  cmd.linear.x = std::clamp(linear, 0.0, params_.ugv_max_linear_vel_mps);
  cmd.angular.z = std::clamp(
    angular, -params_.ugv_max_angular_vel_rps, params_.ugv_max_angular_vel_rps);

  // When heading error is large, rotate in place instead of arcing forward.
  // Below the threshold, ramp linear speed down smoothly to zero.
  constexpr double kRotateInPlaceThreshRad = M_PI / 4.0;  // 45 deg
  if (std::abs(heading_error) > kRotateInPlaceThreshRad) {
    cmd.linear.x = 0.0;
  } else {
    const double turn_scale = 1.0 - std::abs(heading_error) / kRotateInPlaceThreshRad;
    cmd.linear.x *= turn_scale;
  }

  const double half_angle_rad = params_.ugv_avoidance_arc_half_angle_deg * M_PI / 180.0;
  const double half_body_length = 0.5 * params_.robot_body_length_m;
  const double half_body_width = 0.5 * params_.robot_body_width_m;
  const FrontArcStats stats = compute_front_arc_stats(
    map_snapshot,
    odom,
    half_body_length,
    half_body_width,
    params_.ugv_avoidance_max_range_m,
    half_angle_rad);

  if (!std::isfinite(stats.min_clearance)) {
    return cmd;
  }

  // stats.min_clearance is the distance from the robot's bounding-box edge to
  // the nearest raw obstacle cell. No blind-zone hack needed — the robot body
  // geometry is already accounted for.
  const double clearance = stats.min_clearance;

  if (clearance < params_.ugv_avoidance_slowdown_distance_m) {
    const double span =
      params_.ugv_avoidance_slowdown_distance_m - params_.ugv_avoidance_hard_stop_distance_m;
    double scale = 0.0;
    if (span > 1e-6) {
      scale = (clearance - params_.ugv_avoidance_hard_stop_distance_m) / span;
    }
    cmd.linear.x *= std::clamp(scale, 0.0, 1.0);
  }

  // If we're already in recovery, the state machine owns the command stream
  // until both phases complete (recovery_active_ is cleared inside
  // compute_recovery_command). Don't re-enter or short-circuit on clearance.
  if (recovery_active_) {
    return compute_recovery_command(odom, map_snapshot);
  }

  if (clearance < params_.ugv_avoidance_hard_stop_distance_m) {
    // Initialise the recovery state machine. Pick the turn direction here
    // (left vs right, by side clearance) so the whole sequence is committed
    // upfront and not re-decided every tick.
    const double cx = odom.pose.pose.position.x;
    const double cy = odom.pose.pose.position.y;
    const double cyaw = yaw_from_quaternion(odom.pose.pose.orientation);

    constexpr double kSideArcHalfAngle = M_PI / 4.0;  // 45 deg arc
    const double left_clr = compute_arc_clearance(
      map_snapshot, odom, half_body_length, half_body_width,
      params_.ugv_avoidance_max_range_m, M_PI / 2.0, kSideArcHalfAngle);
    const double right_clr = compute_arc_clearance(
      map_snapshot, odom, half_body_length, half_body_width,
      params_.ugv_avoidance_max_range_m, -M_PI / 2.0, kSideArcHalfAngle);
    const int sign = (left_clr >= right_clr) ? +1 : -1;

    recovery_active_ = true;
    recovery_phase_ = RecoveryPhase::BACKUP;
    recovery_start_x_ = cx;
    recovery_start_y_ = cy;
    recovery_target_yaw_ = normalize_angle(cyaw + sign * (M_PI / 2.0));

    fprintf(stderr,
      "[ugv_ctrl] HARD STOP -> recovery: clearance=%.2f, "
      "left_clr=%.2f right_clr=%.2f, will backup then turn %s 90deg\n",
      clearance, left_clr, right_clr, (sign > 0 ? "LEFT" : "RIGHT"));

    return compute_recovery_command(odom, map_snapshot);
  }

  return cmd;
}

// Two-phase deterministic recovery:
//   BACKUP: drive straight backwards until we've travelled kBackupDistance
//           OR rear clearance gets too tight (give up the backup early).
//   TURN:   rotate in place to the absolute target yaw chosen at recovery
//           entry (current yaw ± 90 deg).
// On TURN completion, clears recovery_active_ so normal control resumes
// the next tick. The phase progression terminates deterministically — no
// time-based cooldown, no oscillation hysteresis needed.
geometry_msgs::msg::Twist UgvController::compute_recovery_command(
  const nav_msgs::msg::Odometry & odom,
  const MapSnapshot & map_snapshot)
{
  geometry_msgs::msg::Twist cmd;

  constexpr double kBackupDistance = 0.8;     // m
  constexpr double kBackupSpeed = 0.15;       // m/s
  constexpr double kRearMinClearance = 0.20;  // m from body edge
  constexpr double kRearArcHalfAngle = M_PI / 3.0;  // 120 deg behind
  constexpr double kTurnTolerance = 0.10;     // rad (~5.7 deg)
  constexpr double kTurnSpeed = 0.6;          // rad/s

  const double cx = odom.pose.pose.position.x;
  const double cy = odom.pose.pose.position.y;
  const double cyaw = yaw_from_quaternion(odom.pose.pose.orientation);
  const double half_body_length = 0.5 * params_.robot_body_length_m;
  const double half_body_width = 0.5 * params_.robot_body_width_m;

  if (recovery_phase_ == RecoveryPhase::BACKUP) {
    const double dx = cx - recovery_start_x_;
    const double dy = cy - recovery_start_y_;
    const double traveled = std::hypot(dx, dy);

    const double rear_clr = compute_arc_clearance(
      map_snapshot, odom, half_body_length, half_body_width,
      params_.ugv_avoidance_max_range_m, M_PI, kRearArcHalfAngle);

    const bool rear_blocked = rear_clr < kRearMinClearance;
    const bool backed_far_enough = traveled >= kBackupDistance;

    if (!rear_blocked && !backed_far_enough) {
      cmd.linear.x = -kBackupSpeed;
      return cmd;
    }

    // Either travelled the budget or hit something behind — switch to turn.
    fprintf(stderr,
      "[ugv_ctrl] recovery BACKUP done: traveled=%.2f m rear_clr=%.2f "
      "(%s), entering TURN\n",
      traveled, rear_clr,
      rear_blocked ? "rear blocked" : "budget reached");
    recovery_phase_ = RecoveryPhase::TURN;
    // Fall through to compute the turn command this same tick.
  }

  if (recovery_phase_ == RecoveryPhase::TURN) {
    const double yaw_err = normalize_angle(recovery_target_yaw_ - cyaw);
    if (std::abs(yaw_err) < kTurnTolerance) {
      fprintf(stderr,
        "[ugv_ctrl] recovery TURN complete: yaw_err=%.2f rad, exiting recovery\n",
        yaw_err);
      recovery_phase_ = RecoveryPhase::DONE;
      recovery_active_ = false;
      return cmd;  // zero cmd this tick, normal control next tick
    }
    cmd.angular.z = std::clamp(
      params_.ugv_heading_kp * yaw_err, -kTurnSpeed, kTurnSpeed);
    return cmd;
  }

  // RecoveryPhase::DONE shouldn't be reachable here (recovery_active_ is
  // already false in that case), but return zero cmd for safety.
  return cmd;
}

}  // namespace simple_nav_3d
