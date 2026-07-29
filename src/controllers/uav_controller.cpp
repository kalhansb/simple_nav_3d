#include "simple_nav_3d/controllers/uav_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "simple_nav_3d/grid_utils.hpp"

namespace
{

// Scan the 2D local map for the minimum clearance in the direction
// the UAV intends to move. Returns the distance from the robot center
// to the nearest occupied cell along that movement vector.
double min_clearance_in_direction(
  const simple_nav_3d::MapSnapshot & map,
  double robot_x, double robot_y,
  double move_dx, double move_dy,
  double scan_range,
  double half_angle_rad)
{
  using simple_nav_3d::GridIndex;
  using simple_nav_3d::flatten;
  using simple_nav_3d::in_bounds;

  double min_dist = std::numeric_limits<double>::infinity();

  if (!map.valid || map.width <= 0 || map.height <= 0) {
    return min_dist;
  }

  const double move_heading = std::atan2(move_dy, move_dx);

  for (int gy = 0; gy < map.height; ++gy) {
    for (int gx = 0; gx < map.width; ++gx) {
      GridIndex idx{gx, gy};
      if (!in_bounds(idx, map.width, map.height)) {
        continue;
      }
      // Use raw obstacles if available, otherwise inflated.
      const auto & occ = map.occupied_raw.empty() ? map.occupied : map.occupied_raw;
      if (occ[static_cast<size_t>(flatten(idx, map.width))] == 0) {
        continue;
      }

      const double wx = map.origin_x + (static_cast<double>(gx) + 0.5) * map.resolution;
      const double wy = map.origin_y + (static_cast<double>(gy) + 0.5) * map.resolution;

      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      const double dist = std::hypot(dx, dy);

      if (dist > scan_range || dist < 1e-3) {
        continue;
      }

      // Check if this obstacle is within the cone of the movement direction.
      const double angle_to_obs = std::atan2(dy, dx);
      const double angle_diff = std::abs(simple_nav_3d::normalize_angle(angle_to_obs - move_heading));
      if (angle_diff <= half_angle_rad) {
        min_dist = std::min(min_dist, dist);
      }
    }
  }

  return min_dist;
}

}  // namespace

namespace simple_nav_3d
{

UavController::UavController(const NodeParameters & params)
: params_(params)
{
}

std::string UavController::name() const
{
  return "flight_controller_uav";
}

geometry_msgs::msg::Twist UavController::compute_command(
  const nav_msgs::msg::Odometry & odom,
  const nav_msgs::msg::Path & global_path,
  const MapSnapshot & map_snapshot)
{
  geometry_msgs::msg::Twist cmd;

  if (global_path.poses.empty()) {
    return cmd;
  }

  const auto & pos = odom.pose.pose.position;
  const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);

  // Find nearest waypoint on path, then look ahead.
  size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < global_path.poses.size(); ++i) {
    const auto & p = global_path.poses[i].pose.position;
    const double d = std::hypot(p.x - pos.x, p.y - pos.y);
    if (d < nearest_dist) {
      nearest_dist = d;
      nearest_idx = i;
    }
  }

  constexpr double kLookaheadM = 1.5;
  size_t target_idx = nearest_idx;
  double acc = 0.0;
  for (size_t i = nearest_idx + 1; i < global_path.poses.size(); ++i) {
    const auto & a = global_path.poses[i - 1].pose.position;
    const auto & b = global_path.poses[i].pose.position;
    acc += std::hypot(b.x - a.x, b.y - a.y);
    target_idx = i;
    if (acc >= kLookaheadM) {
      break;
    }
  }

  const auto & target = global_path.poses[target_idx].pose.position;

  // 3D position error in world frame.
  const double dx = target.x - pos.x;
  const double dy = target.y - pos.y;
  const double dz = target.z - pos.z;

  const double dist_xy = std::hypot(dx, dy);
  const double dist_3d = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (dist_3d < params_.uav_goal_xyz_tol_m * 0.25) {
    return cmd;
  }

  // Desired yaw: face toward the XY target (or hold current yaw if very close).
  double desired_yaw = yaw;
  if (dist_xy > 0.3) {
    desired_yaw = std::atan2(dy, dx);
  }
  const double yaw_error = normalize_angle(desired_yaw - yaw);

  // Proportional gains.
  constexpr double kPxy = 1.0;
  constexpr double kPz = 1.0;
  constexpr double kPyaw = 1.5;

  // World-frame velocity command.
  double vx_world = kPxy * dx;
  double vy_world = kPxy * dy;
  double vz = std::clamp(kPz * dz, -params_.uav_max_vz_mps, params_.uav_max_vz_mps);

  // --- Obstacle avoidance using local map ---
  constexpr double kHardStopDist = 0.5;   // meters — full stop
  constexpr double kSlowdownDist = 2.0;   // meters — start slowing
  constexpr double kScanRange = 3.0;      // meters — how far to look
  constexpr double kHalfAngle = M_PI / 3; // 60° half-cone in movement direction

  const double move_speed_xy = std::hypot(vx_world, vy_world);

  if (move_speed_xy > 0.01 && map_snapshot.valid) {
    // Normalize movement direction.
    const double move_dx = vx_world / move_speed_xy;
    const double move_dy = vy_world / move_speed_xy;

    const double clearance = min_clearance_in_direction(
      map_snapshot, pos.x, pos.y, move_dx, move_dy, kScanRange, kHalfAngle);

    if (std::isfinite(clearance)) {
      if (clearance < kHardStopDist) {
        // Hard stop — zero XY velocity.
        vx_world = 0.0;
        vy_world = 0.0;
      } else if (clearance < kSlowdownDist) {
        // Linear slowdown between hard stop and slowdown distance.
        const double scale = (clearance - kHardStopDist) / (kSlowdownDist - kHardStopDist);
        vx_world *= scale;
        vy_world *= scale;
      }
    }
  }

  // Transform world XY velocity into body frame for Gazebo cmd_vel.
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
  double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

  // Clamp body-frame velocities.
  vx_body = std::clamp(vx_body, -params_.uav_max_vx_mps, params_.uav_max_vx_mps);
  vy_body = std::clamp(vy_body, -params_.uav_max_vy_mps, params_.uav_max_vy_mps);

  const double yaw_rate = std::clamp(
    kPyaw * yaw_error, -params_.uav_max_yaw_rate_rps, params_.uav_max_yaw_rate_rps);

  cmd.linear.x = vx_body;
  cmd.linear.y = vy_body;
  cmd.linear.z = vz;
  cmd.angular.z = yaw_rate;

  return cmd;
}

}  // namespace simple_nav_3d
