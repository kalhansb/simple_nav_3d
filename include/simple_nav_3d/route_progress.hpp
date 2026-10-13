// Moved comments: doc/simple_nav_3d_code_notes.md
#ifndef SIMPLE_NAV_3D__ROUTE_PROGRESS_HPP_
#define SIMPLE_NAV_3D__ROUTE_PROGRESS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "simple_nav_3d/grid_utils.hpp"  // same_goal_pose
#include "std_msgs/msg/header.hpp"

namespace simple_nav_3d
{

/// Route id = goal_route header.stamp as integer ns. Not rclcpp::Time: a Time
/// built from a msg is RCL_SYSTEM_TIME and throws when compared with now().
inline int64_t route_id_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

/// Route-following state of the navigator (RP §6.2): the held route, its id
/// and the current waypoint index k. Header-only so it can be unit-tested; its
/// caller is the navigator node, an executable that cannot be linked into a test.
class RouteProgress
{
public:
  enum class Intake { kAccepted, kSameId, kInvalid };

  /// kInvalid: empty, or a non-finite x/y/z. No state change (no clear-on-empty).
  /// kSameId: id equals the LAST ACCEPTED id, whether that route is held,
  /// finished or preempted. No state change. (notes: route-id-last-accepted)
  /// kAccepted: replace the route, k = 0, active.
  Intake accept(const nav_msgs::msg::Path & route)
  {
    if (route.poses.empty()) {
      return Intake::kInvalid;
    }
    for (const auto & ps : route.poses) {
      const auto & q = ps.pose.position;
      if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
        return Intake::kInvalid;
      }
    }
    const int64_t id = route_id_ns(route.header.stamp);
    if (has_id_ && id == id_) {
      return Intake::kSameId;
    }
    route_ = route;
    id_ = id;
    has_id_ = true;
    k_ = 0;
    active_ = true;
    return Intake::kAccepted;
  }

  /// While active, k is not the last waypoint and |robot - w_k| < pass_radius,
  /// ++k. Returns how many waypoints were passed. Only w_k is tested (no
  /// skip-ahead) and the last waypoint is never passed.
  std::size_t advance(
    const geometry_msgs::msg::Point & robot, double pass_radius_m, bool use_3d)
  {
    std::size_t passed = 0;
    while (active_ && k_ + 1 < route_.poses.size() &&
      distance(robot, route_.poses[k_].pose.position, use_3d) < pass_radius_m)
    {
      ++k_;
      ++passed;
    }
    return passed;
  }

  /// goal_pose arrived: stop following. Keeps the id and content. Returns
  /// whether a route was active.
  bool preempt()
  {
    const bool was = active_;
    active_ = false;
    return was;
  }
  /// Last waypoint reached. Keeps the id.
  void finish() {active_ = false;}

  bool active() const {return active_;}
  bool at_last() const {return active_ && k_ + 1 == route_.poses.size();}
  std::size_t index() const {return k_;}
  std::size_t size() const {return route_.poses.size();}
  bool has_id() const {return has_id_;}
  int64_t id() const {return id_;}
  const std_msgs::msg::Header & header() const {return route_.header;}

  /// w_k with the route header (frame = route frame, stamp = route id). Only
  /// valid once a route was accepted.
  geometry_msgs::msg::PoseStamped current_goal() const
  {
    geometry_msgs::msg::PoseStamped g;
    g.header = route_.header;
    g.pose = route_.poses[k_].pose;
    return g;
  }

  /// w_k..w_last with the route header, so the id is echoed back to explo.
  nav_msgs::msg::Path remaining() const
  {
    nav_msgs::msg::Path out;
    out.header = route_.header;
    out.poses.assign(route_.poses.begin() + static_cast<std::ptrdiff_t>(k_), route_.poses.end());
    return out;
  }

  /// 2D for UGV, 3D for UAV: the navigator's arrival metric.
  static double distance(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b, bool use_3d)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return use_3d ? std::sqrt(dx * dx + dy * dy + dz * dz) : std::hypot(dx, dy);
  }

private:
  nav_msgs::msg::Path route_;
  std::size_t k_{0};
  bool active_{false};
  bool has_id_{false};
  int64_t id_{0};
};

/// goal_pose intake: end any held route FIRST, then run the same-goal test, so
/// even a goal equal to w_k ends the route (RP §6.1). Returns true when the
/// goal_pose should be ignored as a repeat; *route_was_active reports whether a
/// route was preempted. (notes: route-goal-pose-ends-route)
inline bool goal_pose_is_repeat(
  RouteProgress & route, bool has_active_goal,
  const geometry_msgs::msg::Pose & active, const geometry_msgs::msg::Pose & incoming,
  bool * route_was_active)
{
  const bool was = route.preempt();
  if (route_was_active) {
    *route_was_active = was;
  }
  return has_active_goal && same_goal_pose(active, incoming);
}

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__ROUTE_PROGRESS_HPP_
