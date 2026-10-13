// Moved comments: doc/simple_nav_3d_code_notes.md
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/grid_utils.hpp"
#include "simple_nav_3d/route_progress.hpp"

namespace simple_nav_3d
{

using namespace std::chrono_literals;

class SimpleNavNavigatorNode : public rclcpp::Node
{
public:
  SimpleNavNavigatorNode()
  : Node("simple_nav_navigator")
  {
    params_ = load_and_validate_params(*this);

    active_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(params_.active_goal_topic, 10);
    active_route_pub_ = create_publisher<nav_msgs::msg::Path>(params_.active_route_topic, 10);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      params_.goal_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // A goal_pose ends any held route before the same-goal test, so even a
        // goal equal to w_k ends it. (notes: nav-goal-pose-ends-route)
        bool route_was_active = false;
        const bool repeat = goal_pose_is_repeat(
          route_, has_active_goal_, active_goal_.pose, msg->pose, &route_was_active);
        if (route_was_active) {
          log_route_end(route_.header(), route_.index(), route_.size(), "goal_pose");
        }
        // Ignore a re-publish of the goal already being driven. Orientation is
        // part of the comparison, so a yaw-only revision passes through as a
        // new goal. (notes: nav-ignore-same-goal)
        if (repeat) {
          return;
        }
        // Tell a new goal from a re-arm of the one just reached. Both are
        // accepted and republished; a re-arm logs at DEBUG so an upstream
        // re-publishing an unchanged goal does not flood INFO.
        // (notes: nav-new-goal-vs-rearm)
        const bool is_rearm =
          had_goal_ever_ && same_goal_pose(active_goal_.pose, msg->pose);
        active_goal_ = *msg;
        has_active_goal_ = true;
        had_goal_ever_ = true;
        active_goal_pub_->publish(active_goal_);
        if (is_rearm) {
          RCLCPP_DEBUG(get_logger(),
            "Navigator re-armed the same goal (x=%.2f y=%.2f z=%.2f yaw=%.2f)",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            yaw_from_quaternion(msg->pose.orientation));
        } else {
          RCLCPP_INFO(get_logger(),
            "Navigator accepted new goal (x=%.2f y=%.2f z=%.2f yaw=%.2f)",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            yaw_from_quaternion(msg->pose.orientation));
        }
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    route_sub_ = create_subscription<nav_msgs::msg::Path>(
      params_.goal_route_topic, 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {on_route(*msg);});

    timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this]() { on_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "navigator node started: goal_in=%s active_goal_out=%s goal_route_in=%s "
      "active_route_out=%s pass_radius=%.2f",
      params_.goal_topic.c_str(),
      params_.active_goal_topic.c_str(),
      params_.goal_route_topic.c_str(),
      params_.active_route_topic.c_str(),
      params_.route_pass_radius_m);
  }

private:
  void on_tick()
  {
    if (!has_active_goal_) {
      return;
    }

    if (has_odom_) {
      // Pass waypoints first, so the arrival test below reads the new w_k.
      if (route_.active()) {
        advance_route();
      }
      const auto & p = latest_odom_.pose.pose.position;
      const auto & g = active_goal_.pose.position;
      const double dist = (params_.mode == "uav")
        ? std::sqrt((g.x - p.x) * (g.x - p.x) + (g.y - p.y) * (g.y - p.y) + (g.z - p.z) * (g.z - p.z))
        : std::hypot(g.x - p.x, g.y - p.y);
      if (dist < final_goal_tolerance(params_)) {
        has_active_goal_ = false;
        // Only the last waypoint can be within tolerance here: advance_route()
        // leaves any other w_k at least the pass radius away.
        if (route_.active()) {
          // One last echo: a pass of w_last-1 in this same tick has not been
          // published yet, and explo arrives only once w_last is its head.
          active_route_pub_->publish(route_.remaining());
          route_.finish();
          log_route_end(route_.header(), route_.index(), route_.size(), "completed");
        }
        // The first arrival on a goal logs at INFO, repeats at DEBUG.
        // active_goal_ is still intact here (only the flag was cleared), so the
        // comparison is against the goal just reached.
        // (notes: nav-repeat-arrival-log)
        const bool repeat = reached_goal_valid_ &&
          same_goal_pose(reached_goal_, active_goal_.pose);
        reached_goal_ = active_goal_.pose;
        reached_goal_valid_ = true;
        if (repeat) {
          RCLCPP_DEBUG(get_logger(),
            "Navigator goal reached again (dist=%.2f m)", dist);
        } else {
          RCLCPP_INFO(get_logger(), "Navigator goal reached (dist=%.2f m)", dist);
        }
        return;
      }
    }

    // Re-publish active goal periodically so downstream nodes can recover from transient drops.
    active_goal_pub_->publish(active_goal_);
    if (route_.active()) {
      active_route_pub_->publish(route_.remaining());
    }
  }

  /// goal_route intake (RP §6.1-6.2). A new id replaces any held route and is
  /// advanced and published at once; the last accepted id is a keep-alive.
  void on_route(const nav_msgs::msg::Path & msg)
  {
    const bool was_active = route_.active();
    const std_msgs::msg::Header old_header = route_.header();
    const std::size_t old_k = route_.index();
    const std::size_t old_n = route_.size();
    switch (route_.accept(msg)) {
      case RouteProgress::Intake::kSameId:
        RCLCPP_DEBUG(
          get_logger(), "Navigator ignored goal_route id=%d.%09u: last accepted id (keep-alive)",
          msg.header.stamp.sec, msg.header.stamp.nanosec);
        return;
      case RouteProgress::Intake::kInvalid:
        RCLCPP_WARN(
          get_logger(), "Navigator ignored goal_route id=%d.%09u: empty or non-finite",
          msg.header.stamp.sec, msg.header.stamp.nanosec);
        return;
      case RouteProgress::Intake::kAccepted:
        break;
    }
    if (was_active) {
      log_route_end(old_header, old_k, old_n, "replaced");
    }
    route_passes_ = 0;
    route_forced_replans_ = 0;
    active_goal_ = route_.current_goal();
    has_active_goal_ = true;
    had_goal_ever_ = true;
    RCLCPP_INFO(
      get_logger(), "Navigator accepted route id=%d.%09u wps=%zu first=(x=%.2f y=%.2f)",
      route_.header().stamp.sec, route_.header().stamp.nanosec, route_.size(),
      active_goal_.pose.position.x, active_goal_.pose.position.y);
    // Pass w0.. before the first publish, so a w0 already inside the radius does
    // not cost two goal changes within 50 ms. (notes: nav-route-advance-on-accept)
    if (has_odom_) {
      advance_route();
    }
    active_goal_pub_->publish(active_goal_);
    active_route_pub_->publish(route_.remaining());
  }

  /// Advance the held route past every w_k inside the pass radius and make the
  /// new w_k the active goal. Logs one leg line per call that passed any.
  void advance_route()
  {
    const geometry_msgs::msg::Point prev = active_goal_.pose.position;  // a copy: overwritten below
    const std::size_t passed = route_.advance(
      latest_odom_.pose.pose.position, params_.route_pass_radius_m, params_.mode == "uav");
    if (passed == 0) {
      return;
    }
    active_goal_ = route_.current_goal();
    route_passes_ += passed;
    // The planner's goal-change reset test, copied: such a jump forces a
    // replan on its next tick. (notes: nav-route-forced-replans)
    const double tol =
      params_.mode == "uav" ? params_.uav_goal_xyz_tol_m : params_.ugv_goal_xy_tol_m;
    if (std::hypot(active_goal_.pose.position.x - prev.x, active_goal_.pose.position.y - prev.y) >
      std::max(0.20, 2.0 * tol))
    {
      ++route_forced_replans_;
    }
    const auto & w = active_goal_.pose;
    RCLCPP_INFO(
      get_logger(),
      "Navigator route leg: id=%d.%09u wp=%zu/%zu passed=%zu (x=%.2f y=%.2f z=%.2f yaw=%.2f)",
      route_.header().stamp.sec, route_.header().stamp.nanosec, route_.index(), route_.size(),
      passed, w.position.x, w.position.y, w.position.z, yaw_from_quaternion(w.orientation));
  }

  /// One INFO line per route that ends: completed (last waypoint reached),
  /// goal_pose (preempted) or replaced (a new route id).
  void log_route_end(
    const std_msgs::msg::Header & header, std::size_t k, std::size_t n, const char * reason)
  {
    RCLCPP_INFO(
      get_logger(),
      "Navigator route end: id=%d.%09u reason=%s at_wp=%zu/%zu passes=%zu forced_replans=%zu",
      header.stamp.sec, header.stamp.nanosec, reason, k, n, route_passes_, route_forced_replans_);
  }

  NodeParameters params_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr active_goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_route_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;

  bool has_active_goal_{false};
  /// True once any goal was accepted, so active_goal_ holds a real pose. Never
  /// cleared on arrival, unlike has_active_goal_; stops a default active_goal_
  /// matching a real goal at the origin in the re-arm test.
  /// (notes: nav-had-goal-ever)
  bool had_goal_ever_{false};
  bool has_odom_{false};
  /// The goal this node last declared reached, for the repeat test on the
  /// arrival log line. Not used for any control decision.
  geometry_msgs::msg::Pose reached_goal_;
  bool reached_goal_valid_{false};
  geometry_msgs::msg::PoseStamped active_goal_;
  nav_msgs::msg::Odometry latest_odom_;
  /// The held goal_route; while active, active_goal_ is its w_k.
  RouteProgress route_;
  /// Per-route counters for the route-end line, reset on each accepted route.
  std::size_t route_passes_{0};
  std::size_t route_forced_replans_{0};
};

}  // namespace simple_nav_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_nav_3d::SimpleNavNavigatorNode>());
  rclcpp::shutdown();
  return 0;
}
