// Moved comments: doc/simple_nav_controller_node_notes.md
#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_nav_3d/controller_factory.hpp"
#include "simple_nav_3d/controllers/controller_base.hpp"
#include "simple_nav_3d/mapping/occupancy_grid_utils.hpp"
#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/grid_utils.hpp"

namespace simple_nav_3d
{

using namespace std::chrono_literals;

class SimpleNavControllerNode : public rclcpp::Node
{
public:
  SimpleNavControllerNode()
  : Node("simple_nav_controller")
  {
    params_ = load_and_validate_params(*this);
    controller_ = create_controller(params_.controller, params_);

    // Role-based path source: when pipeline.role==local, follow the local
    // planner's output; otherwise follow the global planner's output. This
    // mirrors the planner_node role logic — the controller code itself is
    // role-agnostic.
    const bool is_local_role = (params_.pipeline_role == "local");
    is_uav_ = (params_.mode == "uav");
    const std::string path_input_topic =
      is_local_role ? params_.local_path_topic : params_.global_path_topic;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(params_.cmd_vel_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        last_odom_time_ = now();
        has_odom_ = true;
      });

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      params_.local_map_topic, map_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = *msg;
        last_map_time_ = now();
        has_map_ = true;
      });

    raw_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      params_.local_map_raw_topic, map_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_raw_map_ = *msg;
        last_raw_map_time_ = now();
        has_raw_map_ = true;
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_input_topic, 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
        last_path_time_ = now();
        has_path_ = true;
      });

    // The desired yaw comes from the active goal, not the path's last waypoint:
    // every pose here carries a meaningful orientation, a new goal replaces the
    // old one, and the yaw is available as soon as the goal is.
    // (notes: ctrl-yaw-from-active-goal)
    active_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      params_.active_goal_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // A move in position is a new destination: any rotation in progress for
        // the previous one is abandoned. A change in ORIENTATION alone is not,
        // because the tick below re-reads the target yaw out of active_goal_
        // every time -- an in-place yaw revision is simply tracked.
        if (has_active_goal_) {
          const auto & a = active_goal_.pose.position;
          const auto & b = msg->pose.position;
          if (std::hypot(b.x - a.x, b.y - a.y) > 1e-6 ||
              std::abs(b.z - a.z) > 1e-6)
          {
            end_rotate("goal moved");
          }
        }
        active_goal_ = *msg;
        has_active_goal_ = true;
      });

    timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this]() { on_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "controller node started: role=%s path_in=%s goal_in=%s local_map=%s "
      "cmd_vel=%s controller=%s yaw_tol=%.2frad w_max=%.2frad/s",
      params_.pipeline_role.c_str(),
      path_input_topic.c_str(),
      params_.active_goal_topic.c_str(),
      params_.local_map_topic.c_str(),
      params_.cmd_vel_topic.c_str(),
      controller_->name().c_str(),
      params_.ugv_goal_yaw_tol_rad,
      params_.ugv_max_angular_vel_rps);
  }

private:
  void on_tick()
  {
    const rclcpp::Time t = now();
    const bool odom_stale = !has_odom_ || (t - last_odom_time_).seconds() > params_.sensor_timeout_sec;
    const bool map_stale = !has_map_ || (t - last_map_time_).seconds() > params_.sensor_timeout_sec;
    const bool raw_map_stale = !has_raw_map_ || (t - last_raw_map_time_).seconds() > params_.sensor_timeout_sec;
    const bool path_stale = !has_path_ || (t - last_path_time_).seconds() > params_.sensor_timeout_sec;

    geometry_msgs::msg::Twist cmd;

    // Tell the controller the goal is gone: it only ever sees non-empty paths,
    // so it cannot drop latched state itself. Must stay above the
    // rotate-to-goal branch, which returns. (notes: ctrl-path-cleared-notice)
    if (latest_path_.poses.empty()) {
      controller_->on_path_cleared();
    }

    // Rotate-to-goal-yaw, UGV only, triggered by proximity to the active goal;
    // an empty path is deliberately not a trigger. active_goal_ is never
    // stale-checked: the navigator stops publishing it at the goal.
    // (notes: ctrl-rotate-to-goal-trigger)
    const bool rotate_applicable =
      !odom_stale && !is_uav_ && has_active_goal_ && goal_within_tolerance();

    if (!rotate_applicable) {
      // Leaving the region for any reason ends the episode. Notably this is
      // what happens when the robot is pushed back out of tolerance by the
      // recovery manoeuvre or by odometry drift: the rotation is abandoned
      // rather than resumed later against a goal the robot is no longer at.
      end_rotate("not at goal");
    } else {
      const double yaw_err = normalize_angle(
        yaw_from_quaternion(active_goal_.pose.orientation) -
        yaw_from_quaternion(latest_odom_.pose.pose.orientation));

      if (std::abs(yaw_err) <= params_.ugv_goal_yaw_tol_rad) {
        end_rotate("aligned");
      } else if (controller_->has_pending_maneuver()) {
        // Do not preempt a controller recovery: it is tick-counted, so
        // preempting suspends rather than ends it. It finishes or times out on
        // its own, then the rotation runs.
        // (notes: ctrl-rotate-defers-to-recovery)
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "rotate-to-goal deferred: controller manoeuvre in progress "
          "(yaw_err=%.2f rad)", yaw_err);
      } else {
        if (!rotating_) {
          rotating_ = true;
          rotate_start_ = t;
          ++rotate_episodes_;
          RCLCPP_INFO(get_logger(),
            "rotate-to-goal start: yaw_err=%.2f rad tol=%.2f (episode %ld)",
            yaw_err, params_.ugv_goal_yaw_tol_rad, rotate_episodes_);
        }
        const double held_sec = (t - rotate_start_).seconds();
        if (held_sec > kRotateBackstopSec) {
          // Bounds a rotation that no new goal position supersedes, since the
          // latch ignores staleness. Mission end (healthy) and a dead planner
          // both reach it; tell them apart by planner liveness, not
          // rotate_backstops_. (notes: ctrl-rotate-backstop)
          ++rotate_backstops_;
          RCLCPP_WARN(get_logger(),
            "rotate-to-goal ABANDONED after %.0fs (backstop %.0fs): yaw_err=%.2f "
            "rad still outside tol %.2f and no new goal position arrived. "
            "Stopping. This is expected at mission end and a fault only if the "
            "mission was still running -- check the planner, not this count. "
            "(%ld backstops)",
            held_sec, kRotateBackstopSec, yaw_err, params_.ugv_goal_yaw_tol_rad,
            rotate_backstops_);
          rotating_ = false;
          has_active_goal_ = false;   // do not re-arm on the same dead goal
          cmd_pub_->publish(cmd);     // zero twist: stop the wheels
          return;
        }
        // cmd is a default-constructed Twist, so linear.x is already 0 and the
        // rotation is strictly in place. Stated rather than assigned because the
        // requirement is on the BEHAVIOUR, not on the struct's initialiser.
        cmd.angular.z = std::clamp(
          params_.ugv_heading_kp * yaw_err,
          -params_.ugv_max_angular_vel_rps,
          params_.ugv_max_angular_vel_rps);
        cmd_pub_->publish(cmd);
        return;
      }
    }

    if (odom_stale || path_stale || latest_path_.poses.empty()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
        "Controller idle: odom_stale=%d path_stale=%d path_empty=%d",
        odom_stale, path_stale, latest_path_.poses.empty());
    }
    if (!odom_stale && !path_stale && !latest_path_.poses.empty()) {
      MapSnapshot map_snapshot;
      if (!map_stale) {
        map_snapshot = snapshot_from_occupancy_grid(latest_map_);
        if (!raw_map_stale) {
          const MapSnapshot raw_snapshot = snapshot_from_occupancy_grid(latest_raw_map_);
          // The grids come from independent publishers, and the UGV controller
          // indexes occupied_raw with map_snapshot.width, so a size mismatch
          // would read past the end. On mismatch, warn and keep the inflated
          // grid only. (notes: ctrl-raw-map-size-check)
          if (raw_snapshot.valid &&
              raw_snapshot.width == map_snapshot.width &&
              raw_snapshot.height == map_snapshot.height)
          {
            map_snapshot.occupied_raw = raw_snapshot.occupied;
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "raw local map geometry does not match the inflated one "
              "(%dx%d vs %dx%d, valid=%d) -- ignoring it and treating inflated "
              "cells as obstacles. Check that '%s' and '%s' come from the same "
              "mapper window.",
              raw_snapshot.width, raw_snapshot.height,
              map_snapshot.width, map_snapshot.height,
              static_cast<int>(raw_snapshot.valid),
              params_.local_map_raw_topic.c_str(),
              params_.local_map_topic.c_str());
          }
        }
      }
      cmd = controller_->compute_command(latest_odom_, latest_path_, map_snapshot);
    }
    cmd_pub_->publish(cmd);
  }

  /// True iff the robot is inside the arrival radius of the latched goal.
  /// Uses the same final_goal_tolerance() the navigator and both planners use,
  /// so all four agree on where "at the goal" is.
  bool goal_within_tolerance() const
  {
    if (!has_active_goal_ || !has_odom_) return false;
    const auto & g = active_goal_.pose.position;
    const auto & p = latest_odom_.pose.pose.position;
    return std::hypot(g.x - p.x, g.y - p.y) <= final_goal_tolerance(params_);
  }

  /// Retire a rotation episode. Idempotent, and silent unless one was running,
  /// because it is called on every tick that is not rotating.
  void end_rotate(const char * why)
  {
    if (!rotating_) return;
    rotating_ = false;
    RCLCPP_INFO(get_logger(), "rotate-to-goal end: %s", why);
  }

  NodeParameters params_;
  std::unique_ptr<ControllerBase> controller_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr active_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_uav_{false};
  bool has_odom_{false};
  bool has_map_{false};
  bool has_raw_map_{false};
  bool has_path_{false};
  nav_msgs::msg::Odometry latest_odom_;
  nav_msgs::msg::OccupancyGrid latest_map_;
  nav_msgs::msg::OccupancyGrid latest_raw_map_;
  nav_msgs::msg::Path latest_path_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_map_time_;
  rclcpp::Time last_raw_map_time_;
  rclcpp::Time last_path_time_;

  // Rotate-to-goal state. active_goal_ has no freshness test, so
  // kRotateBackstopSec is the only bound on a silent upstream. Keep it well
  // above the planner's goal_republish_sec; do not tighten it toward
  // goal_rotate_timeout_sec. (notes: ctrl-rotate-state-backstop-sizing)
  bool has_active_goal_{false};
  geometry_msgs::msg::PoseStamped active_goal_;
  bool rotating_{false};
  rclcpp::Time rotate_start_;
  long rotate_episodes_{0};
  long rotate_backstops_{0};
  static constexpr double kRotateBackstopSec = 30.0;
};

}  // namespace simple_nav_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_nav_3d::SimpleNavControllerNode>());
  rclcpp::shutdown();
  return 0;
}
