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

    // D2. The goal itself, not the path's last waypoint, is where the desired
    // yaw comes from now.
    //
    // The path was never a sound channel for it. The planner stamps the goal
    // orientation onto the final waypoint only when that waypoint is already
    // near the goal, so the orientation is absent for most of the approach and
    // appears late; and the old latch here accepted it only if the quaternion
    // was non-identity, which makes a COMMANDED yaw of zero -- a perfectly
    // ordinary heading, and the one an unstamped waypoint also carries --
    // indistinguishable from "no yaw was requested". A goal facing +x therefore
    // never produced a rotation at all. Worse, the latch was never cleared when
    // the goal changed, so a rotation interrupted by a new goal stayed armed and
    // the robot would later turn to the OLD goal's heading on arriving at the
    // new one.
    //
    // Reading the active goal directly removes all three problems: every pose on
    // this topic has a meaningful orientation, a new goal replaces the old one
    // by construction, and the yaw is available from the moment the goal is.
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

    // Tell the controller the goal is gone. It cannot see this for itself:
    // every path below either returns early or skips compute_command entirely
    // when the path is empty, so a controller only ever observes non-empty
    // paths and cannot drop state it latched for the goal that just ended.
    // Placed above the rotate-to-goal branch because that branch returns.
    if (latest_path_.poses.empty()) {
      controller_->on_path_cleared();
    }

    // D2. Rotate-to-goal-yaw.
    //
    // The trigger is proximity to the active goal, not an empty path. The old
    // condition worked only because the planner happens to publish one empty
    // path when it retires a goal; it therefore depended on that single message
    // arriving and on the controller's copy of the path being the one that went
    // empty. Proximity is the condition the upstream arrival test actually uses,
    // so triggering on it makes the two agree by construction instead of by
    // coincidence.
    //
    // The empty path is NOT kept as a second trigger, and that is deliberate. A
    // path empties for reasons that have nothing to do with arriving: the
    // planner clears state when the navigator goes quiet, when a goal is
    // withdrawn, when its inputs starve. Rotating on any of those would spin the
    // robot in place wherever it happened to be standing -- possibly tens of
    // metres from the goal -- to face a heading that only means anything at the
    // goal. Proximity is not merely a better trigger than an empty path, it is
    // the precondition that makes the manoeuvre meaningful at all.
    //
    // Nothing is lost by dropping it: rotation does not translate, so once the
    // robot is inside the radius it stays inside it, and the latched goal keeps
    // the test true for the whole episode even after the navigator falls silent.
    //
    // WHY THE GOAL LATCH IS NEVER STALE-CHECKED: the navigator deliberately
    // STOPS publishing active_goal the moment the robot is within
    // final_goal_tolerance -- which is precisely when this branch needs the goal
    // most. Any freshness test on active_goal_ would therefore expire the target
    // in the middle of every rotation it is supposed to serve. The latch is
    // instead retired by events (a new goal position, alignment, or the backstop
    // below), and that is why the backstop has to exist.
    //
    // UGV only. The test is planar and the rate limit it clamps against is the
    // UGV's; a UAV already servos its own yaw toward the path inside
    // UavController::compute_command, so there is nothing here for it to do.
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
        // Do not cut into a recovery. Rotating away mid-back-up would abandon
        // the manoeuvre in the state it was invoked to escape, and the recovery
        // is tick-counted so preempting it suspends rather than ends it. It
        // finishes or times out on its own within a bounded number of ticks,
        // and the rotation gets its turn afterwards.
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
          // The latch is deliberately immune to staleness (see above), so
          // without this bound a rotation whose goal is never superseded would
          // be commanded forever. What ends an episode normally is a NEW goal
          // POSITION arriving on active_goal -- the planner's next goal, relayed
          // by the navigator. This fires when no such goal comes for 30 s.
          //
          // THE WARN DOES NOT DIAGNOSE, and must not be read as if it did. Two
          // very different things reach this line and look identical from here:
          //   - the mission ended. The planner stops issuing goals, so nothing
          //     supersedes the latched one. This is HEALTHY and, in generation
          //     9, expected: the planner no longer gates arrival on yaw at all
          //     (`yaw_required = (phase_ == EXPLOIT) || !fov_is_omnidirectional_`
          //     in explo_planner_node.cpp is false once the FOV is
          //     omnidirectional and exploitation is off), so its own
          //     `failGoal("budget-rotate")` deadline is UNREACHABLE by
          //     construction and this backstop is the only rotate bound left;
          //   - the planner died mid-mission. A genuine fault.
          // Separate them with the planner's own liveness, never with this
          // count: a nonzero rotate_backstops_ at the end of a clean run is the
          // normal reading, not a defect.
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
          // The two grids come from two independent publishers, so nothing in
          // the type system makes them the same size. Downstream, the UGV
          // controller indexes occupied_raw with an index it derived from
          // map_snapshot.width -- so a mismatch is not a wrong answer, it is a
          // read past the end of the vector. The grids are expected to agree
          // (same node, same rolling window) and a disagreement means a
          // configuration fault, so it is reported rather than papered over,
          // and the inflated grid is left in place as the fallback.
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

  // D2. Rotate-to-goal state. The target comes from the active goal, not from
  // the path; see the subscription for why the path was the wrong source.
  //
  // active_goal_ is held without a freshness test on purpose -- the navigator
  // stops publishing it exactly when the rotation needs it. kRotateBackstopSec
  // is the price of that: the only bound left on a rotation whose upstream has
  // gone silent. 30 s sits far above the interval at which a working planner
  // supersedes a goal it has not finished with (goal_republish_sec is 5 s, and
  // a republish of the SAME pose does not end an episode -- only a new POSITION
  // does), so it does not preempt normal operation.
  //
  // It is NOT sized against the planner's goal_rotate_timeout_sec of 15 s.
  // That deadline was the original justification and it is unreachable in
  // generation 9 (see the backstop branch), so this is a backstop with nothing
  // behind it. Reaching it is therefore not by itself a fault -- mission end
  // reaches it too. The WARN says so; do not re-tighten the constant toward 15
  // on the strength of a deadline that no longer fires.
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
