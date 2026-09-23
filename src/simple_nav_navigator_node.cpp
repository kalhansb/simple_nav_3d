// Moved comments: doc/simple_nav_3d_code_notes.md
#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/grid_utils.hpp"

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

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      params_.goal_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Ignore a re-publish of the goal already being driven. Orientation is
        // part of the comparison, so a yaw-only revision passes through as a
        // new goal. (notes: nav-ignore-same-goal)
        if (has_active_goal_ && same_goal_pose(active_goal_.pose, msg->pose)) {
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

    timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this]() { on_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "navigator node started: goal_in=%s active_goal_out=%s",
      params_.goal_topic.c_str(),
      params_.active_goal_topic.c_str());
  }

private:
  void on_tick()
  {
    if (!has_active_goal_) {
      return;
    }

    if (has_odom_) {
      const auto & p = latest_odom_.pose.pose.position;
      const auto & g = active_goal_.pose.position;
      const double dist = (params_.mode == "uav")
        ? std::sqrt((g.x - p.x) * (g.x - p.x) + (g.y - p.y) * (g.y - p.y) + (g.z - p.z) * (g.z - p.z))
        : std::hypot(g.x - p.x, g.y - p.y);
      if (dist < final_goal_tolerance(params_)) {
        has_active_goal_ = false;
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
  }

  NodeParameters params_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr active_goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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
};

}  // namespace simple_nav_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_nav_3d::SimpleNavNavigatorNode>());
  rclcpp::shutdown();
  return 0;
}
