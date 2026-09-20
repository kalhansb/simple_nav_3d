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
        // Ignore a re-publish of the goal already being driven. ORIENTATION IS
        // PART OF THE COMPARISON: this test used to be position-only, which
        // silently discarded every yaw-only goal revision before the controller
        // could see it. See same_goal_pose() for why that mattered and why it
        // was latent rather than observed.
        if (has_active_goal_ && same_goal_pose(active_goal_.pose, msg->pose)) {
          return;
        }
        // Distinguish a genuinely NEW goal from a re-arm of the one just
        // finished. Both are accepted — re-arming is how a goal published while
        // this node was down gets picked up at all — but they are not equally
        // interesting, and conflating them made the robot log unreadable: after
        // arrival `has_active_goal_` is false, so an upstream that re-publishes
        // an unchanged goal (the planner's keep-alive, or its EXPLOIT re-anchor
        // at tick rate while the controller rotates) produced an
        // accepted/reached INFO pair per re-publish — up to 10 per second, for
        // as long as the rotation lasted.
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
        // Same reasoning as the re-arm branch above: an upstream that keeps
        // re-publishing an unchanged goal after arrival makes this fire once per
        // re-publish. The FIRST arrival on a given goal is the interesting one
        // and stays at INFO; the repeats drop to DEBUG. `active_goal_` is still
        // intact here (only the flag was cleared), so the comparison is against
        // the goal that was just reached.
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
  /// A goal has been accepted at some point, so `active_goal_` holds a real
  /// pose. Distinct from has_active_goal_, which is cleared on arrival: the
  /// re-arm test below needs "what was the last goal" AFTER it stopped being
  /// active, and reading a default-constructed active_goal_ before the first
  /// goal would make the origin-with-identity-orientation compare equal to a
  /// real goal at the origin.
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
