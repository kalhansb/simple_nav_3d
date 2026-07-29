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
        // Ignore duplicate goals (exact same position)
        if (has_active_goal_) {
          const auto & g = active_goal_.pose.position;
          if (msg->pose.position.x == g.x &&
              msg->pose.position.y == g.y &&
              msg->pose.position.z == g.z) return;
        }
        active_goal_ = *msg;
        has_active_goal_ = true;
        active_goal_pub_->publish(active_goal_);
        RCLCPP_INFO(get_logger(), "Navigator accepted new goal (x=%.2f y=%.2f z=%.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
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
        RCLCPP_INFO(get_logger(), "Navigator goal reached (dist=%.2f m)", dist);
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
  bool has_odom_{false};
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
