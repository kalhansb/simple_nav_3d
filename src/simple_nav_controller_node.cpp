#include <chrono>
#include <memory>

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
        // Latch goal yaw from the last waypoint if it has a non-identity
        // orientation. When the path goes empty (goal reached by position),
        // we rotate in place to this yaw.
        if (!msg->poses.empty()) {
          const auto & q = msg->poses.back().pose.orientation;
          if (std::abs(q.w) < 0.999 || std::abs(q.z) > 0.01) {
            pending_goal_yaw_ = yaw_from_quaternion(q);
            has_pending_goal_yaw_ = true;
          }
        }
      });

    timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this]() { on_tick(); });

    RCLCPP_INFO(
      get_logger(),
      "controller node started: role=%s path_in=%s local_map=%s cmd_vel=%s controller=%s",
      params_.pipeline_role.c_str(),
      path_input_topic.c_str(),
      params_.local_map_topic.c_str(),
      params_.cmd_vel_topic.c_str(),
      controller_->name().c_str());
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

    // Rotate-to-goal: path went empty (navigator cleared goal on position),
    // but we have a latched goal yaw. Rotate in place until aligned.
    if (!odom_stale && latest_path_.poses.empty() && has_pending_goal_yaw_) {
      double cur_yaw = yaw_from_quaternion(latest_odom_.pose.pose.orientation);
      double yaw_err = normalize_angle(pending_goal_yaw_ - cur_yaw);
      if (std::abs(yaw_err) > 0.15) {  // ~8.5 deg tolerance
        cmd.angular.z = std::clamp(1.5 * yaw_err, -1.5, 1.5);
        cmd_pub_->publish(cmd);
        return;
      }
      // Rotation complete.
      has_pending_goal_yaw_ = false;
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
          map_snapshot.occupied_raw = raw_snapshot.occupied;
        }
      }
      cmd = controller_->compute_command(latest_odom_, latest_path_, map_snapshot);
    }
    cmd_pub_->publish(cmd);
  }

  NodeParameters params_;
  std::unique_ptr<ControllerBase> controller_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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

  // Rotate-to-goal state. Latched from the last non-empty path's final
  // waypoint orientation. Used to rotate in place after the nav stack
  // clears the goal on position.
  bool   has_pending_goal_yaw_{false};
  double pending_goal_yaw_{0.0};
};

}  // namespace simple_nav_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_nav_3d::SimpleNavControllerNode>());
  rclcpp::shutdown();
  return 0;
}
