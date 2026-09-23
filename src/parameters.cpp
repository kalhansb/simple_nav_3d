// Moved comments: doc/simple_nav_3d_code_notes.md
#include "simple_nav_3d/parameters.hpp"

#include <stdexcept>

namespace simple_nav_3d
{

namespace
{

std::string join_ns_topic(const std::string & ns, const std::string & suffix)
{
  if (ns.empty()) {
    return suffix;
  }
  if (ns.back() == '/') {
    return ns + suffix;
  }
  return ns + "/" + suffix;
}

std::string with_fallback(const std::string & value, const std::string & fallback)
{
  return value.empty() ? fallback : value;
}

void require_positive(double value, const std::string & name)
{
  if (value <= 0.0) {
    throw std::runtime_error(name + " must be > 0");
  }
}

void require_non_empty(const std::string & value, const std::string & name)
{
  if (value.empty()) {
    throw std::runtime_error(name + " must not be empty");
  }
}

void require_absolute_topic(const std::string & value, const std::string & name)
{
  require_non_empty(value, name);
  if (value.front() != '/') {
    throw std::runtime_error(name + " must be absolute (start with '/')");
  }
}

}  // namespace

NodeParameters load_and_validate_params(rclcpp::Node & node)
{
  NodeParameters p;

  p.robot_name = node.declare_parameter<std::string>("robot.name", "robot1");
  p.mode = node.declare_parameter<std::string>("robot.mode", "ugv");
  p.robot_namespace = node.declare_parameter<std::string>("robot.namespace", "");
  p.pipeline_role = node.declare_parameter<std::string>("pipeline.role", "global");

  const std::string odom_topic_raw = node.declare_parameter<std::string>("topics.odom", "");
  const std::string points_topic_raw = node.declare_parameter<std::string>("topics.points", "");
  const std::string cmd_vel_topic_raw = node.declare_parameter<std::string>("topics.cmd_vel", "");
  const std::string goal_topic_raw = node.declare_parameter<std::string>("topics.goal", "");
  const std::string active_goal_topic_raw = node.declare_parameter<std::string>("topics.active_goal", "");
  const std::string global_path_topic_raw = node.declare_parameter<std::string>("topics.global_path", "");
  const std::string local_path_topic_raw = node.declare_parameter<std::string>("topics.local_path", "");
  const std::string local_map_topic_raw = node.declare_parameter<std::string>("topics.local_map", "");
  const std::string external_local_map_topic_raw = node.declare_parameter<std::string>("topics.external_local_map", "");
  const std::string global_map_topic_raw = node.declare_parameter<std::string>("topics.global_map", "");
  const std::string planning_map_topic_raw = node.declare_parameter<std::string>("topics.planning_map", "");
  const std::string local_planning_map_topic_raw = node.declare_parameter<std::string>("topics.local_planning_map", "");
  const std::string robot_body_polygon_topic_raw =
    node.declare_parameter<std::string>("topics.robot_body_polygon", "");

  if (p.robot_namespace.empty()) {
    p.robot_namespace = "/" + p.robot_name;
  }

  p.odom_topic = with_fallback(odom_topic_raw, join_ns_topic(p.robot_namespace, "odom_ground_truth"));
  p.points_topic = with_fallback(points_topic_raw, join_ns_topic(p.robot_namespace, "rgbd_points"));
  p.cmd_vel_topic = with_fallback(cmd_vel_topic_raw, join_ns_topic(p.robot_namespace, "cmd_vel"));
  p.goal_topic = with_fallback(goal_topic_raw, join_ns_topic(p.robot_namespace, "goal_pose"));
  p.active_goal_topic = with_fallback(
    active_goal_topic_raw, join_ns_topic(p.robot_namespace, "simple_nav_3d/active_goal"));
  p.global_path_topic = with_fallback(
    global_path_topic_raw, join_ns_topic(p.robot_namespace, "simple_nav_3d/global_path"));
  p.local_path_topic = with_fallback(
    local_path_topic_raw, join_ns_topic(p.robot_namespace, "simple_nav_3d/local_path"));
  p.local_map_topic = with_fallback(
    local_map_topic_raw, join_ns_topic(p.robot_namespace, "simple_nav_3d/local_map"));
  p.local_map_raw_topic = p.local_map_topic + "_raw";
  p.external_local_map_topic = external_local_map_topic_raw;  // Empty means disabled
  p.global_map_topic = with_fallback(
    global_map_topic_raw, join_ns_topic(p.robot_namespace, "simple_nav_3d/global_map"));
  p.planning_map_topic = with_fallback(planning_map_topic_raw, p.global_map_topic);
  p.local_planning_map_topic = local_planning_map_topic_raw;  // No fallback; must be set for role=local
  p.robot_body_polygon_topic = with_fallback(
    robot_body_polygon_topic_raw,
    join_ns_topic(p.robot_namespace, "simple_nav_3d/robot_body_polygon"));

  p.map_frame = node.declare_parameter<std::string>("frames.map", "map");
  p.odom_frame = node.declare_parameter<std::string>("frames.odom", "odom");
  p.base_frame = node.declare_parameter<std::string>("frames.base_link", "base_link");

  p.sensor_type = node.declare_parameter<std::string>("sensors.type", "rgbd");
  p.expected_rate_hz = node.declare_parameter<double>("sensors.expected_rate_hz", 15.0);
  p.sensor_timeout_sec = node.declare_parameter<double>("sensors.sensor_timeout_sec", 5.0);
  p.sensor_sync_window_sec = node.declare_parameter<double>("sensors.sync_window_sec", 0.10);
  p.min_range_m = node.declare_parameter<double>("sensors.min_range_m", 0.25);
  p.max_range_m = node.declare_parameter<double>("sensors.max_range_m", 10.0);
  p.voxel_size_m = node.declare_parameter<double>("sensors.voxel_size_m", 0.10);
  p.sensor_mount_height_m = node.declare_parameter<double>("sensors.sensor_mount_height_m", 0.272);
  p.min_obstacle_height_m = node.declare_parameter<double>("sensors.min_obstacle_height_m", 0.40);
  p.max_obstacle_height_m = node.declare_parameter<double>("sensors.max_obstacle_height_m", 1.0);

  p.planner = node.declare_parameter<std::string>("pipeline.planner", "planner_2d");
  p.controller = node.declare_parameter<std::string>("pipeline.controller", "local_controller_ugv");
  p.safety = node.declare_parameter<std::string>("pipeline.safety", "safety_ground");

  p.robot_body_length_m = node.declare_parameter<double>("robot.body_length_m", 1.0);
  p.robot_body_width_m = node.declare_parameter<double>("robot.body_width_m", 0.6);

  p.ugv_max_linear_vel_mps = node.declare_parameter<double>("ugv.max_linear_vel_mps", 1.2);
  p.ugv_max_angular_vel_rps = node.declare_parameter<double>("ugv.max_angular_vel_rps", 1.0);
  p.ugv_goal_xy_tol_m = node.declare_parameter<double>("ugv.goal_xy_tol_m", 0.2);
  // Default is the value the launch file passes, so declaring it changes
  // nothing for the dscovox pipeline. (notes: nav-ugv-goal-yaw-tol-default)
  p.ugv_goal_yaw_tol_rad = node.declare_parameter<double>("ugv.goal_yaw_tol_rad", 0.2);
  p.ugv_heading_kp = node.declare_parameter<double>("ugv.heading_kp", 1.5);
  p.ugv_linear_kp = node.declare_parameter<double>("ugv.linear_kp", 0.8);
  p.ugv_avoidance_hard_stop_distance_m = node.declare_parameter<double>(
    "ugv.avoidance_hard_stop_distance_m", 0.55);
  p.ugv_avoidance_slowdown_distance_m = node.declare_parameter<double>(
    "ugv.avoidance_slowdown_distance_m", 1.4);
  p.ugv_avoidance_max_range_m = node.declare_parameter<double>(
    "ugv.avoidance_max_range_m", 3.0);
  p.ugv_avoidance_arc_half_angle_deg = node.declare_parameter<double>(
    "ugv.avoidance_arc_half_angle_deg", 45.0);
  p.ugv_avoidance_turn_gain = node.declare_parameter<double>(
    "ugv.avoidance_turn_gain", 0.9);
  p.ugv_forward_blind_zone_m = node.declare_parameter<double>(
    "ugv.forward_blind_zone_m", 0.55);
  p.ugv_local_plan_window_size_m = node.declare_parameter<double>(
    "ugv.local_plan_window_size_m", 8.0);
  p.ugv_local_plan_resolution_m = node.declare_parameter<double>(
    "ugv.local_plan_resolution_m", 0.20);
  p.ugv_local_plan_inflation_m = node.declare_parameter<double>(
    "ugv.local_plan_inflation_m", 1.0);
  p.ugv_obstacle_persistence_sec = node.declare_parameter<double>(
    "ugv.obstacle_persistence_sec", 2.0);
  p.ugv_global_map_size_m = node.declare_parameter<double>(
    "ugv.global_map_size_m", 80.0);
  p.ugv_global_map_resolution_m = node.declare_parameter<double>(
    "ugv.global_map_resolution_m", 0.20);
  p.ugv_global_map_origin_x_m = node.declare_parameter<double>(
    "ugv.global_map_origin_x_m", -40.0);
  p.ugv_global_map_origin_y_m = node.declare_parameter<double>(
    "ugv.global_map_origin_y_m", -40.0);
  p.ugv_replan_cost_threshold_m = node.declare_parameter<double>(
    "ugv.replan_cost_threshold_m", 0.0);
  p.ugv_side_flip_cooldown_sec = node.declare_parameter<double>(
    "ugv.side_flip_cooldown_sec", 3.0);
  // Minimum seconds between global-role A* attempts. 0 disables decimation and
  // restores planning on every 100 ms tick. Matched to dscovox's 1.0 s
  // global_planning_map republish period — a shorter value cannot see newer
  // data, it can only spend more CPU on the same map.
  p.ugv_global_replan_period_sec = node.declare_parameter<double>(
    "ugv.global_replan_period_sec", 1.0);
  p.ugv_local_corridor_radius_m = node.declare_parameter<double>(
    "ugv.local_corridor_radius_m", 2.0);
  p.ugv_global_map_min_hits = node.declare_parameter<int>(
    "ugv.global_map_min_hits", 3);

  p.uav_max_vx_mps = node.declare_parameter<double>("uav.max_vx_mps", 1.5);
  p.uav_max_vy_mps = node.declare_parameter<double>("uav.max_vy_mps", 1.5);
  p.uav_max_vz_mps = node.declare_parameter<double>("uav.max_vz_mps", 0.8);
  p.uav_max_yaw_rate_rps = node.declare_parameter<double>("uav.max_yaw_rate_rps", 1.0);
  p.uav_goal_xyz_tol_m = node.declare_parameter<double>("uav.goal_xyz_tol_m", 0.3);
  p.uav_goal_yaw_tol_rad = node.declare_parameter<double>("uav.goal_yaw_tol_rad", 0.25);
  p.uav_nominal_height_m = node.declare_parameter<double>("uav.nominal_height_m", 1.5);

  p.ground_filter_bin_size_m = node.declare_parameter<double>(
    "ground_filter.bin_size_m", 0.5);
  p.ground_filter_tolerance_m = node.declare_parameter<double>(
    "ground_filter.tolerance_m", 0.10);
  p.ground_filter_valid_margin_m = node.declare_parameter<double>(
    "ground_filter.valid_margin_m", 0.30);

  require_non_empty(p.robot_name, "robot.name");
  require_non_empty(p.mode, "robot.mode");
  require_absolute_topic(p.robot_namespace, "robot.namespace");

  if (p.pipeline_role != "global" && p.pipeline_role != "local") {
    throw std::runtime_error("pipeline.role must be 'global' or 'local'");
  }
  // local_path_topic is required for any node with pipeline.role==local because
  // both the local planner (output) and the controller (input) read it. The
  // local_planning_map_topic is only needed by the planner; we let the planner
  // node enforce it at subscription time so the controller doesn't need it set.
  if (p.pipeline_role == "local") {
    require_absolute_topic(
      p.local_path_topic, "topics.local_path (required when pipeline.role=local)");
  }

  require_absolute_topic(p.odom_topic, "topics.odom");
  require_absolute_topic(p.points_topic, "topics.points");
  require_absolute_topic(p.cmd_vel_topic, "topics.cmd_vel");
  require_absolute_topic(p.goal_topic, "topics.goal");
  require_absolute_topic(p.active_goal_topic, "topics.active_goal");
  require_absolute_topic(p.global_path_topic, "topics.global_path");
  require_absolute_topic(p.local_path_topic, "topics.local_path");
  require_absolute_topic(p.local_map_topic, "topics.local_map");
  require_absolute_topic(p.global_map_topic, "topics.global_map");
  require_absolute_topic(p.planning_map_topic, "topics.planning_map");
  require_absolute_topic(p.robot_body_polygon_topic, "topics.robot_body_polygon");

  require_non_empty(p.map_frame, "frames.map");
  require_non_empty(p.odom_frame, "frames.odom");
  require_non_empty(p.base_frame, "frames.base_link");

  if (p.sensor_type != "rgbd") {
    throw std::runtime_error("sensors.type must be 'rgbd'");
  }

  require_positive(p.expected_rate_hz, "sensors.expected_rate_hz");
  require_positive(p.sensor_timeout_sec, "sensors.sensor_timeout_sec");
  require_positive(p.sensor_sync_window_sec, "sensors.sync_window_sec");
  require_positive(p.max_range_m, "sensors.max_range_m");
  require_positive(p.voxel_size_m, "sensors.voxel_size_m");
  require_positive(p.ugv_max_linear_vel_mps, "ugv.max_linear_vel_mps");
  require_positive(p.ugv_max_angular_vel_rps, "ugv.max_angular_vel_rps");
  require_positive(p.ugv_goal_xy_tol_m, "ugv.goal_xy_tol_m");
  require_positive(p.ugv_goal_yaw_tol_rad, "ugv.goal_yaw_tol_rad");
  require_positive(p.ugv_heading_kp, "ugv.heading_kp");
  require_positive(p.ugv_linear_kp, "ugv.linear_kp");
  require_positive(
    p.ugv_avoidance_hard_stop_distance_m, "ugv.avoidance_hard_stop_distance_m");
  require_positive(
    p.ugv_avoidance_slowdown_distance_m, "ugv.avoidance_slowdown_distance_m");
  require_positive(p.ugv_avoidance_max_range_m, "ugv.avoidance_max_range_m");
  require_positive(p.ugv_avoidance_arc_half_angle_deg, "ugv.avoidance_arc_half_angle_deg");
  require_positive(p.ugv_avoidance_turn_gain, "ugv.avoidance_turn_gain");
  require_positive(p.ugv_forward_blind_zone_m, "ugv.forward_blind_zone_m");
  require_positive(p.ugv_local_plan_window_size_m, "ugv.local_plan_window_size_m");
  require_positive(p.ugv_local_plan_resolution_m, "ugv.local_plan_resolution_m");
  if (p.ugv_local_plan_inflation_m < 0.0) {
    throw std::runtime_error("ugv.local_plan_inflation_m must be >= 0");
  }
  if (p.ugv_obstacle_persistence_sec < 0.0) {
    throw std::runtime_error("ugv.obstacle_persistence_sec must be >= 0");
  }
  require_positive(p.ugv_global_map_size_m, "ugv.global_map_size_m");
  require_positive(p.ugv_global_map_resolution_m, "ugv.global_map_resolution_m");
  if (p.ugv_replan_cost_threshold_m < 0.0) {
    throw std::runtime_error("ugv.replan_cost_threshold_m must be >= 0");
  }
  if (p.ugv_local_corridor_radius_m < 0.0) {
    throw std::runtime_error("ugv.local_corridor_radius_m must be >= 0");
  }
  if (p.ugv_global_replan_period_sec < 0.0) {
    throw std::runtime_error("ugv.global_replan_period_sec must be >= 0");
  }
  if (p.ugv_global_map_min_hits < 1) {
    throw std::runtime_error("ugv.global_map_min_hits must be >= 1");
  }
  require_positive(p.ground_filter_bin_size_m, "ground_filter.bin_size_m");
  require_positive(p.ground_filter_tolerance_m, "ground_filter.tolerance_m");
  require_positive(p.ground_filter_valid_margin_m, "ground_filter.valid_margin_m");
  require_positive(p.robot_body_length_m, "robot.body_length_m");
  require_positive(p.robot_body_width_m, "robot.body_width_m");

  if (p.min_range_m < 0.0) {
    throw std::runtime_error("sensors.min_range_m must be >= 0");
  }
  if (p.sensor_mount_height_m < 0.0) {
    throw std::runtime_error("sensors.sensor_mount_height_m must be >= 0");
  }
  if (p.max_range_m <= p.min_range_m) {
    throw std::runtime_error("sensors.max_range_m must be > sensors.min_range_m");
  }
  if (p.max_obstacle_height_m <= p.min_obstacle_height_m) {
    throw std::runtime_error(
      "sensors.max_obstacle_height_m must be > sensors.min_obstacle_height_m");
  }
  if (p.ugv_avoidance_slowdown_distance_m <= p.ugv_avoidance_hard_stop_distance_m) {
    throw std::runtime_error(
      "ugv.avoidance_slowdown_distance_m must be > ugv.avoidance_hard_stop_distance_m");
  }

  if (p.mode == "ugv") {
    if (p.planner != "planner_2d" || p.controller != "local_controller_ugv" ||
      p.safety != "safety_ground")
    {
      throw std::runtime_error(
        "UGV mode requires pipeline.planner=planner_2d, "
        "pipeline.controller=local_controller_ugv, pipeline.safety=safety_ground");
    }
  } else if (p.mode == "uav") {
    if (p.planner != "planner_3d" || p.controller != "flight_controller_uav" ||
      p.safety != "safety_air")
    {
      throw std::runtime_error(
        "UAV mode requires pipeline.planner=planner_3d, "
        "pipeline.controller=flight_controller_uav, pipeline.safety=safety_air");
    }
  } else {
    throw std::runtime_error("robot.mode must be 'ugv' or 'uav'");
  }

  return p;
}

}  // namespace simple_nav_3d
