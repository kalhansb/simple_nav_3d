#ifndef SIMPLE_NAV_3D__PARAMETERS_HPP_
#define SIMPLE_NAV_3D__PARAMETERS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace simple_nav_3d
{

struct NodeParameters
{
  std::string robot_name;
  std::string mode;
  std::string robot_namespace;

  // Pipeline role: "global" or "local". Drives which input map / output path
  // topic the planner uses, and which path topic the controller subscribes to.
  // - "global": planner reads `planning_map_topic`, publishes `global_path_topic`;
  //             controller subscribes to `global_path_topic`.
  // - "local":  planner reads `local_planning_map_topic`, publishes `local_path_topic`;
  //             controller subscribes to `local_path_topic`.
  // Two planner instances can run in parallel with different roles.
  std::string pipeline_role;

  std::string odom_topic;
  std::string points_topic;
  std::string cmd_vel_topic;
  std::string goal_topic;
  std::string active_goal_topic;
  std::string global_path_topic;
  std::string local_path_topic;
  std::string local_map_topic;
  std::string local_map_raw_topic;
  std::string external_local_map_topic;  // If non-empty, costmap subscribes to this instead of building from points
  std::string global_map_topic;
  std::string planning_map_topic;
  std::string local_planning_map_topic;  // Required when pipeline_role == "local"
  std::string robot_body_polygon_topic;

  std::string map_frame;
  std::string odom_frame;
  std::string base_frame;

  std::string sensor_type;
  double expected_rate_hz;
  double sensor_timeout_sec;
  double sensor_sync_window_sec;
  double min_range_m;
  double max_range_m;
  double voxel_size_m;
  double sensor_mount_height_m;
  double min_obstacle_height_m;
  double max_obstacle_height_m;

  std::string planner;
  std::string controller;
  std::string safety;

  double robot_body_length_m;
  double robot_body_width_m;

  double ugv_max_linear_vel_mps;
  double ugv_max_angular_vel_rps;
  double ugv_goal_xy_tol_m;
  double ugv_heading_kp;
  double ugv_linear_kp;
  double ugv_avoidance_hard_stop_distance_m;
  double ugv_avoidance_slowdown_distance_m;
  double ugv_avoidance_max_range_m;
  double ugv_avoidance_arc_half_angle_deg;
  double ugv_avoidance_turn_gain;
  double ugv_forward_blind_zone_m;
  double ugv_local_plan_window_size_m;
  double ugv_local_plan_resolution_m;
  double ugv_local_plan_inflation_m;
  double ugv_obstacle_persistence_sec;
  double ugv_global_map_size_m;
  double ugv_global_map_resolution_m;
  double ugv_global_map_origin_x_m;
  double ugv_global_map_origin_y_m;
  double ugv_replan_cost_threshold_m;
  double ugv_side_flip_cooldown_sec;
  double ugv_global_replan_period_sec;
  // Half-width of the corridor the local planner builds around the global
  // path. Cells outside the corridor are masked off so the local planner
  // refines within global's chosen homotopy. Used only when role==local and
  // a global path is available; if A* fails inside the corridor, the local
  // planner falls back to a free A* over the whole local map.
  double ugv_local_corridor_radius_m;
  int ugv_global_map_min_hits;

  double uav_max_vx_mps;
  double uav_max_vy_mps;
  double uav_max_vz_mps;
  double uav_max_yaw_rate_rps;
  double uav_goal_xyz_tol_m;
  double uav_goal_yaw_tol_rad;
  double uav_nominal_height_m;

  double ground_filter_bin_size_m;
  double ground_filter_tolerance_m;
  double ground_filter_valid_margin_m;
};

NodeParameters load_and_validate_params(rclcpp::Node & node);

inline double final_goal_tolerance(const NodeParameters & params)
{
  if (params.mode == "uav") {
    return std::max(0.05, params.uav_goal_xyz_tol_m);
  }
  return std::max(0.05, params.ugv_goal_xy_tol_m);
}

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__PARAMETERS_HPP_
