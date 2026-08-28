#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_nav_3d/mapping/occupancy_grid_utils.hpp"
#include "simple_nav_3d/grid_utils.hpp"
#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/planner_factory.hpp"
#include "simple_nav_3d/planners/planner_base.hpp"
#include "simple_nav_3d/planners/uav_planner.hpp"
#include "scovox_msgs/srv/get_region.hpp"

namespace simple_nav_3d
{

using namespace std::chrono_literals;

namespace
{

double path_length(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double total = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

nav_msgs::msg::Path truncate_path_by_length(const nav_msgs::msg::Path & path, double max_len_m)
{
  if (path.poses.size() < 2 || max_len_m <= 0.0) {
    return path;
  }

  nav_msgs::msg::Path out;
  out.header = path.header;
  out.poses.push_back(path.poses.front());

  double acc = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (acc + seg <= max_len_m) {
      out.poses.push_back(path.poses[i]);
      acc += seg;
      continue;
    }

    const double remain = max_len_m - acc;
    if (remain > 1e-3 && seg > 1e-6) {
      const double t = remain / seg;
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = a.x + t * (b.x - a.x);
      p.pose.position.y = a.y + t * (b.y - a.y);
      p.pose.position.z = a.z + t * (b.z - a.z);
      p.pose.orientation = path.poses[i].pose.orientation;
      out.poses.push_back(p);
    }
    break;
  }

  if (out.poses.size() < 2) {
    out = path;
  }
  return out;
}

int departure_side_wrt_goal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & current,
  const geometry_msgs::msg::Point & goal,
  double min_departure_dist)
{
  if (path.poses.size() < 2) {
    return 0;
  }

  geometry_msgs::msg::Point anchor = path.poses.front().pose.position;
  bool found = false;
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    if (std::hypot(p.x - current.x, p.y - current.y) >= min_departure_dist) {
      anchor = p;
      found = true;
      break;
    }
  }
  if (!found) {
    anchor = path.poses.back().pose.position;
  }

  const double gx = goal.x - current.x;
  const double gy = goal.y - current.y;
  const double px = anchor.x - current.x;
  const double py = anchor.y - current.y;
  const double gnorm = std::hypot(gx, gy);
  const double pnorm = std::hypot(px, py);
  if (gnorm < 1e-6 || pnorm < 1e-6) {
    return 0;
  }

  const double cross = gx * py - gy * px;
  if (cross > 1e-6) {
    return 1;
  }
  if (cross < -1e-6) {
    return -1;
  }
  return 0;
}

const char * side_name(int side)
{
  if (side > 0) {
    return "left";
  }
  if (side < 0) {
    return "right";
  }
  return "center";
}

bool path_still_valid(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & grid)
{
  if (path.poses.empty()) return false;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const double res = static_cast<double>(grid.info.resolution);
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (w <= 0 || h <= 0 || res <= 0.0) return false;

  for (const auto & pose : path.poses) {
    int gx = static_cast<int>(std::floor((pose.pose.position.x - ox) / res));
    int gy = static_cast<int>(std::floor((pose.pose.position.y - oy) / res));
    if (gx < 0 || gy < 0 || gx >= w || gy >= h) continue;  // OOB/unknown ok
    if (grid.data[gy * w + gx] >= 50) return false;  // goes through occupied
  }
  return true;
}

// True iff (x, y) lies inside the local map's axis-aligned envelope.
bool point_in_map(const MapSnapshot & m, double x, double y)
{
  return x >= m.origin_x &&
         y >= m.origin_y &&
         x < m.origin_x + m.width  * m.resolution &&
         y < m.origin_y + m.height * m.resolution;
}

// Pick the slice of the global path that we want the local planner to refine.
//
// Walks the global path from the waypoint nearest the robot forward, keeping
// every waypoint that lies inside the local map. Stops at the first one that
// exits the window — that exit point becomes the local target. If the global
// path's actual endpoint is inside the window we use that endpoint instead so
// the local planner finishes the goal cleanly.
//
// Returns false if the global path has no waypoint inside the local window
// (e.g., the global path is entirely outside the rolling crop, or empty).
bool slice_global_path_for_local(
  const nav_msgs::msg::Path & global_path,
  const MapSnapshot & local_map,
  const geometry_msgs::msg::Point & robot,
  std::vector<geometry_msgs::msg::Point> & corridor_centerline,
  geometry_msgs::msg::Point & local_target)
{
  corridor_centerline.clear();
  if (global_path.poses.empty() || !local_map.valid) return false;

  // Find the global-path waypoint nearest the robot — that's where the
  // robot is "on" the global path right now.
  size_t nearest_i = 0;
  double nearest_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < global_path.poses.size(); ++i) {
    const auto & p = global_path.poses[i].pose.position;
    const double d2 = (p.x - robot.x) * (p.x - robot.x) + (p.y - robot.y) * (p.y - robot.y);
    if (d2 < nearest_d2) {
      nearest_d2 = d2;
      nearest_i = i;
    }
  }

  // Walk forward; collect waypoints that are inside the local window.
  // The first time we see an out-of-window waypoint we stop — that defines
  // the exit point. If we never exit the window the path's last waypoint
  // (the actual goal) becomes our local target.
  for (size_t i = nearest_i; i < global_path.poses.size(); ++i) {
    const auto & p = global_path.poses[i].pose.position;
    if (!point_in_map(local_map, p.x, p.y)) break;
    corridor_centerline.push_back(p);
  }

  if (corridor_centerline.empty()) return false;
  local_target = corridor_centerline.back();
  return true;
}

// Stamp a corridor of half-width `radius_m` around the centerline into the
// local map by marking every cell *outside* the corridor as occupied. This
// hard-constrains the local A* to refine within global's homotopy. The
// corridor is rasterized by walking the centerline at sub-cell resolution
// and unioning the open cells around each sample point.
//
// Cells that are already occupied stay occupied — the corridor mask never
// frees a previously-blocked cell. The robot's own start cell is always
// included so A* can start.
MapSnapshot apply_corridor_mask(
  const MapSnapshot & in,
  const std::vector<geometry_msgs::msg::Point> & centerline,
  double radius_m,
  const geometry_msgs::msg::Point & robot)
{
  MapSnapshot out = in;
  if (!in.valid || centerline.empty() || radius_m <= 0.0) return out;

  const int w = in.width;
  const int h = in.height;
  const double res = in.resolution;
  const double ox = in.origin_x;
  const double oy = in.origin_y;

  // 1) Build a "free cell" mask covering the corridor union.
  std::vector<uint8_t> in_corridor(static_cast<size_t>(w) * static_cast<size_t>(h), 0);
  const int rcells = std::max(1, static_cast<int>(std::ceil(radius_m / res)));
  const double r2 = radius_m * radius_m;

  // Make sure the robot's own footprint is in-corridor.
  std::vector<geometry_msgs::msg::Point> samples = centerline;
  samples.push_back(robot);

  // Walk the centerline densely (one sample per ~half a cell) so the
  // corridor isn't a string of disjoint discs around sparse waypoints.
  std::vector<geometry_msgs::msg::Point> dense;
  dense.reserve(samples.size() * 4);
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
    const auto & a = samples[i];
    const auto & b = samples[i + 1];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / (0.5 * res))));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      geometry_msgs::msg::Point p;
      p.x = a.x + t * (b.x - a.x);
      p.y = a.y + t * (b.y - a.y);
      dense.push_back(p);
    }
  }
  if (dense.empty()) dense = samples;

  for (const auto & p : dense) {
    const int cx = static_cast<int>(std::floor((p.x - ox) / res));
    const int cy = static_cast<int>(std::floor((p.y - oy) / res));
    for (int dy = -rcells; dy <= rcells; ++dy) {
      const int ny = cy + dy;
      if (ny < 0 || ny >= h) continue;
      for (int dx = -rcells; dx <= rcells; ++dx) {
        const int nx = cx + dx;
        if (nx < 0 || nx >= w) continue;
        const double wx = ox + (nx + 0.5) * res;
        const double wy = oy + (ny + 0.5) * res;
        const double dxw = wx - p.x;
        const double dyw = wy - p.y;
        if (dxw * dxw + dyw * dyw <= r2) {
          in_corridor[static_cast<size_t>(ny) * static_cast<size_t>(w) + static_cast<size_t>(nx)] = 1;
        }
      }
    }
  }

  // 2) Mark every out-of-corridor cell as occupied. Already-occupied cells
  //    stay occupied. occupied_raw (the pre-inflation copy) is left alone so
  //    the controller's emergency-stop logic still sees true obstacles.
  for (size_t i = 0; i < out.occupied.size() && i < in_corridor.size(); ++i) {
    if (in_corridor[i] == 0) out.occupied[i] = 1;
  }
  return out;
}

/// Build a VoxelGrid3D from scovox GetRegion response with 3D inflation.
VoxelGrid3D build_voxel_grid(
  const scovox_msgs::msg::ScovoxMap & dss_map,
  double grid_resolution,
  double occ_threshold,
  double inflation_m)
{
  VoxelGrid3D grid;
  if (dss_map.voxels.empty()) {
    return grid;
  }

  // Find bounding box of all voxels.
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x, min_z = min_x;
  double max_x = -min_x, max_y = -min_x, max_z = -min_x;

  for (const auto & v : dss_map.voxels) {
    min_x = std::min(min_x, static_cast<double>(v.position.x));
    min_y = std::min(min_y, static_cast<double>(v.position.y));
    min_z = std::min(min_z, static_cast<double>(v.position.z));
    max_x = std::max(max_x, static_cast<double>(v.position.x));
    max_y = std::max(max_y, static_cast<double>(v.position.y));
    max_z = std::max(max_z, static_cast<double>(v.position.z));
  }

  // Add margin (enough for inflation + path clearance).
  const double margin = std::max(grid_resolution, inflation_m + grid_resolution);
  min_x -= margin; min_y -= margin; min_z -= margin;
  max_x += margin; max_y += margin; max_z += margin;

  grid.resolution = grid_resolution;
  grid.origin_x = min_x;
  grid.origin_y = min_y;
  grid.origin_z = min_z;
  grid.size_x = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / grid_resolution)));
  grid.size_y = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / grid_resolution)));
  grid.size_z = std::max(1, static_cast<int>(std::ceil((max_z - min_z) / grid_resolution)));

  // Cap total size to avoid OOM.
  const size_t total = static_cast<size_t>(grid.size_x) * grid.size_y * grid.size_z;
  if (total > 10000000) {  // 10M cells max
    return grid;  // invalid
  }

  grid.occupied.assign(total, 0);

  // Mark raw occupied cells.
  std::vector<uint8_t> raw_occupied(total, 0);
  for (const auto & v : dss_map.voxels) {
    const double p_occ = (v.a_occ + v.a_free > 0.0f)
      ? static_cast<double>(v.a_occ) / (v.a_occ + v.a_free)
      : 0.0;
    if (p_occ < occ_threshold) {
      continue;
    }

    const int gx = static_cast<int>(std::floor((v.position.x - grid.origin_x) / grid_resolution));
    const int gy = static_cast<int>(std::floor((v.position.y - grid.origin_y) / grid_resolution));
    const int gz = static_cast<int>(std::floor((v.position.z - grid.origin_z) / grid_resolution));
    if (grid.in_bounds(gx, gy, gz)) {
      raw_occupied[grid.flatten(gx, gy, gz)] = 1;
    }
  }

  // 3D spherical inflation.
  const int inflate_cells = static_cast<int>(std::ceil(inflation_m / grid_resolution));
  const double inflate_r_sq = inflation_m * inflation_m;

  for (int z = 0; z < grid.size_z; ++z) {
    for (int y = 0; y < grid.size_y; ++y) {
      for (int x = 0; x < grid.size_x; ++x) {
        if (raw_occupied[grid.flatten(x, y, z)] == 0) {
          continue;
        }
        // Inflate this occupied cell in all directions.
        for (int dz = -inflate_cells; dz <= inflate_cells; ++dz) {
          for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
            for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
              // Spherical check: only inflate within radius.
              const double dist_sq =
                (dx * dx + dy * dy + dz * dz) * grid_resolution * grid_resolution;
              if (dist_sq > inflate_r_sq) {
                continue;
              }
              const int nx = x + dx;
              const int ny = y + dy;
              const int nz = z + dz;
              if (grid.in_bounds(nx, ny, nz)) {
                grid.occupied[grid.flatten(nx, ny, nz)] = 1;
              }
            }
          }
        }
      }
    }
  }

  grid.valid = true;
  return grid;
}

}  // namespace

class SimpleNavPlannerNode : public rclcpp::Node
{
public:
  SimpleNavPlannerNode()
  : Node("simple_nav_planner")
  {
    params_ = load_and_validate_params(*this);
    planner_ = create_planner(params_.planner, params_);
    is_uav_ = (params_.mode == "uav");
    is_local_role_ = (params_.pipeline_role == "local");
    liveness_logger_ = rclcpp::get_logger(
      params_.robot_name.empty() ? "nav_liveness" : params_.robot_name + ".nav_liveness");

    // Role-based input/output topic selection. The planner code itself is
    // role-agnostic — only the I/O wiring differs between global and local
    // instances.
    const std::string input_map_topic =
      is_local_role_ ? params_.local_planning_map_topic : params_.planning_map_topic;
    input_map_topic_ = input_map_topic;
    const std::string output_path_topic =
      is_local_role_ ? params_.local_path_topic : params_.global_path_topic;
    if (is_local_role_ && input_map_topic.empty()) {
      throw std::runtime_error(
        "planner: pipeline.role=local requires topics.local_planning_map to be set");
    }

    // For UAV mode, get a typed pointer to the UAV planner.
    if (is_uav_) {
      uav_planner_ = dynamic_cast<UavPlanner *>(planner_.get());
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = *msg;
        has_map_ = true;
      });

    // When running as the local planner, also listen to the global planner's
    // output. We use it both to pick the local target (the global path's exit
    // point from the rolling window) and to mask the local A* into a corridor
    // around global's chosen homotopy. If no global path is published the
    // local planner falls back to the user goal and a free A*.
    if (is_local_role_) {
      global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        params_.global_path_topic, 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
          latest_global_path_ = *msg;
          has_global_path_ = !msg->poses.empty();
        });
    }

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      params_.active_goal_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (has_goal_) {
          const auto & p_old = latest_goal_.pose.position;
          const auto & p_new = msg->pose.position;
          const double d_goal = std::hypot(p_new.x - p_old.x, p_new.y - p_old.y);
          const double goal_tol = is_uav_ ? params_.uav_goal_xyz_tol_m : params_.ugv_goal_xy_tol_m;
          const double goal_change_reset_m = std::max(0.20, 2.0 * goal_tol);
          if (d_goal > goal_change_reset_m) {
            has_prev_path_ = false;
            prev_side_ = 0;
            prev_cost_m_ = std::numeric_limits<double>::infinity();
            prev_path_size_ = 0;
            last_flip_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            prev_path_.poses.clear();
            prev_full_path_.poses.clear();
            // Drop the global path too. It was planned to the OLD goal, and
            // the global planner publishes nothing when it fails to plan — so
            // without this the local planner would keep masking its A* into a
            // corridor around a route to a goal we have already left. That
            // silently biases every local plan toward the wrong homotopy for
            // as long as global keeps failing.
            has_global_path_ = false;
            latest_global_path_.poses.clear();
            // A new goal must be planned on the very next tick, not whenever
            // the replan period happens to expire.
            force_replan_ = true;
            RCLCPP_INFO(
              get_logger(),
              "Goal changed (delta=%.2f m), resetting plan acceptance cache",
              d_goal);
          }
        }
        latest_goal_ = *msg;
        has_goal_ = true;
        ticks_since_goal_msg_ = 0;
      });

    timer_ = rclcpp::create_timer(this, get_clock(), 100ms, [this]() { on_tick(); });

    // --- UAV: set up scovox GetRegion service client ---
    if (is_uav_) {
      // Declare the service name parameter.
      const std::string scovox_service = declare_parameter<std::string>(
        "scovox_get_region_service", "/scovox_node/get_region");

      srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      get_region_client_ = create_client<scovox_msgs::srv::GetRegion>(
        scovox_service, rmw_qos_profile_services_default, srv_cb_group_);

      // Timer to periodically fetch 3D voxels (1 Hz).
      voxel_fetch_timer_ = rclcpp::create_timer(this, get_clock(), 1000ms, [this]() { fetch_voxel_grid(); });

      RCLCPP_INFO(get_logger(), "UAV 3D planner: scovox service=%s", scovox_service.c_str());
    }

    // On liveness_logger_, NOT get_logger(). The launch file sets this node's
    // own logger to warn, so on get_logger() this banner is invisible in every
    // campaign log — and it is the only line that says which map topic the
    // planner is watching. That is precisely how a global planner pointed at a
    // topic nobody publishes went unnoticed for the whole campaign history.
    // liveness_logger_ is a separate logger name and is deliberately not
    // covered by the per-node selectors, so this line and the periodic
    // "global plan ok" heartbeat always survive together: one says what was
    // wired, the other proves it produced work.
    RCLCPP_INFO(
      liveness_logger_,
      "planner node started: role=%s active_goal=%s in_map=%s out_path=%s "
      "planner=%s corridor=%.1fm — NO-OP until '%s' has a publisher",
      params_.pipeline_role.c_str(),
      params_.active_goal_topic.c_str(),
      input_map_topic.c_str(),
      output_path_topic.c_str(),
      planner_->name().c_str(),
      is_local_role_ ? params_.ugv_local_corridor_radius_m : 0.0,
      input_map_topic.c_str());
  }

private:
  void fetch_voxel_grid()
  {
    if (!uav_planner_ || !has_odom_ || !has_goal_) {
      return;
    }

    if (!get_region_client_->service_is_ready()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for scovox GetRegion service...");
      return;
    }

    // If a request is already in flight, skip.
    if (voxel_request_pending_) {
      return;
    }

    // Build AABB encompassing robot position and goal, with margin.
    const auto & rp = latest_odom_.pose.pose.position;
    const auto & gp = latest_goal_.pose.position;
    constexpr double kMargin = 5.0;  // meters
    constexpr double kZMargin = 3.0;

    auto request = std::make_shared<scovox_msgs::srv::GetRegion::Request>();
    request->min_corner.x = std::min(rp.x, gp.x) - kMargin;
    request->min_corner.y = std::min(rp.y, gp.y) - kMargin;
    request->min_corner.z = std::min(rp.z, gp.z) - kZMargin;
    request->max_corner.x = std::max(rp.x, gp.x) + kMargin;
    request->max_corner.y = std::max(rp.y, gp.y) + kMargin;
    request->max_corner.z = std::max(rp.z, gp.z) + kZMargin;

    voxel_request_pending_ = true;

    get_region_client_->async_send_request(request,
      [this](rclcpp::Client<scovox_msgs::srv::GetRegion>::SharedFuture future) {
        voxel_request_pending_ = false;
        try {
          const auto response = future.get();
          constexpr double kGridRes = 0.3;      // planning grid resolution
          constexpr double kOccThreshold = 0.6;  // occupancy probability threshold
          // Inflate by half body diagonal + safety margin.
          const double body_radius = 0.5 * std::hypot(
            params_.robot_body_length_m, params_.robot_body_width_m);
          const double inflation_m = body_radius + 0.2;  // 0.2m extra safety
          VoxelGrid3D grid = build_voxel_grid(
            response->map, kGridRes, kOccThreshold, inflation_m);
          if (grid.valid && uav_planner_) {
            uav_planner_->update_voxel_grid(grid);
            RCLCPP_DEBUG(get_logger(),
              "3D voxel grid updated: %dx%dx%d (%.1fm res, %zu voxels from scovox)",
              grid.size_x, grid.size_y, grid.size_z, grid.resolution,
              response->map.voxels.size());
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "GetRegion service failed: %s", e.what());
        }
      });
  }

  /// `anomalous` picks the severity, and the distinction matters more than it
  /// looks: the campaign log level for this node is WARN, so an INFO line here
  /// is not written to the per-cell nav log at all. Clearing on "goal reached"
  /// is routine and stays INFO. Clearing because the navigator stopped sending
  /// goals is the robot silently coasting to a halt with an empty path, and at
  /// INFO that produced a nav log with no entry for it whatsoever — the run
  /// looked idle rather than broken.
  void clear_plan_state(const char * reason, bool anomalous = false)
  {
    has_goal_ = false;
    has_prev_path_ = false;
    prev_side_ = 0;
    prev_cost_m_ = 0.0;
    prev_path_size_ = 0;
    last_flip_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    prev_path_.poses.clear();
    prev_full_path_.poses.clear();

    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = latest_odom_.header.frame_id.empty() ? params_.odom_frame :
      latest_odom_.header.frame_id;
    path_pub_->publish(empty);
    if (anomalous) {
      RCLCPP_WARN(get_logger(), "planner cleared active goal: %s", reason);
    } else {
      RCLCPP_INFO(get_logger(), "planner cleared active goal: %s", reason);
    }
  }

  /// End a starvation episode: report it if it ever got past the grace period,
  /// then reset. Idempotent, so it is safe to call on every non-starving tick.
  void clear_starvation()
  {
    if (ticks_starved_ > kStarveGraceTicks) {
      // Names no specific input: starvation now covers a missing odom as well
      // as a missing map, and a recovery line that always says "map" would be
      // wrong for half the episodes it reports.
      RCLCPP_INFO(liveness_logger_, "planner recovered: inputs arrived after %.0f s "
        "(map topic '%s')", ticks_starved_ * 0.1, input_map_topic_.c_str());
    }
    ticks_starved_ = 0;
  }

  void on_tick()
  {
    constexpr double kReplanHorizonM = 10.0;

    // For UAV mode, don't require the 2D planning map — use the 3D voxel grid instead.
    const bool map_ready = is_uav_ ? true : has_map_;

    if (!has_goal_ || !has_odom_ || !map_ready) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
        "Planner waiting: goal=%d odom=%d map=%d",
        has_goal_, has_odom_, map_ready);

      // Starvation is the one waiting-state that is never normal. A planner
      // with a goal and a pose but no map has been asked to work and cannot,
      // and if its map topic has no publisher it will wait forever — silently,
      // because the line above is DEBUG and campaign logs run at WARN. That is
      // exactly how this node stayed inert across every run of the project
      // while still printing a healthy startup banner.
      //
      // Grace period first: dscovox publishes nothing until its first fused
      // frame and only once a subscriber exists, so a few seconds of no map at
      // startup is expected and must not train anyone to ignore this.
      // The condition is "a goal is pending and something needed to serve it is
      // missing", not specifically the map. The map was the input that failed
      // historically, but scoping the warning to it left the symmetric case —
      // goal and map present, no ODOM — in exactly the silence this check
      // exists to end: an odom remap regression produces a nav log with two
      // startup banners and nothing after, while the exploration planner times
      // every goal out and the run reads as difficult terrain.
      //
      // `has_goal_` still gates it. A planner with no goal is idle by design
      // between explore steps, and warning on that would fire on every healthy
      // run until nobody read the line.
      if (has_goal_ && (!has_odom_ || !map_ready)) {
        ++ticks_starved_;
        if (ticks_starved_ > kStarveGraceTicks) {
          RCLCPP_WARN_THROTTLE(liveness_logger_, *get_clock(), 10000,
            "planner starving: goal pending but odom=%d map=%d after %.0f s "
            "— map topic '%s'; check the missing input has a publisher; this "
            "planner is a no-op until it arrives",
            static_cast<int>(has_odom_), static_cast<int>(map_ready),
            ticks_starved_ * 0.1, input_map_topic_.c_str());
        }
      } else {
        // No goal pending: idle by design, not starving. Clear the state here
        // rather than leaving it to the full-readiness path below, which is
        // only reached once all three inputs are present. Without this, a
        // starvation that ends because the GOAL was withdrawn keeps its
        // counter, and the next fully-ready tick prints a recovery line
        // inflated by however long the planner sat idle — attributing a goal
        // gap to an input recovery. A liveness line that reports the wrong
        // cause is worse than none: it is how these checks stop being read.
        clear_starvation();
      }
      return;
    }
    clear_starvation();

    // Detect navigator stopped publishing (crash, cancel, etc.).
    // Uses tick count instead of sim-time to avoid false triggers from sim-time jumps.
    // At 100ms timer, 100 ticks ≈ 10s sim-time without a goal message.
    constexpr int kGoalStaleTicks = 100;
    ++ticks_since_goal_msg_;
    if (ticks_since_goal_msg_ > kGoalStaleTicks) {
      clear_plan_state("navigator stopped publishing", /*anomalous=*/true);
      return;
    }

    const auto & current = latest_odom_.pose.pose.position;
    const auto & goal = latest_goal_.pose.position;
    const double goal_dist_m = std::hypot(goal.x - current.x, goal.y - current.y);
    if (goal_dist_m <= final_goal_tolerance(params_)) {
      clear_plan_state("goal reached");
      return;
    }

    // Global-role replan decimation.
    //
    // The global instance plans A* over the 150 m fused grid (375x375 cells at
    // 0.40 m). Its input map only changes at 1 Hz — dscovox republishes
    // global_planning_map on a 1.0 s timer — so planning at the 10 Hz tick rate
    // recomputes the same answer nine times out of ten, on the biggest grid in
    // the system, for both robots, inside a cell budget that is already
    // 1.149 x sim-time.
    //
    // The gate is stamped on every ATTEMPT, not on every success. A failing
    // plan publishes nothing and leaves has_prev_path_ false, so a
    // success-stamped gate would degenerate to full tick rate in exactly the
    // case that costs the most: a blocked goal cell makes best_goal_endpoint_cell
    // run its full relaxation scan before giving up.
    //
    // Two things still preempt the period: a goal change (force_replan_), and a
    // latched path that the newest map has just invalidated — deferring either
    // would mean steering along a route we already know is wrong.
    const bool is_global_ugv = !is_uav_ && !is_local_role_;
    if (is_global_ugv && params_.ugv_global_replan_period_sec > 0.0) {
      const double since_attempt =
        has_plan_attempt_ ? (now() - last_plan_attempt_).seconds()
                          : std::numeric_limits<double>::infinity();
      bool skip = !force_replan_ &&
        since_attempt >= 0.0 &&
        since_attempt < params_.ugv_global_replan_period_sec;
      if (skip && has_prev_path_ && !path_still_valid(prev_full_path_, latest_map_)) {
        skip = false;
      }
      if (skip) {
        return;
      }
      force_replan_ = false;
      last_plan_attempt_ = now();
      has_plan_attempt_ = true;
    }

    const MapSnapshot map_snapshot = snapshot_from_occupancy_grid(latest_map_);

    // For role=local, refine inside global's chosen homotopy:
    //   1. Slice the global path to the segment that lies in our local map.
    //   2. Use the slice's exit point as the A* target (instead of the user
    //      goal, which is usually outside the rolling window anyway).
    //   3. Mask the local map down to a corridor of half-width
    //      ugv.local_corridor_radius_m around the slice. The corridor
    //      forces the local A* to stay on global's side of every obstacle.
    //   4. If A* fails inside the corridor (the slice runs through a freshly
    //      observed obstacle), retry without the mask so the local planner
    //      can dodge. The retry will likely pick a different homotopy than
    //      global; the global planner will catch up on its next replan.
    geometry_msgs::msg::PoseStamped goal_for_planner = latest_goal_;
    MapSnapshot map_for_planner = map_snapshot;
    bool corridor_active = false;
    if (is_local_role_ && has_global_path_) {
      std::vector<geometry_msgs::msg::Point> centerline;
      geometry_msgs::msg::Point local_target_pt;
      if (slice_global_path_for_local(
            latest_global_path_, map_snapshot, current, centerline, local_target_pt))
      {
        goal_for_planner.header = latest_goal_.header;
        goal_for_planner.pose.position = local_target_pt;
        goal_for_planner.pose.orientation.w = 1.0;
        if (params_.ugv_local_corridor_radius_m > 0.0) {
          map_for_planner = apply_corridor_mask(
            map_snapshot, centerline, params_.ugv_local_corridor_radius_m, current);
          corridor_active = true;
        }
      }
    }

    PlannerOutput out = planner_->compute_plan(latest_odom_, goal_for_planner, map_for_planner);
    if (corridor_active && !out.has_path) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Local planner: corridor A* failed, retrying with free local map");
      out = planner_->compute_plan(latest_odom_, goal_for_planner, map_snapshot);
    }
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "Planner result: has_path=%d path_size=%zu corridor=%d",
      out.has_path, out.path.poses.size(), corridor_active ? 1 : 0);
    if (out.has_path) {
      const nav_msgs::msg::Path plan_horizon = truncate_path_by_length(out.path, kReplanHorizonM);
      const int side = departure_side_wrt_goal(plan_horizon, current, goal, 0.6);
      const double cost_m = path_length(plan_horizon);

      // Side-flip rejection is a global-planner concern: it prevents the
      // robot from committing to one homotopy and then flipping to a worse
      // alternative. The local planner runs at higher rate over a smaller
      // window where the cost gradient is more reliable, so the cooldown
      // would only lock in stale paths. Skip it entirely for role=local.
      if (has_prev_path_ && !is_uav_ && !is_local_role_) {
        const bool side_flip = prev_side_ != 0 && side != 0 && prev_side_ != side;
        const bool non_improving = cost_m + 1e-3 >= prev_cost_m_;
        const double since_last_flip = (now() - last_flip_time_).seconds();
        const bool in_cooldown = since_last_flip < params_.ugv_side_flip_cooldown_sec;

        if (side_flip && (non_improving || in_cooldown)) {
          if (path_still_valid(prev_full_path_, latest_map_)) {
            RCLCPP_DEBUG(
              get_logger(),
              "Planner side flip rejected: prev=%s new=%s prev_cost=%.2f new_cost=%.2f cooldown=%.1f/%.1fs",
              side_name(prev_side_), side_name(side), prev_cost_m_, cost_m,
              since_last_flip, params_.ugv_side_flip_cooldown_sec);
            path_pub_->publish(prev_full_path_);
            return;
          }
          RCLCPP_WARN(get_logger(),
            "Side flip reject overridden: prev path now goes through occupied cells");
        }

        if (side_flip) {
          last_flip_time_ = now();
          RCLCPP_INFO(
            get_logger(),
            "Planner side flip accepted: prev=%s new=%s prev_cost=%.2f new_cost=%.2f",
            side_name(prev_side_), side_name(side), prev_cost_m_, cost_m);
        }
      }

      prev_side_ = side;
      prev_cost_m_ = cost_m;
      prev_path_size_ = plan_horizon.poses.size();
      prev_path_ = plan_horizon;
      // Stamp the goal orientation onto the last waypoint so the controller
      // can rotate in place to face the desired direction after reaching it.
      // Only stamp if the last waypoint is near the actual goal — the local
      // planner's path ends at an intermediate target, not the final goal.
      if (!out.path.poses.empty()) {
        const auto & wp = out.path.poses.back().pose.position;
        double dg = std::hypot(wp.x - goal.x, wp.y - goal.y);
        if (dg < final_goal_tolerance(params_) * 2.0) {
          out.path.poses.back().pose.orientation = latest_goal_.pose.orientation;
        }
      }
      prev_full_path_ = out.path;
      has_prev_path_ = true;
      path_pub_->publish(out.path);

      // Positive heartbeat for the global instance. The whole failure this
      // release fixes was a planner that looked healthy because it only ever
      // printed a startup banner: absence of output was indistinguishable from
      // absence of work. A periodic line that can ONLY be printed after a real
      // plan makes "global planner is alive" checkable from the campaign logs
      // instead of inferable from silence.
      if (is_global_ugv) {
        RCLCPP_INFO_THROTTLE(liveness_logger_, *get_clock(), 30000,
          "global plan ok: wps=%zu len=%.1fm goal=(%.1f,%.1f) map=%ux%u@%.2fm",
          out.path.poses.size(), path_length(out.path), goal.x, goal.y,
          latest_map_.info.width, latest_map_.info.height,
          latest_map_.info.resolution);
      }
    } else if (has_prev_path_ && (is_uav_ || path_still_valid(prev_full_path_, latest_map_))) {
      // Re-stamp orientation on reused path if last waypoint is near goal.
      if (!prev_full_path_.poses.empty()) {
        const auto & wp = prev_full_path_.poses.back().pose.position;
        double dg = std::hypot(wp.x - goal.x, wp.y - goal.y);
        if (dg < final_goal_tolerance(params_) * 2.0) {
          prev_full_path_.poses.back().pose.orientation = latest_goal_.pose.orientation;
        }
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Replan failed, reusing previous path (goal_dist=%.2f)",
        std::hypot(goal.x - current.x, goal.y - current.y));
      path_pub_->publish(prev_full_path_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Replan failed, no previous path to reuse (goal_dist=%.2f)",
        std::hypot(goal.x - current.x, goal.y - current.y));
    }
  }

  NodeParameters params_;
  std::unique_ptr<PlannerBase> planner_;
  bool is_uav_{false};
  bool is_local_role_{false};
  UavPlanner * uav_planner_{nullptr};  // non-owning, only valid when is_uav_

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  // Only set when role==local: the global planner's latest published path,
  // used to derive a local target + corridor mask in on_tick.
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // UAV 3D voxel fetching.
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::Client<scovox_msgs::srv::GetRegion>::SharedPtr get_region_client_;
  rclcpp::TimerBase::SharedPtr voxel_fetch_timer_;
  bool voxel_request_pending_{false};

  bool has_goal_{false};
  bool has_odom_{false};
  bool has_map_{false};
  bool has_global_path_{false};
  nav_msgs::msg::Path latest_global_path_;
  int ticks_since_goal_msg_{0};

  // Starvation watch: consecutive 100 ms ticks spent with a goal and a pose
  // but no map. 200 ticks = 20 s of grace before the first warning.
  std::string input_map_topic_;
  int ticks_starved_{0};
  static constexpr int kStarveGraceTicks = 200;

  // Liveness lines go to a SEPARATE logger, and that is not cosmetic.
  //
  // The launch file runs this node at `<robot>.simple_nav_global_planner:=warn`
  // — deliberately, because the planner's INFO chatter is per-replan. But the
  // positive half of the "did the global planner ever plan?" gate is an INFO
  // heartbeat, and a heartbeat that the log level eats is a check that has
  // stopped checking: it reads as PASS whether or not the planner works. That
  // is the same class of defect as the topic bug it exists to catch.
  //
  // Escalating it to WARN would work but poisons the WARN stream that the
  // campaign failure counts are read from. `<robot>.nav_liveness` sits outside
  // the `<robot>.simple_nav_global_planner` subtree the selector names, so it
  // keeps the default level and the line survives at its honest severity.
  rclcpp::Logger liveness_logger_{rclcpp::get_logger("nav_liveness")};

  // Replan decimation. Timestamped on every ATTEMPT, not on every success:
  // a planner that is failing must be rate-limited too, and failure publishes
  // nothing, so gating on a stored path would leave the failing case running
  // at the full 10 Hz tick rate — the one case where each attempt is most
  // expensive, because a blocked goal cell triggers the full endpoint
  // relaxation scan.
  rclcpp::Time last_plan_attempt_{0, 0, RCL_ROS_TIME};
  bool has_plan_attempt_{false};
  bool force_replan_{false};
  bool has_prev_path_{false};
  int prev_side_{0};
  double prev_cost_m_{0.0};
  size_t prev_path_size_{0};
  rclcpp::Time last_flip_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path prev_path_;
  nav_msgs::msg::Path prev_full_path_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  nav_msgs::msg::Odometry latest_odom_;
  nav_msgs::msg::OccupancyGrid latest_map_;
};

}  // namespace simple_nav_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simple_nav_3d::SimpleNavPlannerNode>();
  // Use MultiThreadedExecutor so the service callback group can run
  // concurrently with the main timer callbacks.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
