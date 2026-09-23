// Moved comments: doc/simple_nav_3d_code_notes.md
#include "simple_nav_3d/planners/uav_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace
{

struct GridIdx3
{
  int x{0}, y{0}, z{0};
};

inline int flatten3(const GridIdx3 & c, int sx, int sy)
{
  return c.z * sy * sx + c.y * sx + c.x;
}

inline double heuristic3(const GridIdx3 & a, const GridIdx3 & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 26-connected neighbor offsets.
static const std::array<std::array<int, 3>, 26> kNeighbors = {{
  // 6 face neighbors
  {{ 1, 0, 0}}, {{-1, 0, 0}}, {{ 0, 1, 0}}, {{ 0,-1, 0}}, {{ 0, 0, 1}}, {{ 0, 0,-1}},
  // 12 edge neighbors
  {{ 1, 1, 0}}, {{ 1,-1, 0}}, {{-1, 1, 0}}, {{-1,-1, 0}},
  {{ 1, 0, 1}}, {{ 1, 0,-1}}, {{-1, 0, 1}}, {{-1, 0,-1}},
  {{ 0, 1, 1}}, {{ 0, 1,-1}}, {{ 0,-1, 1}}, {{ 0,-1,-1}},
  // 8 corner neighbors
  {{ 1, 1, 1}}, {{ 1, 1,-1}}, {{ 1,-1, 1}}, {{ 1,-1,-1}},
  {{-1, 1, 1}}, {{-1, 1,-1}}, {{-1,-1, 1}}, {{-1,-1,-1}},
}};

inline double step_cost(const std::array<int, 3> & d)
{
  const int n = (d[0] != 0 ? 1 : 0) + (d[1] != 0 ? 1 : 0) + (d[2] != 0 ? 1 : 0);
  if (n == 1) return 1.0;
  if (n == 2) return std::sqrt(2.0);
  return std::sqrt(3.0);
}

bool line_of_sight_3d(
  const simple_nav_3d::VoxelGrid3D & grid,
  const GridIdx3 & a,
  const GridIdx3 & b)
{
  const int dx = b.x - a.x;
  const int dy = b.y - a.y;
  const int dz = b.z - a.z;
  const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
  if (steps == 0) {
    return grid.in_bounds(a.x, a.y, a.z) && grid.occupied[grid.flatten(a.x, a.y, a.z)] == 0;
  }

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const int cx = static_cast<int>(std::round(a.x + t * dx));
    const int cy = static_cast<int>(std::round(a.y + t * dy));
    const int cz = static_cast<int>(std::round(a.z + t * dz));
    if (!grid.in_bounds(cx, cy, cz)) {
      return false;
    }
    if (grid.occupied[grid.flatten(cx, cy, cz)] != 0) {
      return false;
    }
  }
  return true;
}

std::vector<GridIdx3> los_prune_3d(
  const std::vector<GridIdx3> & path,
  const simple_nav_3d::VoxelGrid3D & grid)
{
  if (path.size() < 3) {
    return path;
  }

  std::vector<GridIdx3> out;
  out.push_back(path.front());

  size_t anchor = 0;
  while (anchor < path.size() - 1) {
    size_t best = anchor + 1;
    for (size_t j = anchor + 1; j < path.size(); ++j) {
      if (!line_of_sight_3d(grid, path[anchor], path[j])) {
        break;
      }
      best = j;
    }
    out.push_back(path[best]);
    anchor = best;
  }

  return out;
}

GridIdx3 world_to_grid3(
  double wx, double wy, double wz,
  const simple_nav_3d::VoxelGrid3D & grid)
{
  return GridIdx3{
    static_cast<int>(std::floor((wx - grid.origin_x) / grid.resolution)),
    static_cast<int>(std::floor((wy - grid.origin_y) / grid.resolution)),
    static_cast<int>(std::floor((wz - grid.origin_z) / grid.resolution))};
}

}  // namespace

namespace simple_nav_3d
{

UavPlanner::UavPlanner(const NodeParameters & params)
: params_(params)
{
}

std::string UavPlanner::name() const
{
  return "planner_3d";
}

void UavPlanner::update_voxel_grid(const VoxelGrid3D & grid)
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  cached_grid_ = grid;
}

PlannerOutput UavPlanner::compute_plan(
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::PoseStamped & goal,
  const MapSnapshot & /*map_snapshot*/)
{
  PlannerOutput result;

  VoxelGrid3D grid;
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid = cached_grid_;
  }

  if (!grid.valid || grid.size_x < 2 || grid.size_y < 2 || grid.size_z < 2) {
    // Fallback: straight line if no 3D grid available yet.
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.map_frame;
    path.header.stamp = odom.header.stamp;

    geometry_msgs::msg::PoseStamped start;
    start.header = path.header;
    start.pose = odom.pose.pose;
    path.poses.push_back(start);
    path.poses.push_back(goal);

    result.path = path;
    result.has_path = true;
    return result;
  }

  const double rx = odom.pose.pose.position.x;
  const double ry = odom.pose.pose.position.y;
  const double rz = odom.pose.pose.position.z;

  GridIdx3 start_cell = world_to_grid3(rx, ry, rz, grid);

  // If goal Z is near zero (e.g., from RViz 2D Goal Pose), use nominal height.
  double goal_z = goal.pose.position.z;
  if (std::abs(goal_z) < 0.1) {
    goal_z = params_.uav_nominal_height_m;
  }
  GridIdx3 goal_cell = world_to_grid3(
    goal.pose.position.x, goal.pose.position.y, goal_z, grid);

  // Clamp to grid bounds.
  auto clamp_cell = [&](GridIdx3 & c) {
    c.x = std::clamp(c.x, 0, grid.size_x - 1);
    c.y = std::clamp(c.y, 0, grid.size_y - 1);
    c.z = std::clamp(c.z, 0, grid.size_z - 1);
  };
  clamp_cell(start_cell);
  clamp_cell(goal_cell);

  // Ensure start cell is free.
  grid.occupied[grid.flatten(start_cell.x, start_cell.y, start_cell.z)] = 0;

  // If goal is occupied, find nearest free cell.
  if (grid.occupied[grid.flatten(goal_cell.x, goal_cell.y, goal_cell.z)] != 0) {
    double best_dist = std::numeric_limits<double>::infinity();
    GridIdx3 best = goal_cell;
    const int search_r = 5;
    for (int dz = -search_r; dz <= search_r; ++dz) {
      for (int dy = -search_r; dy <= search_r; ++dy) {
        for (int dx = -search_r; dx <= search_r; ++dx) {
          GridIdx3 c{goal_cell.x + dx, goal_cell.y + dy, goal_cell.z + dz};
          if (!grid.in_bounds(c.x, c.y, c.z)) continue;
          if (grid.occupied[grid.flatten(c.x, c.y, c.z)] != 0) continue;
          double d = heuristic3(c, goal_cell);
          if (d < best_dist) {
            best_dist = d;
            best = c;
          }
        }
      }
    }
    goal_cell = best;
  }

  // --- 3D A* ---
  const int total_cells = grid.size_x * grid.size_y * grid.size_z;
  const int start_flat = flatten3(start_cell, grid.size_x, grid.size_y);
  const int goal_flat = flatten3(goal_cell, grid.size_x, grid.size_y);

  if (start_flat == goal_flat) {
    // Already at goal.
    result.has_path = true;
    result.path.header.frame_id = params_.map_frame;
    result.path.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped p;
    p.header = result.path.header;
    p.pose = odom.pose.pose;
    result.path.poses.push_back(p);
    return result;
  }

  struct OpenNode
  {
    int idx;
    double f;
  };
  struct Cmp
  {
    bool operator()(const OpenNode & a, const OpenNode & b) const { return a.f > b.f; }
  };

  std::vector<double> g_score(static_cast<size_t>(total_cells),
    std::numeric_limits<double>::infinity());
  std::vector<int> came_from(static_cast<size_t>(total_cells), -1);
  std::priority_queue<OpenNode, std::vector<OpenNode>, Cmp> open;

  g_score[start_flat] = 0.0;
  open.push(OpenNode{start_flat, heuristic3(start_cell, goal_cell)});

  const int sx = grid.size_x;
  const int sy = grid.size_y;

  // Height penalty biases the path toward uav_nominal_height_m while allowing
  // climbs around obstacles; the preferred height blends start z, nominal, goal
  // z by xy progress, so start and goal altitudes hold.
  // (notes: uav-height-penalty)
  const double nominal_z = params_.uav_nominal_height_m;
  const double start_z = grid.origin_z + (static_cast<double>(start_cell.z) + 0.5) * grid.resolution;
  const double goal_z_val = grid.origin_z + (static_cast<double>(goal_cell.z) + 0.5) * grid.resolution;
  const double total_xy_dist = std::max(1e-3,
    std::hypot(
      static_cast<double>(goal_cell.x - start_cell.x),
      static_cast<double>(goal_cell.y - start_cell.y)));
  constexpr double kHeightPenaltyWeight = 2.0;  // cost per meter of deviation

  bool found = false;
  int expansions = 0;
  constexpr int kMaxExpansions = 500000;

  while (!open.empty() && expansions < kMaxExpansions) {
    const OpenNode current = open.top();
    open.pop();
    ++expansions;

    if (current.idx == goal_flat) {
      found = true;
      break;
    }

    // Skip stale entries.
    if (current.f > g_score[current.idx] + heuristic3(goal_cell, goal_cell) + 1e6) {
      continue;
    }

    // Decode flat index to 3D.
    GridIdx3 cur;
    cur.z = current.idx / (sy * sx);
    const int rem = current.idx % (sy * sx);
    cur.y = rem / sx;
    cur.x = rem % sx;

    for (const auto & d : kNeighbors) {
      GridIdx3 n{cur.x + d[0], cur.y + d[1], cur.z + d[2]};
      if (!grid.in_bounds(n.x, n.y, n.z)) {
        continue;
      }
      const int n_flat = flatten3(n, sx, sy);
      if (grid.occupied[n_flat] != 0) {
        continue;
      }

      // Step cost + penalty for deviating from preferred height.
      // Preferred height blends: near start → start_z, mid-path → nominal_z, near goal → goal_z.
      const double n_world_z = grid.origin_z + (static_cast<double>(n.z) + 0.5) * grid.resolution;
      const double progress = std::clamp(
        std::hypot(
          static_cast<double>(n.x - start_cell.x),
          static_cast<double>(n.y - start_cell.y)) / total_xy_dist,
        0.0, 1.0);
      // Blend: 0→start_z, 0.3→nominal, 0.7→nominal, 1.0→goal_z
      double preferred_z;
      if (progress < 0.3) {
        const double t = progress / 0.3;
        preferred_z = start_z + t * (nominal_z - start_z);
      } else if (progress > 0.7) {
        const double t = (progress - 0.7) / 0.3;
        preferred_z = nominal_z + t * (goal_z_val - nominal_z);
      } else {
        preferred_z = nominal_z;
      }
      const double height_deviation = std::abs(n_world_z - preferred_z);
      const double height_penalty = kHeightPenaltyWeight * height_deviation * grid.resolution;

      const double tentative = g_score[current.idx] + step_cost(d) + height_penalty;
      if (tentative < g_score[n_flat]) {
        came_from[n_flat] = current.idx;
        g_score[n_flat] = tentative;
        open.push(OpenNode{n_flat, tentative + heuristic3(n, goal_cell)});
      }
    }
  }

  if (!found) {
    // Fallback: straight line.
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.map_frame;
    path.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped start_p;
    start_p.header = path.header;
    start_p.pose = odom.pose.pose;
    path.poses.push_back(start_p);
    path.poses.push_back(goal);
    result.path = path;
    result.has_path = true;
    return result;
  }

  // Backtrack path.
  std::vector<GridIdx3> chain;
  int cur = goal_flat;
  while (cur != -1) {
    GridIdx3 c;
    c.z = cur / (sy * sx);
    const int r = cur % (sy * sx);
    c.y = r / sx;
    c.x = r % sx;
    chain.push_back(c);
    if (cur == start_flat) break;
    cur = came_from[cur];
  }
  std::reverse(chain.begin(), chain.end());

  // Line-of-sight pruning in 3D.
  const std::vector<GridIdx3> pruned = los_prune_3d(chain, grid);

  // Convert to world-frame path.
  nav_msgs::msg::Path out;
  out.header.frame_id = params_.map_frame;
  out.header.stamp = odom.header.stamp;

  for (const auto & c : pruned) {
    geometry_msgs::msg::PoseStamped p;
    p.header = out.header;
    p.pose.position.x = grid.origin_x + (static_cast<double>(c.x) + 0.5) * grid.resolution;
    p.pose.position.y = grid.origin_y + (static_cast<double>(c.y) + 0.5) * grid.resolution;
    p.pose.position.z = grid.origin_z + (static_cast<double>(c.z) + 0.5) * grid.resolution;
    p.pose.orientation.w = 1.0;
    out.poses.push_back(p);
  }

  result.path = out;
  result.has_path = !out.poses.empty();
  return result;
}

}  // namespace simple_nav_3d
