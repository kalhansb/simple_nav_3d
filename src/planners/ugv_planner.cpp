// Moved comments: doc/simple_nav_3d_code_notes.md
#include "simple_nav_3d/planners/ugv_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <optional>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "simple_nav_3d/grid_utils.hpp"

using simple_nav_3d::GridIndex;
using simple_nav_3d::flatten;
using simple_nav_3d::in_bounds;

namespace
{

GridIndex world_to_grid(
  double wx,
  double wy,
  double origin_x,
  double origin_y,
  double resolution);

double heuristic(const GridIndex & a, const GridIndex & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::hypot(dx, dy);
}

bool line_of_sight_free(
  const std::vector<uint8_t> & occupied,
  int width,
  int height,
  const GridIndex & a,
  const GridIndex & b)
{
  const int dx = b.x - a.x;
  const int dy = b.y - a.y;
  const int steps = std::max(std::abs(dx), std::abs(dy));
  if (steps == 0) {
    return in_bounds(a, width, height) && occupied[flatten(a, width)] == 0;
  }

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    GridIndex c{
      static_cast<int>(std::round(static_cast<double>(a.x) + t * static_cast<double>(dx))),
      static_cast<int>(std::round(static_cast<double>(a.y) + t * static_cast<double>(dy)))};
    if (!in_bounds(c, width, height)) {
      return false;
    }
    if (occupied[flatten(c, width)] != 0) {
      return false;
    }
  }

  return true;
}

std::vector<GridIndex> los_prune_path(
  const std::vector<GridIndex> & in,
  const std::vector<uint8_t> & occupied,
  int width,
  int height)
{
  if (in.size() < 3) {
    return in;
  }

  std::vector<GridIndex> out;
  out.push_back(in.front());

  size_t anchor = 0;
  while (anchor < in.size() - 1) {
    size_t best = anchor + 1;
    for (size_t j = anchor + 1; j < in.size(); ++j) {
      if (!line_of_sight_free(occupied, width, height, in[anchor], in[j])) {
        break;
      }
      best = j;
    }
    out.push_back(in[best]);
    anchor = best;
  }

  return out;
}

bool world_point_free(
  const std::vector<uint8_t> & occupied,
  int width,
  int height,
  double origin_x,
  double origin_y,
  double resolution,
  double x,
  double y)
{
  GridIndex c = world_to_grid(x, y, origin_x, origin_y, resolution);
  if (!in_bounds(c, width, height)) {
    return false;
  }
  return occupied[flatten(c, width)] == 0;
}

bool segment_world_free(
  const std::vector<uint8_t> & occupied,
  int width,
  int height,
  double origin_x,
  double origin_y,
  double resolution,
  const geometry_msgs::msg::PoseStamped & p0,
  const geometry_msgs::msg::PoseStamped & p1)
{
  const double dx = p1.pose.position.x - p0.pose.position.x;
  const double dy = p1.pose.position.y - p0.pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double step = std::max(0.05, 0.5 * resolution);
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / step)));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double x = p0.pose.position.x + t * dx;
    const double y = p0.pose.position.y + t * dy;
    if (!world_point_free(occupied, width, height, origin_x, origin_y, resolution, x, y)) {
      return false;
    }
  }
  return true;
}

bool path_world_free(
  const nav_msgs::msg::Path & path,
  const std::vector<uint8_t> & occupied,
  int width,
  int height,
  double origin_x,
  double origin_y,
  double resolution)
{
  if (path.poses.empty()) {
    return false;
  }
  for (size_t i = 0; i < path.poses.size(); ++i) {
    if (!world_point_free(
        occupied,
        width,
        height,
        origin_x,
        origin_y,
        resolution,
        path.poses[i].pose.position.x,
        path.poses[i].pose.position.y))
    {
      return false;
    }
    if (i > 0 && !segment_world_free(
        occupied,
        width,
        height,
        origin_x,
        origin_y,
        resolution,
        path.poses[i - 1],
        path.poses[i]))
    {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path chaikin_once(const nav_msgs::msg::Path & input)
{
  nav_msgs::msg::Path out;
  out.header = input.header;

  if (input.poses.size() < 3) {
    return input;
  }

  out.poses.push_back(input.poses.front());
  for (size_t i = 0; i + 1 < input.poses.size(); ++i) {
    const auto & a = input.poses[i].pose.position;
    const auto & b = input.poses[i + 1].pose.position;

    geometry_msgs::msg::PoseStamped q;
    q.header = input.header;
    q.pose.position.x = 0.75 * a.x + 0.25 * b.x;
    q.pose.position.y = 0.75 * a.y + 0.25 * b.y;
    q.pose.position.z = 0.0;
    q.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped r;
    r.header = input.header;
    r.pose.position.x = 0.25 * a.x + 0.75 * b.x;
    r.pose.position.y = 0.25 * a.y + 0.75 * b.y;
    r.pose.position.z = 0.0;
    r.pose.orientation.w = 1.0;

    out.poses.push_back(q);
    out.poses.push_back(r);
  }
  out.poses.push_back(input.poses.back());
  return out;
}

nav_msgs::msg::Path resample_path_uniform(const nav_msgs::msg::Path & input, double step_m)
{
  nav_msgs::msg::Path out;
  out.header = input.header;
  if (input.poses.size() < 2 || step_m <= 1e-6) {
    return input;
  }

  out.poses.push_back(input.poses.front());
  for (size_t i = 1; i < input.poses.size(); ++i) {
    const auto & a = input.poses[i - 1].pose.position;
    const auto & b = input.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
      continue;
    }

    const int n = std::max(1, static_cast<int>(std::floor(dist / step_m)));
    for (int k = 1; k <= n; ++k) {
      const double t = std::min(1.0, (static_cast<double>(k) * step_m) / dist);
      geometry_msgs::msg::PoseStamped p;
      p.header = input.header;
      p.pose.position.x = a.x + t * dx;
      p.pose.position.y = a.y + t * dy;
      p.pose.position.z = 0.0;
      p.pose.orientation.w = 1.0;
      out.poses.push_back(p);
    }
  }

  if (out.poses.empty() ||
    std::hypot(
      out.poses.back().pose.position.x - input.poses.back().pose.position.x,
      out.poses.back().pose.position.y - input.poses.back().pose.position.y) > 1e-3)
  {
    out.poses.push_back(input.poses.back());
  }

  return out;
}

GridIndex world_to_grid(
  double wx,
  double wy,
  double origin_x,
  double origin_y,
  double resolution)
{
  return GridIndex{
    static_cast<int>(std::floor((wx - origin_x) / resolution)),
    static_cast<int>(std::floor((wy - origin_y) / resolution))};
}

bool clear_toward_goal(
  const std::vector<uint8_t> & occupied,
  int width,
  int height,
  const GridIndex & from,
  const GridIndex & goal_raw)
{
  const int dx = goal_raw.x - from.x;
  const int dy = goal_raw.y - from.y;
  const int steps = std::max(std::abs(dx), std::abs(dy));
  if (steps <= 1) {
    return true;
  }

  for (int i = 1; i < steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    GridIndex c{
      static_cast<int>(std::round(static_cast<double>(from.x) + t * static_cast<double>(dx))),
      static_cast<int>(std::round(static_cast<double>(from.y) + t * static_cast<double>(dy)))};
    if (!in_bounds(c, width, height)) {
      return false;
    }
    if (occupied[flatten(c, width)] != 0) {
      return false;
    }
  }

  return true;
}

std::optional<GridIndex> best_goal_endpoint_cell(
  const std::vector<uint8_t> & occupied,
  const std::vector<uint8_t> & reachable,
  int width,
  int height,
  const GridIndex & start,
  const GridIndex & goal_raw)
{
  if (!in_bounds(goal_raw, width, height)) {
    return std::nullopt;
  }

  const double start_to_goal_cells = heuristic(start, goal_raw);
  const double min_progress_cells =
    (start_to_goal_cells > 6.0) ? std::max(3.0, 0.2 * start_to_goal_cells) : 0.0;

  // Fast path: the goal cell is free and reachable. It is exactly what the scan
  // below would return (it scores 0 and passes both filters), and it skips the
  // full width*height sweep. (notes: ugv-goal-endpoint-fast-path)
  if (occupied[flatten(goal_raw, width)] == 0 && reachable[flatten(goal_raw, width)] != 0) {
    return goal_raw;
  }

  // Prefer the closest free cell to requested goal that has a clear corridor toward it.
  std::optional<GridIndex> best;
  double best_dist = std::numeric_limits<double>::infinity();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      GridIndex c{x, y};
      if (occupied[flatten(c, width)] != 0) {
        continue;
      }
      if (reachable[flatten(c, width)] == 0) {
        continue;
      }
      if (heuristic(start, c) < min_progress_cells) {
        continue;
      }
      if (!clear_toward_goal(occupied, width, height, c, goal_raw)) {
        continue;
      }
      const double d = heuristic(c, goal_raw);
      if (d < best_dist) {
        best_dist = d;
        best = c;
      }
    }
  }
  if (best.has_value()) {
    return best;
  }

  // Fallback: closest reachable free cell to goal (still on robot side).
  best_dist = std::numeric_limits<double>::infinity();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      GridIndex c{x, y};
      if (occupied[flatten(c, width)] != 0) {
        continue;
      }
      if (reachable[flatten(c, width)] == 0) {
        continue;
      }
      if (heuristic(start, c) < min_progress_cells) {
        continue;
      }
      const double d = heuristic(c, goal_raw);
      if (d < best_dist) {
        best_dist = d;
        best = c;
      }
    }
  }

  if (best.has_value()) {
    return best;
  }

  // Last resort: still return a reachable free cell, even if progress is small.
  best_dist = std::numeric_limits<double>::infinity();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      GridIndex c{x, y};
      if (occupied[flatten(c, width)] != 0) {
        continue;
      }
      if (reachable[flatten(c, width)] == 0) {
        continue;
      }
      const double d = heuristic(c, goal_raw);
      if (d < best_dist) {
        best_dist = d;
        best = c;
      }
    }
  }

  return best;
}

// Frees a minimum-length corridor from a start boxed in by inflated cells to
// the nearest originally-free cell: BFS through occupied cells, then only the
// parent chain is freed. Bounded by max_carve_cells.
// (notes: ugv-carve-escape-corridor)
void carve_escape_corridor(
  std::vector<uint8_t> & occupied,
  int width,
  int height,
  const GridIndex & start,
  int max_carve_cells)
{
  if (!in_bounds(start, width, height)) return;

  const std::vector<std::pair<int, int>> neighbors = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  // Fast path: if any neighbor of start is already free, the regular flood
  // fill below will find a path on its own. No carving needed.
  for (const auto & d : neighbors) {
    GridIndex n{start.x + d.first, start.y + d.second};
    if (in_bounds(n, width, height) && occupied[flatten(n, width)] == 0) {
      return;
    }
  }

  const int cell_count = width * height;
  std::vector<int> came_from(static_cast<size_t>(cell_count), -1);
  std::vector<uint8_t> visited(static_cast<size_t>(cell_count), 0);
  const int start_flat = flatten(start, width);
  visited[start_flat] = 1;

  std::queue<std::pair<GridIndex, int>> q;
  q.push({start, 0});

  int breakthrough_flat = -1;
  while (!q.empty() && breakthrough_flat < 0) {
    const auto [cur, dist] = q.front();
    q.pop();
    if (dist >= max_carve_cells) continue;
    for (const auto & d : neighbors) {
      GridIndex n{cur.x + d.first, cur.y + d.second};
      if (!in_bounds(n, width, height)) continue;
      const int nf = flatten(n, width);
      if (visited[nf]) continue;
      visited[nf] = 1;
      came_from[nf] = flatten(cur, width);
      if (occupied[nf] == 0) {
        // First original-free cell we touched — stop and unwind from here.
        breakthrough_flat = nf;
        break;
      }
      q.push({n, dist + 1});
    }
  }

  if (breakthrough_flat < 0) return;  // Genuinely surrounded; nothing to do.

  // Walk back from breakthrough to start, freeing every cell along the way.
  // The breakthrough cell itself was already free, so don't touch it.
  int cur = came_from[breakthrough_flat];
  while (cur != -1 && cur != start_flat) {
    occupied[cur] = 0;
    cur = came_from[cur];
  }
}

std::vector<uint8_t> compute_reachable_free_cells(
  const std::vector<uint8_t> & occupied,
  int width,
  int height,
  const GridIndex & start)
{
  std::vector<uint8_t> reachable(static_cast<size_t>(width * height), 0);
  if (!in_bounds(start, width, height)) {
    return reachable;
  }

  const int start_flat = flatten(start, width);
  if (occupied[start_flat] != 0) {
    return reachable;
  }

  std::queue<GridIndex> q;
  q.push(start);
  reachable[start_flat] = 1;

  const std::vector<std::pair<int, int>> neighbors = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!q.empty()) {
    const GridIndex cur = q.front();
    q.pop();

    for (const auto & d : neighbors) {
      GridIndex n{cur.x + d.first, cur.y + d.second};
      if (!in_bounds(n, width, height)) {
        continue;
      }
      const int nf = flatten(n, width);
      if (reachable[nf] != 0 || occupied[nf] != 0) {
        continue;
      }
      reachable[nf] = 1;
      q.push(n);
    }
  }

  return reachable;
}

}  // namespace

namespace simple_nav_3d
{

UgvPlanner::UgvPlanner(const NodeParameters & params)
: params_(params)
{
}

std::string UgvPlanner::name() const
{
  return "planner_2d";
}

PlannerOutput UgvPlanner::compute_plan(
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::PoseStamped & goal,
  const MapSnapshot & map_snapshot)
{
  PlannerOutput result;

  if (!map_snapshot.valid) {
    return result;
  }

  const int width = map_snapshot.width;
  const int height = map_snapshot.height;
  if (width < 10 || height < 10) {
    return result;
  }

  std::vector<uint8_t> occupied = map_snapshot.occupied;
  const double origin_x = map_snapshot.origin_x;
  const double origin_y = map_snapshot.origin_y;
  const double map_max_x = origin_x + static_cast<double>(width) * map_snapshot.resolution;
  const double map_max_y = origin_y + static_cast<double>(height) * map_snapshot.resolution;

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const GridIndex start_idx = world_to_grid(
    robot_x,
    robot_y,
    origin_x,
    origin_y,
    map_snapshot.resolution);

  const double goal_x_clamped = std::clamp(goal.pose.position.x, origin_x + 1e-3, map_max_x - 1e-3);
  const double goal_y_clamped = std::clamp(goal.pose.position.y, origin_y + 1e-3, map_max_y - 1e-3);
  const GridIndex goal_idx_raw = world_to_grid(
    goal_x_clamped,
    goal_y_clamped,
    origin_x,
    origin_y,
    map_snapshot.resolution);

  if (!in_bounds(start_idx, width, height) || !in_bounds(goal_idx_raw, width, height)) {
    return result;
  }

  // Ensure start cell is free to avoid immediate failure at robot center.
  occupied[flatten(start_idx, width)] = 0;

  // If the start is boxed in by an inflated halo, carve a minimum escape
  // corridor. Budgeted to 2 m so the carve never frees arbitrary chunks of the
  // map. (notes: ugv-escape-carve-budget)
  const int max_carve_cells = std::max(
    1, static_cast<int>(std::ceil(2.0 / map_snapshot.resolution)));
  carve_escape_corridor(occupied, width, height, start_idx, max_carve_cells);

  const std::vector<uint8_t> reachable = compute_reachable_free_cells(
    occupied, width, height, start_idx);

  const std::optional<GridIndex> maybe_goal_idx = best_goal_endpoint_cell(
    occupied, reachable, width, height, start_idx, goal_idx_raw);
  if (!maybe_goal_idx.has_value()) {
    return result;
  }
  const GridIndex goal_idx = *maybe_goal_idx;

  struct OpenNode
  {
    int idx;
    double f;
  };
  struct Cmp
  {
    bool operator()(const OpenNode & a, const OpenNode & b) const { return a.f > b.f; }
  };

  const int cell_count = width * height;
  std::vector<double> g_score(static_cast<size_t>(cell_count), std::numeric_limits<double>::infinity());
  std::vector<int> came_from(static_cast<size_t>(cell_count), -1);
  std::priority_queue<OpenNode, std::vector<OpenNode>, Cmp> open;

  const int start_flat = flatten(start_idx, width);
  const int goal_flat = flatten(goal_idx, width);
  g_score[start_flat] = 0.0;
  open.push(OpenNode{start_flat, heuristic(start_idx, goal_idx)});

  const std::vector<std::pair<int, int>> neighbors = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  bool found = false;
  while (!open.empty()) {
    const OpenNode current = open.top();
    open.pop();

    if (current.idx == goal_flat) {
      found = true;
      break;
    }

    GridIndex cur{current.idx % width, current.idx / width};
    for (const auto & d : neighbors) {
      GridIndex n{cur.x + d.first, cur.y + d.second};
      if (!in_bounds(n, width, height)) {
        continue;
      }
      const int n_flat = flatten(n, width);
      if (occupied[n_flat] != 0) {
        continue;
      }
      const double step = (d.first == 0 || d.second == 0) ? 1.0 : std::sqrt(2.0);
      const double tentative = g_score[current.idx] + step;
      if (tentative < g_score[n_flat]) {
        came_from[n_flat] = current.idx;
        g_score[n_flat] = tentative;
        open.push(OpenNode{n_flat, tentative + heuristic(n, goal_idx)});
      }
    }
  }

  if (!found) {
    return result;
  }

  std::vector<int> chain;
  int cur = goal_flat;
  while (cur != -1) {
    chain.push_back(cur);
    if (cur == start_flat) {
      break;
    }
    cur = came_from[cur];
  }
  if (chain.empty() || chain.back() != start_flat) {
    return result;
  }
  std::reverse(chain.begin(), chain.end());

  std::vector<GridIndex> raw_cells;
  raw_cells.reserve(chain.size());
  for (int idx_flat : chain) {
    raw_cells.push_back(GridIndex{idx_flat % width, idx_flat / width});
  }

  const std::vector<GridIndex> pruned_cells = los_prune_path(raw_cells, occupied, width, height);

  nav_msgs::msg::Path out;
  out.header.frame_id = odom.header.frame_id.empty() ? params_.odom_frame : odom.header.frame_id;
  out.header.stamp = odom.header.stamp;
  for (const GridIndex & gi : pruned_cells) {
    const double wx = origin_x + (static_cast<double>(gi.x) + 0.5) * map_snapshot.resolution;
    const double wy = origin_y + (static_cast<double>(gi.y) + 0.5) * map_snapshot.resolution;

    geometry_msgs::msg::PoseStamped p;
    p.header = out.header;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    out.poses.push_back(p);
  }

  // End at obstacle-aware reachable endpoint (goal cell center), not raw goal through obstacles.
  geometry_msgs::msg::PoseStamped goal_reached;
  goal_reached.header = out.header;
  goal_reached.pose.position.x =
    origin_x + (static_cast<double>(goal_idx.x) + 0.5) * map_snapshot.resolution;
  goal_reached.pose.position.y =
    origin_y + (static_cast<double>(goal_idx.y) + 0.5) * map_snapshot.resolution;
  goal_reached.pose.position.z = 0.0;
  goal_reached.pose.orientation.w = 1.0;
  if (out.poses.empty() ||
    std::hypot(
      out.poses.back().pose.position.x - goal_reached.pose.position.x,
      out.poses.back().pose.position.y - goal_reached.pose.position.y) > 1e-3)
  {
    out.poses.push_back(goal_reached);
  }

  const nav_msgs::msg::Path smoothed = chaikin_once(out);
  if (path_world_free(
      smoothed,
      occupied,
      width,
      height,
      origin_x,
      origin_y,
      map_snapshot.resolution))
  {
    out = smoothed;
  }

  // Keep waypoint spacing consistent for smoother controller tracking and plan handoff.
  out = resample_path_uniform(out, std::max(0.10, params_.ugv_local_plan_resolution_m));

  result.path = out;
  result.has_path = !out.poses.empty();
  return result;
}

}  // namespace simple_nav_3d
