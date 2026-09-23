// Moved comments: doc/occupancy_grid_utils_notes.md
#ifndef SIMPLE_NAV_3D__MAPPING__OCCUPANCY_GRID_UTILS_HPP_
#define SIMPLE_NAV_3D__MAPPING__OCCUPANCY_GRID_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "simple_nav_3d/mapping/map_snapshot.hpp"

namespace simple_nav_3d
{

inline MapSnapshot snapshot_from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid)
{
  MapSnapshot out;
  out.local_map = grid;
  out.resolution = static_cast<double>(grid.info.resolution);
  out.width = static_cast<int>(grid.info.width);
  out.height = static_cast<int>(grid.info.height);
  out.origin_x = grid.info.origin.position.x;
  out.origin_y = grid.info.origin.position.y;

  if (out.width <= 0 || out.height <= 0 || out.resolution <= 0.0) {
    return out;
  }

  const size_t cell_count = static_cast<size_t>(out.width * out.height);
  out.occupied.assign(cell_count, 0);
  const size_t n = std::min(cell_count, grid.data.size());
  for (size_t i = 0; i < n; ++i) {
    out.occupied[i] = (grid.data[i] >= 50) ? 1U : 0U;
  }
  // The wire grid is already inflated with no raw counterpart, so occupied_raw
  // is a copy of occupied: consumers get a conservative lower bound on obstacle
  // distance (the inflation halo reads as solid).
  // (notes: grid-occupied-raw-is-inflated)
  out.occupied_raw = out.occupied;

  out.valid = true;
  return out;
}

// True iff the straight segment (x0,y0)->(x1,y1) crosses no occupied cell.
//
// Exact Amanatides-Woo traversal, not sampling; sole caller is the planner
// node's goal-snap. A cell-corner tie refuses if either shoulder is occupied.
// Symmetry comes from the canonical endpoint order; the tie takes no epsilon.
// (notes: segment-free-exact-traversal)
//
// Out-of-bounds cells refuse, unlike path_still_valid(). In-bounds unknown
// cells read as free, as everywhere in the UGV planner; changing that is a
// planner-wide change. Callers pass the inflated occupied grid.
// (notes: segment-free-bounds-and-unknown)
inline bool segment_free(
  const MapSnapshot & m, double x0, double y0, double x1, double y1)
{
  if (!m.valid || m.resolution <= 0.0 || m.width <= 0 || m.height <= 0) return false;
  const size_t cells =
    static_cast<size_t>(m.width) * static_cast<size_t>(m.height);
  if (m.occupied.size() < cells) return false;

  // Non-finite endpoints refuse explicitly: cell_of() on NaN is undefined
  // behaviour. This also guarantees the tie branch below never sees a
  // non-finite crossing parameter. (notes: segment-free-non-finite)
  if (!std::isfinite(x0) || !std::isfinite(y0) ||
      !std::isfinite(x1) || !std::isfinite(y1))
  {
    return false;
  }

  // Canonical endpoint order: the walk's float comparisons depend on which
  // endpoint is x0, so ordering first makes the result independent of
  // direction. Free, since both endpoints are blocked-checked anyway.
  // (notes: segment-free-endpoint-order)
  if (x1 < x0 || (x1 == x0 && y1 < y0)) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  auto cell_of = [&m](double v, double origin) {
      return static_cast<int>(std::floor((v - origin) / m.resolution));
    };
  auto blocked = [&m](int cx, int cy) {
      if (cx < 0 || cy < 0 || cx >= m.width || cy >= m.height) return true;
      return m.occupied[static_cast<size_t>(cy) * static_cast<size_t>(m.width) +
                        static_cast<size_t>(cx)] != 0U;
    };

  int gx = cell_of(x0, m.origin_x);
  int gy = cell_of(y0, m.origin_y);
  const int gx_end = cell_of(x1, m.origin_x);
  const int gy_end = cell_of(y1, m.origin_y);
  if (blocked(gx, gy)) return false;
  if (blocked(gx_end, gy_end)) return false;

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const int step_x = (dx > 0.0) ? 1 : ((dx < 0.0) ? -1 : 0);
  const int step_y = (dy > 0.0) ? 1 : ((dy < 0.0) ? -1 : 0);

  // Segment parameter t in [0,1] at which the walk leaves the current cell
  // across its next x / y boundary, and how much t one whole cell costs.
  const double kInf = std::numeric_limits<double>::infinity();
  double t_next_x = kInf;
  double t_step_x = kInf;
  if (step_x != 0) {
    const double boundary =
      m.origin_x + (gx + (step_x > 0 ? 1 : 0)) * m.resolution;
    t_next_x = (boundary - x0) / dx;
    t_step_x = m.resolution / std::abs(dx);
  }
  double t_next_y = kInf;
  double t_step_y = kInf;
  if (step_y != 0) {
    const double boundary =
      m.origin_y + (gy + (step_y > 0 ? 1 : 0)) * m.resolution;
    t_next_y = (boundary - y0) / dy;
    t_step_y = m.resolution / std::abs(dy);
  }

  // The step cap is an upper bound, not an exact count (a corner tie advances
  // both axes at once). Exceeding it means the walk failed to converge, and it
  // refuses rather than loops. (notes: segment-free-step-cap)
  const long max_steps =
    static_cast<long>(std::abs(gx_end - gx)) +
    static_cast<long>(std::abs(gy_end - gy)) + 2;
  for (long i = 0; i < max_steps; ++i) {
    if (gx == gx_end && gy == gy_end) return true;
    if (t_next_x < t_next_y) {
      gx += step_x;
      t_next_x += t_step_x;
    } else if (t_next_y < t_next_x) {
      gy += step_y;
      t_next_y += t_step_y;
    } else {
      // Exact tie at a cell corner: refuse if either shoulder is occupied, then
      // step both axes. Finite endpoints and the zero-length early return make
      // a non-finite parameter here impossible. (notes: segment-free-exact-tie)
      if (blocked(gx + step_x, gy)) return false;
      if (blocked(gx, gy + step_y)) return false;
      gx += step_x;
      gy += step_y;
      t_next_x += t_step_x;
      t_next_y += t_step_y;
    }
    if (blocked(gx, gy)) return false;
  }
  return false;
}

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__MAPPING__OCCUPANCY_GRID_UTILS_HPP_
