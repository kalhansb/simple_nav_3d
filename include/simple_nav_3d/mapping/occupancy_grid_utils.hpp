#ifndef SIMPLE_NAV_3D__MAPPING__OCCUPANCY_GRID_UTILS_HPP_
#define SIMPLE_NAV_3D__MAPPING__OCCUPANCY_GRID_UTILS_HPP_

#include <cstdint>
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
  // Grid received over topic is already inflated; raw data not available.
  out.occupied_raw = out.occupied;

  out.valid = true;
  return out;
}

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__MAPPING__OCCUPANCY_GRID_UTILS_HPP_
