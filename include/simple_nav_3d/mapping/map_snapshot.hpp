#ifndef SIMPLE_NAV_3D__MAPPING__MAP_SNAPSHOT_HPP_
#define SIMPLE_NAV_3D__MAPPING__MAP_SNAPSHOT_HPP_

#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace simple_nav_3d
{

struct MapSnapshot
{
  bool valid{false};
  double resolution{0.0};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<uint8_t> occupied;
  std::vector<uint8_t> occupied_raw;  // pre-inflation, for true obstacle distance
  nav_msgs::msg::OccupancyGrid local_map;
  sensor_msgs::msg::PointCloud2 obstacle_cloud;  // obstacle-band points in odom frame
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__MAPPING__MAP_SNAPSHOT_HPP_
