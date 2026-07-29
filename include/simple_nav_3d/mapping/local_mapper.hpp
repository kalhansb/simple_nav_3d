#ifndef SIMPLE_NAV_3D__MAPPING__LOCAL_MAPPER_HPP_
#define SIMPLE_NAV_3D__MAPPING__LOCAL_MAPPER_HPP_

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "simple_nav_3d/mapping/map_snapshot.hpp"
#include "simple_nav_3d/parameters.hpp"

namespace simple_nav_3d
{

class LocalMapper
{
public:
  explicit LocalMapper(const NodeParameters & params);

  MapSnapshot build_map(
    const nav_msgs::msg::Odometry & odom,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & points_msg);

private:
  static int64_t world_key(int gx, int gy);

  NodeParameters params_;
  std::unordered_map<int64_t, double> world_obstacle_last_seen_sec_;
  std::unordered_map<int64_t, double> world_free_last_seen_sec_;
  int prune_counter_{0};
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__MAPPING__LOCAL_MAPPER_HPP_
