#ifndef SIMPLE_NAV_3D__PLANNERS__UAV_PLANNER_HPP_
#define SIMPLE_NAV_3D__PLANNERS__UAV_PLANNER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/planners/planner_base.hpp"

namespace simple_nav_3d
{

/// Compact 3D occupancy grid built from dscovox voxels.
struct VoxelGrid3D
{
  bool valid{false};
  double resolution{0.2};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  int size_x{0};
  int size_y{0};
  int size_z{0};
  std::vector<uint8_t> occupied;  // flat array [z * size_y * size_x + y * size_x + x]

  inline int flatten(int x, int y, int z) const
  {
    return z * size_y * size_x + y * size_x + x;
  }

  inline bool in_bounds(int x, int y, int z) const
  {
    return x >= 0 && y >= 0 && z >= 0 && x < size_x && y < size_y && z < size_z;
  }
};

class UavPlanner : public PlannerBase
{
public:
  explicit UavPlanner(const NodeParameters & params);

  std::string name() const override;
  PlannerOutput compute_plan(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & goal,
    const MapSnapshot & map_snapshot) override;

  /// Called by the planner node to update the cached 3D voxel grid.
  void update_voxel_grid(const VoxelGrid3D & grid);

private:
  NodeParameters params_;
  std::mutex grid_mutex_;
  VoxelGrid3D cached_grid_;
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__PLANNERS__UAV_PLANNER_HPP_
