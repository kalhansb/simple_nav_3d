#ifndef SIMPLE_NAV_3D__PLANNERS__PLANNER_BASE_HPP_
#define SIMPLE_NAV_3D__PLANNERS__PLANNER_BASE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "simple_nav_3d/mapping/map_snapshot.hpp"

namespace simple_nav_3d
{

struct PlannerOutput
{
  bool has_path{false};
  nav_msgs::msg::Path path;
};

class PlannerBase
{
public:
  virtual ~PlannerBase() = default;
  virtual std::string name() const = 0;
  virtual PlannerOutput compute_plan(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & goal,
    const MapSnapshot & map_snapshot) = 0;
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__PLANNERS__PLANNER_BASE_HPP_
