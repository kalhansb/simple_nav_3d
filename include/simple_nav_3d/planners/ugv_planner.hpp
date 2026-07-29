#ifndef SIMPLE_NAV_3D__PLANNERS__UGV_PLANNER_HPP_
#define SIMPLE_NAV_3D__PLANNERS__UGV_PLANNER_HPP_

#include <string>

#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/planners/planner_base.hpp"

namespace simple_nav_3d
{

class UgvPlanner : public PlannerBase
{
public:
  explicit UgvPlanner(const NodeParameters & params);

  std::string name() const override;
  PlannerOutput compute_plan(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::PoseStamped & goal,
    const MapSnapshot & map_snapshot) override;

private:
  NodeParameters params_;
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__PLANNERS__UGV_PLANNER_HPP_
