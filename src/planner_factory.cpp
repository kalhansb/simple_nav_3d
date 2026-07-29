#include "simple_nav_3d/planner_factory.hpp"

#include <memory>
#include <stdexcept>

#include "simple_nav_3d/planners/uav_planner.hpp"
#include "simple_nav_3d/planners/ugv_planner.hpp"

namespace simple_nav_3d
{

std::unique_ptr<PlannerBase> create_planner(
  const std::string & planner_name,
  const NodeParameters & params)
{
  if (planner_name == "planner_2d") {
    return std::make_unique<UgvPlanner>(params);
  }
  if (planner_name == "planner_3d") {
    return std::make_unique<UavPlanner>(params);
  }
  throw std::runtime_error("Unsupported planner: " + planner_name);
}

}  // namespace simple_nav_3d
