#ifndef SIMPLE_NAV_3D__PLANNER_FACTORY_HPP_
#define SIMPLE_NAV_3D__PLANNER_FACTORY_HPP_

#include <memory>
#include <string>

#include "simple_nav_3d/parameters.hpp"
#include "simple_nav_3d/planners/planner_base.hpp"

namespace simple_nav_3d
{

std::unique_ptr<PlannerBase> create_planner(
  const std::string & planner_name,
  const NodeParameters & params);

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__PLANNER_FACTORY_HPP_
