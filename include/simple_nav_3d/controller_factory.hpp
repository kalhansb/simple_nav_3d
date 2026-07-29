#ifndef SIMPLE_NAV_3D__CONTROLLER_FACTORY_HPP_
#define SIMPLE_NAV_3D__CONTROLLER_FACTORY_HPP_

#include <memory>
#include <string>

#include "simple_nav_3d/controllers/controller_base.hpp"
#include "simple_nav_3d/parameters.hpp"

namespace simple_nav_3d
{

std::unique_ptr<ControllerBase> create_controller(
  const std::string & controller_name,
  const NodeParameters & params);

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLER_FACTORY_HPP_
