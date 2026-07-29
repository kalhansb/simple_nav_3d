#include "simple_nav_3d/controller_factory.hpp"

#include <memory>
#include <stdexcept>

#include "simple_nav_3d/controllers/uav_controller.hpp"
#include "simple_nav_3d/controllers/ugv_controller.hpp"

namespace simple_nav_3d
{

std::unique_ptr<ControllerBase> create_controller(
  const std::string & controller_name,
  const NodeParameters & params)
{
  if (controller_name == "local_controller_ugv") {
    return std::make_unique<UgvController>(params);
  }
  if (controller_name == "flight_controller_uav") {
    return std::make_unique<UavController>(params);
  }
  throw std::runtime_error("Unsupported controller: " + controller_name);
}

}  // namespace simple_nav_3d
