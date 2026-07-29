#ifndef SIMPLE_NAV_3D__CONTROLLERS__UAV_CONTROLLER_HPP_
#define SIMPLE_NAV_3D__CONTROLLERS__UAV_CONTROLLER_HPP_

#include <string>

#include "simple_nav_3d/controllers/controller_base.hpp"
#include "simple_nav_3d/parameters.hpp"

namespace simple_nav_3d
{

class UavController : public ControllerBase
{
public:
  explicit UavController(const NodeParameters & params);

  std::string name() const override;
  geometry_msgs::msg::Twist compute_command(
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::Path & global_path,
    const MapSnapshot & map_snapshot) override;

private:
  NodeParameters params_;
};

}  // namespace simple_nav_3d

#endif  // SIMPLE_NAV_3D__CONTROLLERS__UAV_CONTROLLER_HPP_
