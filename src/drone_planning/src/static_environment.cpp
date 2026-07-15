#include "drone_planning/static_environment.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace drone_planning
{
namespace
{

void validate_box(const AxisAlignedBox & box, const char * description)
{
  if (!box.min_corner.allFinite() || !box.max_corner.allFinite()) {
    throw std::invalid_argument(std::string(description) + " bounds must be finite");
  }
  if ((box.min_corner.array() >= box.max_corner.array()).any()) {
    throw std::invalid_argument(std::string(description) + " must have positive size");
  }
}

}  // namespace

StaticEnvironment::StaticEnvironment(
  AxisAlignedBox workspace, std::vector<AxisAlignedBox> obstacles)
: workspace_(std::move(workspace)), obstacles_(std::move(obstacles))
{
  validate_box(workspace_, "workspace");
  for (const auto & obstacle : obstacles_) {
    validate_box(obstacle, "obstacle");
  }
}

const AxisAlignedBox & StaticEnvironment::workspace() const
{
  return workspace_;
}

const std::vector<AxisAlignedBox> & StaticEnvironment::obstacles() const
{
  return obstacles_;
}

}  // namespace drone_planning
