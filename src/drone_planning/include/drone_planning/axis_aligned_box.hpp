#pragma once

#include <Eigen/Core>

namespace drone_planning
{

struct AxisAlignedBox
{
  Eigen::Vector3d min_corner{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_corner{Eigen::Vector3d::Zero()};
};

}  // namespace drone_planning
