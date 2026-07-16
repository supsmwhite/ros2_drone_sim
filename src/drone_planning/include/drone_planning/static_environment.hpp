#pragma once

#include <vector>

#include "drone_planning/axis_aligned_box.hpp"

namespace drone_planning
{

class StaticEnvironment
{
public:
  StaticEnvironment(AxisAlignedBox workspace, std::vector<AxisAlignedBox> obstacles);

  const AxisAlignedBox & workspace() const;
  const std::vector<AxisAlignedBox> & obstacles() const;

private:
  AxisAlignedBox workspace_;
  std::vector<AxisAlignedBox> obstacles_;
};

}  // namespace drone_planning
