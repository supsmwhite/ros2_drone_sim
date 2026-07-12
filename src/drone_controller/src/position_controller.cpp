#include "drone_controller/position/position_controller.hpp"

namespace drone_controller
{

void PositionController::set_goal(const geometry_msgs::msg::PoseStamped & goal)
{
  goal_ = goal;
}

void PositionController::set_odometry(const nav_msgs::msg::Odometry & odometry)
{
  odometry_ = odometry;
}

bool PositionController::has_goal() const
{
  return goal_.has_value();
}

bool PositionController::has_odometry() const
{
  return odometry_.has_value();
}

}  // namespace drone_controller
