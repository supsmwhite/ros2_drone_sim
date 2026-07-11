#ifndef DRONE_CONTROLLER__POSITION__POSITION_CONTROLLER_HPP_
#define DRONE_CONTROLLER__POSITION__POSITION_CONTROLLER_HPP_

#include <optional>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace drone_controller
{

class PositionController
{
public:
  void set_goal(const geometry_msgs::msg::PointStamped & goal);
  void set_odometry(const nav_msgs::msg::Odometry & odometry);

  bool has_goal() const;
  bool has_odometry() const;

private:
  std::optional<geometry_msgs::msg::PointStamped> goal_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__POSITION__POSITION_CONTROLLER_HPP_
