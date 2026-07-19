#pragma once

#include <string>

#include <Eigen/Core>

namespace drone_planning
{

enum class YawMode
{
  Fixed,
  PathTangent
};

struct YawReferenceParameters
{
  YawMode mode{YawMode::Fixed};
  double fixed_yaw{0.0};
  double tangent_speed_threshold{0.10};
  double terminal_blend_distance{0.80};
  double filter_time_constant{0.30};
  double max_yaw_rate{0.80};
};

YawMode parse_yaw_mode(const std::string & value);
std::string yaw_mode_name(YawMode mode);

class YawReferenceGenerator
{
public:
  explicit YawReferenceGenerator(YawReferenceParameters parameters = {});

  void initialize(double initial_yaw);

  double update(
    const Eigen::Vector3d & trajectory_position,
    const Eigen::Vector3d & trajectory_velocity,
    const Eigen::Vector3d & goal_position,
    double terminal_yaw,
    double dt);

  double reference() const;

private:
  YawReferenceParameters parameters_;
  double reference_yaw_{0.0};
  double tangent_yaw_{0.0};
  bool initialized_{false};
  bool have_valid_tangent_{false};
};

}  // namespace drone_planning
