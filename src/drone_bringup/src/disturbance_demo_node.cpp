#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drone_msgs/msg/controller_diagnostics.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace drone_bringup
{

namespace
{

constexpr double kVectorEpsilon = 1.0e-9;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return finite(point.x) && finite(point.y) && finite(point.z);
}

std::string format_vector(double x, double y, double z, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << '[' << x << ", " << y << ", " << z
         << ']';
  return stream.str();
}

std::string format_vector_2d(double x, double y, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << '[' << x << ", " << y << ']';
  return stream.str();
}

}  // namespace

class DisturbanceDemoNode : public rclcpp::Node
{
public:
  DisturbanceDemoNode()
  : Node("disturbance_demo_node")
  {
    profile_ = declare_parameter<std::string>("profile", "short_gust");
    if (profile_ != "short_gust" && profile_ != "persistent_release") {
      throw std::invalid_argument("profile must be short_gust or persistent_release");
    }

    target_x_ = declare_parameter<double>("target_x", 0.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);
    target_z_ = declare_parameter<double>("target_z", 1.5);
    start_delay_ = declare_parameter<double>("start_delay", 5.0);
    force_x_ = declare_parameter<double>("force_x", 0.30);
    force_y_ = declare_parameter<double>("force_y", 0.0);
    force_z_ = declare_parameter<double>("force_z", 0.0);
    const double profile_duration = profile_ == "persistent_release" ? 10.0 : 2.0;
    disturbance_duration_ =
      declare_parameter<double>("disturbance_duration", profile_duration);
    recovery_duration_ = declare_parameter<double>("recovery_duration", 10.0);
    force_publish_rate_ = declare_parameter<double>("force_publish_rate", 25.0);
    force_arrow_scale_ = declare_parameter<double>("force_arrow_scale", 1.5);
    integral_arrow_scale_ = declare_parameter<double>("integral_arrow_scale", 1.5);
    show_integral_arrow_ = declare_parameter<bool>("show_integral_arrow", true);
    show_status_text_ = declare_parameter<bool>("show_status_text", true);
    settle_position_tolerance_ =
      declare_parameter<double>("settle_position_tolerance", 0.05);
    settle_velocity_tolerance_ =
      declare_parameter<double>("settle_velocity_tolerance", 0.05);
    settle_duration_ = declare_parameter<double>("settle_duration", 1.0);
    validate_parameters();

    goal_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/drone/goal", 10);
    wrench_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>("/drone/external_wrench", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/drone/disturbance/markers", rclcpp::QoS(1).reliable().transient_local());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {handle_odometry(*message);});
    diagnostics_subscription_ = create_subscription<drone_msgs::msg::ControllerDiagnostics>(
      "/drone/controller/diagnostics", 10,
      [this](const drone_msgs::msg::ControllerDiagnostics::SharedPtr message) {
        handle_diagnostics(*message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / force_publish_rate_));
    timer_ = create_wall_timer(period, [this]() {update();});

    RCLCPP_INFO(
      get_logger(),
      "Disturbance demo started: profile=%s target=[%.2f, %.2f, %.2f] m "
      "Equivalent External Force=[%.2f, %.2f, %.2f] N rate=%.1f Hz",
      profile_.c_str(), target_x_, target_y_, target_z_, force_x_, force_y_, force_z_,
      force_publish_rate_);
    RCLCPP_INFO(get_logger(), "[WAIT_FOR_ODOM] waiting for valid /drone/odom");
  }

private:
  enum class Stage
  {
    WaitForOdom,
    TakeoffAndSettle,
    Countdown,
    DisturbanceActive,
    Recovery,
    Complete
  };

  using SteadyClock = std::chrono::steady_clock;
  using TimePoint = SteadyClock::time_point;

  void validate_parameters() const
  {
    const double force_magnitude = std::hypot(force_x_, force_y_, force_z_);
    if (!finite(target_x_) || !finite(target_y_) || !finite(target_z_) || target_z_ <= 0.0 ||
      std::abs(target_x_) > 1000.0 || std::abs(target_y_) > 1000.0 || target_z_ > 1000.0)
    {
      throw std::invalid_argument("target coordinates must be finite and target_z must be positive");
    }
    if (!finite(start_delay_) || start_delay_ < 0.0 || start_delay_ > 3600.0 ||
      !finite(disturbance_duration_) || disturbance_duration_ <= 0.0 ||
      disturbance_duration_ > 3600.0 || !finite(recovery_duration_) ||
      recovery_duration_ <= 0.0 || recovery_duration_ > 3600.0)
    {
      throw std::invalid_argument("demo durations must be finite and within 0..3600 seconds");
    }
    if (!finite(force_x_) || !finite(force_y_) || !finite(force_z_) ||
      !finite(force_magnitude) || force_magnitude > 2.0)
    {
      throw std::invalid_argument(
              "force components must be finite with magnitude no greater than 2 N");
    }
    if (!finite(force_publish_rate_) || force_publish_rate_ < 10.0 ||
      force_publish_rate_ > 500.0)
    {
      throw std::invalid_argument(
              "force_publish_rate must be finite and within 10..500 Hz to satisfy the wrench timeout");
    }
    if (!finite(force_arrow_scale_) || force_arrow_scale_ <= 0.0 ||
      force_arrow_scale_ > 100.0 || !finite(integral_arrow_scale_) ||
      integral_arrow_scale_ <= 0.0 || integral_arrow_scale_ > 100.0)
    {
      throw std::invalid_argument("arrow scales must be finite and within 0..100");
    }
    if (!finite(settle_position_tolerance_) || settle_position_tolerance_ <= 0.0 ||
      settle_position_tolerance_ > 10.0 || !finite(settle_velocity_tolerance_) ||
      settle_velocity_tolerance_ <= 0.0 || settle_velocity_tolerance_ > 10.0 ||
      !finite(settle_duration_) || settle_duration_ <= 0.0 || settle_duration_ > 60.0)
    {
      throw std::invalid_argument("settling thresholds or duration are invalid");
    }
  }

  void handle_odometry(const nav_msgs::msg::Odometry & message)
  {
    const auto & position = message.pose.pose.position;
    const auto & velocity = message.twist.twist.linear;
    const auto & orientation = message.pose.pose.orientation;
    const double orientation_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (message.header.frame_id != "map" || !finite_point(position) ||
      !finite(velocity.x) || !finite(velocity.y) ||
      !finite(velocity.z) || !finite(orientation.x) || !finite(orientation.y) ||
      !finite(orientation.z) || !finite(orientation.w) || !finite(orientation_norm) ||
      orientation_norm <= kVectorEpsilon)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring non-finite /drone/odom message");
      return;
    }
    position_ = position;
    speed_ = std::hypot(velocity.x, velocity.y, velocity.z);
    has_odometry_ = true;
  }

  void handle_diagnostics(const drone_msgs::msg::ControllerDiagnostics & message)
  {
    integral_x_ = finite(message.horizontal_i_acceleration_x) ?
      message.horizontal_i_acceleration_x : 0.0;
    integral_y_ = finite(message.horizontal_i_acceleration_y) ?
      message.horizontal_i_acceleration_y : 0.0;
    horizontal_saturated_ = message.horizontal_saturated;
    altitude_saturated_ = message.altitude_saturated;
    attitude_saturated_ = message.attitude_saturated;
    mixer_saturated_ = message.mixer_saturated;
  }

  double elapsed_since(const TimePoint & start, const TimePoint & now) const
  {
    return std::chrono::duration<double>(now - start).count();
  }

  double position_error_3d() const
  {
    return std::hypot(position_.x - target_x_, position_.y - target_y_, position_.z - target_z_);
  }

  double horizontal_error() const
  {
    return std::hypot(position_.x - target_x_, position_.y - target_y_);
  }

  bool is_settled() const
  {
    return has_odometry_ && position_error_3d() < settle_position_tolerance_ &&
           speed_ < settle_velocity_tolerance_;
  }

  void transition_to(Stage next, const TimePoint & now)
  {
    stage_ = next;
    stage_started_ = now;
    settle_started_.reset();
    switch (stage_) {
      case Stage::WaitForOdom:
        RCLCPP_INFO(get_logger(), "[WAIT_FOR_ODOM] waiting for valid /drone/odom");
        break;
      case Stage::TakeoffAndSettle:
        RCLCPP_INFO(
          get_logger(), "[TAKEOFF_AND_SETTLE] holding target until position and speed settle");
        break;
      case Stage::Countdown:
        RCLCPP_INFO(
          get_logger(), "[COUNTDOWN] disturbance starts in %.1f s", start_delay_);
        break;
      case Stage::DisturbanceActive:
        RCLCPP_INFO(
          get_logger(), "[DISTURBANCE_ACTIVE] F=[%.2f, %.2f, %.2f] N", force_x_, force_y_,
          force_z_);
        break;
      case Stage::Recovery:
        RCLCPP_INFO(get_logger(), "[RECOVERY] external force removed");
        break;
      case Stage::Complete:
        RCLCPP_INFO(
          get_logger(), "[COMPLETE] final horizontal error=%.3f m", horizontal_error());
        break;
    }
  }

  void update_stage(const TimePoint & now)
  {
    if (stage_ == Stage::WaitForOdom) {
      if (has_odometry_) {
        transition_to(Stage::TakeoffAndSettle, now);
      }
      return;
    }
    if (stage_ == Stage::TakeoffAndSettle) {
      if (!is_settled()) {
        settle_started_.reset();
        return;
      }
      if (!settle_started_) {
        settle_started_ = now;
        return;
      }
      if (elapsed_since(*settle_started_, now) >= settle_duration_) {
        transition_to(Stage::Countdown, now);
      }
      return;
    }
    if (stage_ == Stage::Countdown) {
      if (!is_settled()) {
        transition_to(Stage::TakeoffAndSettle, now);
      } else if (elapsed_since(stage_started_, now) >= start_delay_) {
        transition_to(Stage::DisturbanceActive, now);
      }
      return;
    }
    if (stage_ == Stage::DisturbanceActive &&
      elapsed_since(stage_started_, now) >= disturbance_duration_)
    {
      transition_to(Stage::Recovery, now);
      return;
    }
    if (stage_ == Stage::Recovery && elapsed_since(stage_started_, now) >= recovery_duration_) {
      transition_to(Stage::Complete, now);
    }
  }

  void publish_goal(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.pose.position.x = target_x_;
    message.pose.position.y = target_y_;
    message.pose.position.z = target_z_;
    message.pose.orientation.w = 1.0;
    goal_publisher_->publish(message);
  }

  void publish_wrench(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    if (stage_ == Stage::DisturbanceActive) {
      message.wrench.force.x = force_x_;
      message.wrench.force.y = force_y_;
      message.wrench.force.z = force_z_;
    }
    wrench_publisher_->publish(message);
  }

  visualization_msgs::msg::Marker marker_base(
    const rclcpp::Time & stamp, const std::string & marker_namespace, int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "map";
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  visualization_msgs::msg::Marker delete_marker(
    const rclcpp::Time & stamp, const std::string & marker_namespace, int id, int type) const
  {
    auto marker = marker_base(stamp, marker_namespace, id, type);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  visualization_msgs::msg::Marker target_marker(const rclcpp::Time & stamp) const
  {
    auto marker = marker_base(
      stamp, "disturbance_target", 0,
      visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position.x = target_x_;
    marker.pose.position.y = target_y_;
    marker.pose.position.z = target_z_;
    marker.scale.x = 0.16;
    marker.scale.y = 0.16;
    marker.scale.z = 0.16;
    marker.color.r = 0.20F;
    marker.color.g = 1.00F;
    marker.color.b = 0.30F;
    marker.color.a = 0.95F;
    return marker;
  }

  visualization_msgs::msg::Marker error_marker(const rclcpp::Time & stamp) const
  {
    if (!has_odometry_) {
      return delete_marker(
        stamp, "disturbance_position_error", 0,
        visualization_msgs::msg::Marker::LINE_STRIP);
    }
    auto marker = marker_base(
      stamp, "disturbance_position_error", 0, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = 0.018;
    marker.color.r = 1.00F;
    marker.color.g = 0.85F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;
    marker.points.push_back(position_);
    geometry_msgs::msg::Point target;
    target.x = target_x_;
    target.y = target_y_;
    target.z = target_z_;
    marker.points.push_back(target);
    return marker;
  }

  visualization_msgs::msg::Marker force_marker(const rclcpp::Time & stamp) const
  {
    const double magnitude = std::hypot(force_x_, force_y_, force_z_);
    if (!has_odometry_ || stage_ != Stage::DisturbanceActive || magnitude <= kVectorEpsilon) {
      return delete_marker(
        stamp, "equivalent_external_force", 0,
        visualization_msgs::msg::Marker::ARROW);
    }
    auto marker = marker_base(
      stamp, "equivalent_external_force", 0, visualization_msgs::msg::Marker::ARROW);
    marker.scale.x = 0.055;
    marker.scale.y = 0.11;
    marker.scale.z = 0.14;
    marker.color.r = 1.00F;
    marker.color.g = 0.05F;
    marker.color.b = 0.03F;
    marker.color.a = 1.00F;
    marker.points.push_back(position_);
    geometry_msgs::msg::Point end = position_;
    end.x += force_x_ * force_arrow_scale_;
    end.y += force_y_ * force_arrow_scale_;
    end.z += force_z_ * force_arrow_scale_;
    marker.points.push_back(end);
    return marker;
  }

  visualization_msgs::msg::Marker integral_marker(const rclcpp::Time & stamp) const
  {
    const double magnitude = std::hypot(integral_x_, integral_y_);
    if (!show_integral_arrow_ || !has_odometry_ || magnitude <= kVectorEpsilon) {
      return delete_marker(
        stamp, "horizontal_integral_acceleration", 0,
        visualization_msgs::msg::Marker::ARROW);
    }
    auto marker = marker_base(
      stamp, "horizontal_integral_acceleration", 0, visualization_msgs::msg::Marker::ARROW);
    marker.scale.x = 0.045;
    marker.scale.y = 0.09;
    marker.scale.z = 0.12;
    marker.color.r = 0.05F;
    marker.color.g = 0.65F;
    marker.color.b = 1.00F;
    marker.color.a = 1.00F;
    marker.points.push_back(position_);
    geometry_msgs::msg::Point end = position_;
    end.x += integral_x_ * integral_arrow_scale_;
    end.y += integral_y_ * integral_arrow_scale_;
    marker.points.push_back(end);
    return marker;
  }

  std::string status_text(const TimePoint & now) const
  {
    std::ostringstream stream;
    if (stage_ == Stage::WaitForOdom || stage_ == Stage::TakeoffAndSettle) {
      stream << "TAKEOFF / SETTLING";
    } else if (stage_ == Stage::Countdown) {
      const double countdown = std::max(
        0.0, start_delay_ - elapsed_since(stage_started_, now));
      stream << "GUST IN " << std::fixed << std::setprecision(1) << countdown << " s";
    } else if (stage_ == Stage::DisturbanceActive) {
      const double remaining = std::max(
        0.0, disturbance_duration_ - elapsed_since(stage_started_, now));
      stream << "GUST ACTIVE | " << std::fixed << std::setprecision(1) << remaining << " s\n"
             << "F=" << format_vector(force_x_, force_y_, force_z_, 2)
             << " N | e_xy=" << std::setprecision(3) << horizontal_error() << " m\n"
             << "I_xy=" << format_vector_2d(integral_x_, integral_y_, 3) << " m/s^2";
    } else if (stage_ == Stage::Recovery) {
      stream << "RECOVERY | e_xy=" << std::fixed << std::setprecision(3)
             << horizontal_error() << " m\n"
             << "I_xy=" << format_vector_2d(integral_x_, integral_y_, 3) << " m/s^2";
    } else {
      stream << "COMPLETE | e_xy=" << std::fixed << std::setprecision(3)
             << horizontal_error() << " m";
    }

    std::string warning;
    const auto append_warning = [&warning](const std::string & label) {
        if (!warning.empty()) {
          warning += " | ";
        }
        warning += label + " SATURATION";
      };
    if (horizontal_saturated_) {
      append_warning("H");
    }
    if (altitude_saturated_) {
      append_warning("Z");
    }
    if (attitude_saturated_) {
      append_warning("ATT");
    }
    if (mixer_saturated_) {
      append_warning("MIXER");
    }
    if (!warning.empty()) {
      stream << "\nWARNING: " << warning;
    }
    return stream.str();
  }

  visualization_msgs::msg::Marker text_marker(
    const rclcpp::Time & stamp, const TimePoint & now) const
  {
    if (!show_status_text_) {
      return delete_marker(
        stamp, "disturbance_status", 0,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    }
    auto marker = marker_base(
      stamp, "disturbance_status", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    marker.pose.position.x = has_odometry_ ? position_.x : target_x_;
    marker.pose.position.y = has_odometry_ ? position_.y : target_y_;
    marker.pose.position.z = (has_odometry_ ? position_.z : target_z_) + 0.85;
    marker.scale.z = 0.16;
    marker.color.a = 1.00F;
    if (stage_ == Stage::DisturbanceActive) {
      marker.color.r = 1.00F;
      marker.color.g = 0.25F;
      marker.color.b = 0.05F;
    } else if (stage_ == Stage::Recovery) {
      marker.color.r = 1.00F;
      marker.color.g = 0.85F;
      marker.color.b = 0.05F;
    } else if (stage_ == Stage::Complete) {
      marker.color.r = 0.10F;
      marker.color.g = 1.00F;
      marker.color.b = 0.20F;
    } else {
      marker.color.r = 1.00F;
      marker.color.g = 1.00F;
      marker.color.b = 1.00F;
    }
    marker.text = status_text(now);
    return marker;
  }

  void publish_markers(const rclcpp::Time & stamp, const TimePoint & now)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(target_marker(stamp));
    array.markers.push_back(error_marker(stamp));
    array.markers.push_back(force_marker(stamp));
    array.markers.push_back(integral_marker(stamp));
    array.markers.push_back(text_marker(stamp, now));
    marker_publisher_->publish(array);
  }

  void update()
  {
    const TimePoint steady_now = SteadyClock::now();
    const rclcpp::Time stamp = now();
    update_stage(steady_now);
    publish_goal(stamp);
    publish_wrench(stamp);
    publish_markers(stamp, steady_now);
  }

  std::string profile_;
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{1.5};
  double start_delay_{5.0};
  double force_x_{0.30};
  double force_y_{0.0};
  double force_z_{0.0};
  double disturbance_duration_{2.0};
  double recovery_duration_{10.0};
  double force_publish_rate_{25.0};
  double force_arrow_scale_{1.5};
  double integral_arrow_scale_{1.5};
  bool show_integral_arrow_{true};
  bool show_status_text_{true};
  double settle_position_tolerance_{0.05};
  double settle_velocity_tolerance_{0.05};
  double settle_duration_{1.0};

  Stage stage_{Stage::WaitForOdom};
  TimePoint stage_started_{SteadyClock::now()};
  std::optional<TimePoint> settle_started_;
  geometry_msgs::msg::Point position_;
  double speed_{0.0};
  bool has_odometry_{false};
  double integral_x_{0.0};
  double integral_y_{0.0};
  bool horizontal_saturated_{false};
  bool altitude_saturated_{false};
  bool attitude_saturated_{false};
  bool mixer_saturated_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<drone_msgs::msg::ControllerDiagnostics>::SharedPtr
    diagnostics_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_bringup::DisturbanceDemoNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("disturbance_demo_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
