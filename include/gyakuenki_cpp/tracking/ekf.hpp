#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gyakuenki_cpp/tracking/ekf.hpp"
#include "gyakuenki_interfaces/msg/projected_object.hpp"
#include "keisan/matrix.hpp"

namespace gyakuenki_cpp
{

class BallTracker
{
public:
  BallTracker();

  gyakuenki_interfaces::msg::ProjectedObject process(
    const keisan::Matrix<4, 1> & Pc, const gyakuenki_interfaces::msg::ProjectedObject & raw_ball,
    const rclcpp::Time & now);

private:
  bool ball_initialized_;
  rclcpp::Time last_ball_time_;
  BallEKF ball_ekf_;
};

}  // namespace gyakuenki_cpp
