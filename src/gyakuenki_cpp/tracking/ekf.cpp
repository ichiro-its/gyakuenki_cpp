#include "gyakuenki_cpp/tracking/ball_tracker.hpp"

namespace gyakuenki_cpp
{

BallTracker::BallTracker() : ball_initialized_(false) {}

gyakuenki_interfaces::msg::ProjectedObject BallTracker::process(
  const keisan::Matrix<4, 1> & Pc, const gyakuenki_interfaces::msg::ProjectedObject & raw_ball,
  const rclcpp::Time & now)
{
  auto output = raw_ball;

  // First detection
  if (!ball_initialized_) {
    ball_ekf_.init(Pc[0][0], Pc[1][0]);
    last_ball_time_ = now;
    ball_initialized_ = true;
    return output;
  }

  double dt = (now - last_ball_time_).seconds();
  last_ball_time_ = now;

  if (dt <= 0.0) {
    return output;
  }

  // Predict step
  ball_ekf_.predict(dt);

  // Measurement
  keisan::Matrix<2, 1> z;
  z[0][0] = Pc[0][0];
  z[1][0] = Pc[1][0];

  ball_ekf_.update(z);

  auto pos = ball_ekf_.getPosition();

  output.position.x = pos(0, 0);
  output.position.y = pos(1, 0);

  return output;
}

}  // namespace gyakuenki_cpp
