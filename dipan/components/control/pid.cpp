#include "pid.hpp"
#include <cmath>
#include <algorithm>
namespace chassis {
void Pid::reset() { integral_ = previous_measurement_ = 0; initialized_ = false; }
float Pid::update(float target, float measurement, float dt,float feedforward) {
  if (!std::isfinite(target) || !std::isfinite(measurement) ||
      !std::isfinite(dt) || dt <= 0 || dt > 0.1f || !std::isfinite(feedforward) ||
      !std::isfinite(config_.kp) || !std::isfinite(config_.ki) ||
      !std::isfinite(config_.kd) || !std::isfinite(config_.integral_limit) ||
      config_.integral_limit < 0 ||
      !std::isfinite(config_.output_limit) || config_.output_limit <= 0) {
    reset(); return 0;
  }
  const float error = target - measurement;
  // Derivative on measurement avoids target-step derivative kick.
  const float derivative = initialized_ ? -(measurement - previous_measurement_) / dt : 0;
  previous_measurement_ = measurement; initialized_ = true;
  const float limit = std::fabs(config_.integral_limit);
  const float candidate = std::clamp(integral_ + config_.ki * error * dt, -limit, limit);
  const float raw = feedforward+config_.kp * error + candidate + config_.kd * derivative;
  if (!std::isfinite(raw)) { reset(); return 0; }
  if ((raw <= config_.output_limit && raw >= -config_.output_limit) ||
      (raw > config_.output_limit && error < 0) ||
      (raw < -config_.output_limit && error > 0)) integral_ = candidate;
  return std::clamp(feedforward+config_.kp * error + integral_ + config_.kd * derivative,
                    -config_.output_limit, config_.output_limit);
}
}
