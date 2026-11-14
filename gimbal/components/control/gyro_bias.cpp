#include "gyro_bias.hpp"
#include <cmath>
namespace gimbal {
void GyroBias::reset() { const auto gap=max_gap_ms_;*this = GyroBias{gap}; }
bool GyroBias::update(const float acc[3], const float gyro[3], uint32_t ms) {
  if (!acc || !gyro) { reset(); return false; }
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(acc[i]) || !std::isfinite(gyro[i])) { reset(); return false; }
  const float a = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
  const bool stationary = std::fabs(a - 9.80665f) < 0.5f &&
      std::fabs(gyro[0]) < 0.08f && std::fabs(gyro[1]) < 0.08f && std::fabs(gyro[2]) < 0.08f;
  stationary_=stationary;
  if (ready_) return true; // Normal motion after calibration must not recalibrate bias.
  if (a < 1.0f || a > 40.0f) { reset(); return false; }
  if(seen_ && ms==last_ms_)return false; // a repeated timestamp is never another sample
  const bool consecutive = !seen_ || uint32_t(ms - last_ms_) <= max_gap_ms_;
  if (!consecutive || !stationary) reset();
  stationary_=stationary;
  seen_ = true; last_ms_ = ms;
  if (!stationary) return false;
  for (int i = 0; i < 3; ++i) sum_[i] += gyro[i];
  if (++samples_ >= 2000u) {
    for (int i = 0; i < 3; ++i) bias_[i] = sum_[i] / samples_;
    ready_ = true;
  }
  return ready_;
}
}

