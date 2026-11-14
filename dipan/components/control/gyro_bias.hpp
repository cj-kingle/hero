#pragma once
#include <cstdint>
namespace chassis {
// Input units: acceleration m/s^2, angular velocity rad/s. No sensor I/O.
class GyroBias {
 public:
  explicit GyroBias(uint32_t max_sample_gap_ms=1) : max_gap_ms_(max_sample_gap_ms ? max_sample_gap_ms : 1) {}
  void reset();
  bool update(const float acc[3], const float gyro[3], uint32_t sample_ms);
  bool ready() const { return ready_; }
  uint32_t samples() const { return samples_; }
  const float* bias() const { return bias_; }
  bool stationary() const { return stationary_; }
 private:
  float sum_[3]{}, bias_[3]{};
  uint32_t samples_ = 0, last_ms_ = 0;
  bool ready_ = false, seen_ = false;
  bool stationary_=false;
  uint32_t max_gap_ms_=1;
};
}
