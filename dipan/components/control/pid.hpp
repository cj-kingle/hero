#pragma once

namespace chassis {
struct PidConfig {
  float kp = 0, ki = 0, kd = 0;
  float output_limit = 0, integral_limit = 0;
};
// Gains default to zero until hardware tuning. dt is in seconds.
class Pid {
 public:
  explicit Pid(PidConfig config = {}) : config_(config) {}
  void reset();
  void setOutputLimit(float limit) { config_.output_limit=limit; }
  float update(float target, float measurement, float dt,float feedforward=0);
  float integral() const { return integral_; }
 private:
  PidConfig config_;
  float integral_ = 0, previous_measurement_ = 0;
  bool initialized_ = false;
};
}
