#pragma once

namespace gimbal {
struct PidConfig {
  float kp = 0, ki = 0, kd = 0;
  float output_limit = 0, integral_limit = 0;
  float measurement_filter_tau = 0; // seconds; 0 bypasses measurement low-pass
  bool derivative_from_rate = false; // update() then requires the same-frame measured rate as its fifth argument
  float derivative_filter_tau = 0; // seconds; D-only low-pass, separate from P/I feedback
  float approach_braking_acceleration = 0; // position loop: brake toward-target motion; 0 disables
  bool variable_kp=false;
  float small_error_kp=0,small_error_range=0,full_gain_error=0; // error units match target; position loops use radians
  float error_deadband=0,deadband_transition=0; // stationary P/I quiet band; moving targets use the full error
  float derivative_deadband_rate=0; // optional neutral D fade: zero below 1/4 this rate, full damping at this rate; 0 disables
  bool variable_kd=false;
  float small_error_kd=0,small_d_error_range=0,full_d_error=0; // independent D gain and error bands; position loops use radians
};
bool ValidPidErrorShaping(const PidConfig&);
struct PidTerms {
  float p=0, i=0, d=0, output=0;
  float feedforward=0; // output includes this external contribution and final limiting
  float measurement=0; // actual measurement used by PID; safety checks use raw feedback
  float effective_kp=0,control_error=0; // actual proportional gain and P/I error after shaping
};
// Gains default to zero until hardware tuning. dt is in seconds.
class Pid {
 public:
  explicit Pid(PidConfig config = {}) : config_(config) {}
  void reset();
  float update(float target, float measurement, float dt, float feedforward=0,
      float measurement_rate=0, bool target_stationary=true, float target_rate=0);
  float integral() const { return integral_; }
  const PidTerms& terms() const { return terms_; }
 private:
  PidConfig config_;
  float integral_ = 0, previous_measurement_ = 0;
  float filtered_derivative_ = 0;
  bool initialized_ = false;
  PidTerms terms_{};
};
}
