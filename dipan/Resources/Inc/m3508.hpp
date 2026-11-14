#pragma once
#include "pid.hpp"
#include <cstdint>
namespace chassis {
// Motor state and speed-loop calculation only. CAN parsing/transmission is pending.
class M3508 {
 public:
  explicit M3508(PidConfig speed_pid = {}) : speed_pid_(speed_pid) {}
  // Shaft rpm BEFORE the 14:1 wheel reduction used by this robot.
  void setFeedback(float motor_rpm, float temperature_c, uint32_t now_ms);
  bool ready(uint32_t now_ms) const;
  float calculate(float target_motor_rpm, bool enabled, uint32_t now_ms, float dt);
  void reset() { speed_pid_.reset(); }
  void setCurrentLimit(float limit) { speed_pid_.setOutputLimit(limit); }
 private:
  Pid speed_pid_;
  float motor_rpm_ = 0, temperature_c_ = 0;
  uint32_t last_ms_ = 0;
  bool received_ = false;
};
}
