#pragma once
#include "dm_protocol.hpp"

namespace damiao {
struct MotorConfig {
  uint16_t control_id = 0, feedback_id = 0xffff;
  Limits limits{};
  int direction = 0;                 // Must be +1/-1 after checking the hardware.
  float motor_per_joint = 1;
  float feedback_wrap = 6.283185307f; // Verify against actual output-shaft feedback.
  bool confirmed = false;
  bool absolute_position = false; // use the motor's persistent zero instead of software capture
  bool continuous_absolute = false; // unwrap yaw while feedback is continuous; no persisted turn count
};
struct JointFeedback {
  float position = 0, velocity = 0, torque = 0;
  bool online = false, healthy = false, zeroed = false, enabled = false;
  uint8_t state = 0;
};
class Motor {
 public:
  explicit Motor(MotorConfig config = {}) : config_(config) {}
  bool onFeedback(uint16_t id, const uint8_t* data, size_t size, uint32_t now);
  bool captureZero(uint32_t now); // Explicitly request only at the safe mechanical midpoint.
  JointFeedback joint(uint32_t now) const;
  float positionRate(uint32_t now) const; // Diagnostic only: 10+ ms position difference, rad/s.
  bool configured() const;
  bool torqueFrame(float joint_torque, Payload& frame) const;
  const MotorConfig& config() const { return config_; }
  const Feedback& raw() const { return feedback_; }
 protected:
  bool acceptFeedback(uint16_t id, const Feedback& feedback, uint32_t now);
 private:
  MotorConfig config_;
  Feedback feedback_{};
  float continuous_ = 0, zero_ = 0;
  float rate_anchor_ = 0, position_rate_ = 0;
  uint32_t rate_anchor_ms_ = 0;
  uint32_t last_ms_ = 0;
  bool received_ = false, zeroed_ = false;
};
}
