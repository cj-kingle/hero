#include "dm4340_motor.hpp"
#include "dm4340_drv.hpp"
#include <algorithm>

namespace damiao {
bool DM4340::onFeedback(uint16_t id, const uint8_t* data, size_t size, uint32_t now) {
  if (!data || size != 8 || id != config().feedback_id || !ValidLimits(config().limits)) return false;
  Payload copy;
  std::copy_n(data, copy.size(), copy.begin()); // Legacy parser takes a mutable buffer.
  DM4340_Motor_t motor{};
  dm4340_joint_motor_init(&motor, config().control_id, DM4340_MIT_MODE);
  dm4340_fbdata(&motor, copy.data(), uint32_t(copy.size()));
  const auto& parsed = motor.para;
  // Use the driver's raw codes with PMAX/VMAX/TMAX read from this motor.
  // Its hard-coded float ranges must not override the discovered ranges.
  const auto decode = [](int code, float bound, unsigned bits) {
    return float(code) / float((1u << bits) - 1u) * (2 * bound) - bound;
  };
  Feedback feedback;
  feedback.id = uint8_t(parsed.id); feedback.state = uint8_t(parsed.state);
  feedback.position = decode(parsed.p_int, config().limits.position, 16);
  feedback.velocity = decode(parsed.v_int, config().limits.velocity, 12);
  feedback.torque = decode(parsed.t_int, config().limits.torque, 12);
  feedback.mos_temperature = uint8_t(parsed.Tmos);
  feedback.rotor_temperature = uint8_t(parsed.Tcoil);
  return acceptFeedback(id, feedback, now);
}
}
