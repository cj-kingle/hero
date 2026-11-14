#pragma once
#include "dm_motor.hpp"

namespace damiao {
// Adapter for the supplied, unmodified dm4340_drv.hpp/.cpp.
// Zero capture, angle unwrapping and checked MIT transmission stay shared.
class DM4340 : public Motor {
 public:
  explicit DM4340(MotorConfig config) : Motor(config) {}
  bool onFeedback(uint16_t id, const uint8_t* data, size_t size, uint32_t now);
};
}
