#pragma once
#include "dm_motor.hpp"
namespace damiao { class DM4310 : public Motor {
 public: explicit DM4310(MotorConfig c) : Motor(c) {}
}; }
