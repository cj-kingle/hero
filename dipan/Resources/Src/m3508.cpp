#include "m3508.hpp"
#include <cmath>
namespace chassis {
void M3508::setFeedback(float rpm, float temperature, uint32_t now) {
  received_ = std::isfinite(rpm) && std::isfinite(temperature) &&
              temperature >= -20 && temperature < 125;
  if (!received_) { reset(); return; }
  motor_rpm_ = rpm; temperature_c_ = temperature; last_ms_ = now;
}
bool M3508::ready(uint32_t now) const {
  // Conservative initial software threshold; confirm with hardware before tuning.
  return received_ && uint32_t(now - last_ms_) < 100u && temperature_c_ < 70;
}
float M3508::calculate(float target, bool enabled, uint32_t now, float dt) {
  if (!enabled || !ready(now)) { reset(); return 0; }
  return speed_pid_.update(target, motor_rpm_, dt);
}
}
