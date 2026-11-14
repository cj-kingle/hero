#include "remote_command.hpp"
#include <cmath>
#include <algorithm>
namespace chassis {
namespace {
float Deadband(float x) {
  constexpr float deadband = 0.05f;
  if (!std::isfinite(x) || std::fabs(x) <= deadband) return 0;
  return std::copysign((std::min(std::fabs(x), 1.0f) - deadband) / (1 - deadband), x);
}
}
RemoteCommand ProcessRemote(const hello_world::devices::remote_control::DT7& rc,
                            bool received, uint32_t last_rx_ms, uint32_t now_ms) {
  RemoteCommand result;
  if (!received || uint32_t(now_ms - last_rx_ms) >= 100u) return result;
  using namespace hello_world::devices::remote_control;
  const auto l = rc.rc_l_switch(), r = rc.rc_r_switch();
  if ((l != kSwitchStateUp && l != kSwitchStateMid && l != kSwitchStateDown) ||
      (r != kSwitchStateUp && r != kSwitchStateMid && r != kSwitchStateDown)) return result;
  result.online = true;
  result.emergency_stop = l == kSwitchStateDown;
  result.mode = r == kSwitchStateUp ? Mode::Depart :
                r == kSwitchStateMid ? Mode::Follow : Mode::Gyro;
  result.forward = Deadband(rc.rc_lv());
  result.left = -Deadband(rc.rc_lh());
  result.yaw = -Deadband(rc.rc_rh());
  result.pitch = Deadband(rc.rc_rv());
  result.centered = result.forward == 0 && result.left == 0 &&
                    result.yaw == 0 && result.pitch == 0 && Deadband(rc.rc_wheel()) == 0;
  if (result.emergency_stop) result.forward = result.left = result.yaw = result.pitch = 0;
  return result;
}
}
