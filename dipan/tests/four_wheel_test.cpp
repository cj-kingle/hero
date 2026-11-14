#include "chassis_motor_task.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
using namespace chassis;
struct Bench {
  FourWheelTest controller;
  RemoteCommand rc;
  std::array<C620Feedback, 4> feedback{};
  FourWheelStatus status;
  uint32_t now = 0;
  Bench() {
    rc.online = true; rc.emergency_stop = false; rc.centered = true;
    rc.mode = Mode::Depart;
  }
  void step(bool transport_fault = false, int missing = -1) {
    for (unsigned i = 0; i < 4; ++i) {
      feedback[i].received = int(i) != missing;
      feedback[i].updated_ms = now;
      feedback[i].temperature = 25;
    }
    status = controller.update(rc, feedback, now++, transport_fault);
  }
  void zero() const { for (auto c : status.commands) assert(c == 0); }
  void prepare() {
    for (unsigned i = 0; i < 2001; ++i) { step(); zero(); }
    assert(status.state == State::Working && status.online_mask == 15);
  }
  void settle() {
    const unsigned ticks = unsigned(std::ceil(2 * wheel_config::kMaxMotorRpm /
        wheel_config::kRampRpmPerSecond * 1000)) + 10;
    for (unsigned t = 0; t < ticks; ++t) {
      const auto previous = status.targets.motor_rpm;
      step();
      for (unsigned i = 0; i < 4; ++i) {
        assert(std::fabs(status.targets.motor_rpm[i]) <= wheel_config::kMaxMotorRpm + .01f);
        assert(std::fabs(status.targets.motor_rpm[i] - previous[i]) <=
            wheel_config::kRampRpmPerSecond * .001f + .001f);
        assert(std::abs(status.commands[i]) <= wheel_config::kCurrentLimitRaw);
      }
    }
  }
};
int main() {
  Bench b;
  b.prepare();
  b.rc.centered = false; b.rc.forward = 1; b.settle();
  for (unsigned i = 0; i < 4; ++i) {
    const int direction = i < 2 ? 1 : -1;
    assert(b.status.targets.wheel_rpm[i] > 0);
    assert(b.status.targets.motor_rpm[i] * direction > 0);
    assert(std::fabs(b.status.targets.motor_rpm[i] -
        direction * wheel_config::kMaxMotorRpm) < .01f);
    assert(b.status.commands[i] * direction > 0);
    // With stationary feedback and a full speed request, the loaded-start output
    // must reach the shared limit rather than remain capped by the old PID value.
    assert(std::abs(b.status.commands[i])==int(wheel_config::kCurrentLimitRaw));
    b.feedback[i].rpm = int16_t(std::lround(b.status.targets.motor_rpm[i]));
  }
  b.step();
  for (unsigned i = 0; i < 4; ++i) {
    assert(std::abs(b.status.commands[i]) <= wheel_config::kWheelPids[i].integral_limit +
           wheel_config::kWheelPids[i].kp * .51f + 1);
    b.feedback[i].rpm = 0;
  }
  b.rc.forward = -1; b.settle();
  for (unsigned i = 0; i < 4; ++i) {
    const int direction = i < 2 ? 1 : -1;
    assert(std::fabs(b.status.targets.motor_rpm[i] +
        direction * wheel_config::kMaxMotorRpm) < .01f);
  }
  b.rc.forward = 0; b.rc.left = 1; b.settle();
  const int strafe[] = {-1,1,1,-1};
  for (unsigned i = 0; i < 4; ++i)
    assert(b.status.targets.motor_rpm[i] * strafe[i] > 0);
  b.rc.left = 0; b.rc.yaw = 1; b.settle();
  for (auto target : b.status.targets.motor_rpm) assert(target < 0);
  b.rc.forward = 1; b.rc.left = 1; b.settle();
  assert(b.status.targets.scale < 1); // combined translation/rotation needs common limiting
  FourWheelConfig cfg;
  const auto expected = SolveMecanumX(b.status.velocity, cfg.kinematics);
  for (unsigned i = 0; i < 4; ++i)
    assert(std::fabs(b.status.targets.motor_rpm[i] - expected.motor_rpm[i]) < .01f);
  b.rc.forward = b.rc.left = b.rc.yaw = 0; b.rc.centered = true;
  b.step(); b.zero(); // neutral is zero effort, not active braking
  b.rc.forward = 1; b.rc.centered = false; b.settle();
  b.rc.emergency_stop = true; b.step(); b.zero(); assert(b.status.state == State::Dead);

  Bench missing; missing.prepare(); missing.rc.forward = 1; missing.rc.centered = false;
  missing.settle(); missing.step(false, 2); missing.zero();
  assert(missing.status.state == State::Preparing && missing.status.online_mask == 11);
  missing.step(); missing.zero(); assert(missing.status.ready_count == 0);

  Bench offline; offline.prepare(); offline.rc.forward = 1; offline.settle();
  offline.rc.online = false; offline.step(); offline.zero(); assert(offline.status.state == State::Dead);
  Bench bus; bus.prepare(); bus.rc.forward = 1; bus.settle();
  bus.step(true); bus.zero(); assert(bus.status.state == State::Fault);
  Bench switch_mode; switch_mode.prepare(); switch_mode.rc.mode = Mode::Gyro;
  switch_mode.rc.forward = 1; switch_mode.step(); switch_mode.zero();
  assert(switch_mode.status.state == State::Dead);
  Bench invalid; invalid.prepare(); invalid.rc.yaw = std::numeric_limits<float>::quiet_NaN();
  invalid.step(); invalid.zero(); assert(invalid.status.state == State::Fault);
  std::puts("PASS: four-wheel readiness, signed speed loops, translation/yaw, ramp, limiting, stop and dropout");
}
