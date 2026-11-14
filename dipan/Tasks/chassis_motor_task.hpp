#pragma once
#include "c620_protocol.hpp"
#include "main_task.hpp"
#include "m3508.hpp"
#include "wheel_config.hpp"
#include "chassis_config.hpp"

namespace chassis {
// Four-wheel bench parameters. Rotor/current limits and PID gains reuse the
// individually tested values in wheel_config.hpp.
struct FourWheelConfig {
  float max_yaw_rate = 0.3f; // rad/s, commanded by right-stick horizontal axis
  MecanumConfig kinematics = [] {
    auto c=config::kWheels;c.max_motor_rpm=wheel_config::kMaxMotorRpm;return c;
  }(); // legacy bench limit; integrated mode selects its own limit
  // Full-stick pure forward/reverse reaches the same configured rotor-speed cap.
  float max_translation_mps = kinematics.max_motor_rpm *
      (2.0f * 3.14159265358979323846f * kinematics.wheel_radius_m) /
      (60.0f * kinematics.total_reduction);
};
struct FourWheelStatus {
  State state = State::Dead;
  uint32_t ready_count = 0, online_mask = 0;
  bool rc_online = false, transport_fault = false;
  Velocity velocity{}; // requested body twist before wheel-speed limiting/ramp
  float target_yaw_rate=0; // planner target before gyro feedback correction
  WheelSpeedTargets targets{}; // applied targets after common acceleration limiting
  std::array<int16_t, 4> commands{};
};
class FourWheelTest {
 public:
  explicit FourWheelTest(FourWheelConfig config = {}) : config_(config) {}
  FourWheelStatus update(const RemoteCommand& command,
      const std::array<C620Feedback, 4>& feedback, uint32_t now_ms, bool transport_fault);
  FourWheelStatus updateIntegrated(const RemoteCommand& command,
      const std::array<C620Feedback,4>& feedback,const BoardFeedback& board,
      uint32_t now_ms,bool transport_fault);
 private:
  FourWheelStatus updateCore(const RemoteCommand&,const std::array<C620Feedback,4>&,
      uint32_t,bool,const BoardFeedback*);
  void reset();
  FourWheelConfig config_;
  RobotStateMachine state_;
  ChassisController planner_{[] { auto c=config::kMotion;c.follow_pid=config::kFollowPid;return c; }()};
  Mode previous_mode_=Mode::Depart;
  Pid yaw_rate_pid_{config::kYawRatePid};
  Pid gyro_rate_pid_{config::kGyroYawRatePid};
  uint32_t previous_update_ms_=0;
  bool have_update_=false;
  std::array<float, 4> previous_targets_{};
  float previous_wheel_angle_=0;
  bool previous_wheel_gyro_=false;
  std::array<M3508, 4> motors_{{
      M3508{wheel_config::kWheelPids[0]}, M3508{wheel_config::kWheelPids[1]},
      M3508{wheel_config::kWheelPids[2]}, M3508{wheel_config::kWheelPids[3]}}};
};
extern FourWheelStatus four_wheel_status;
}
