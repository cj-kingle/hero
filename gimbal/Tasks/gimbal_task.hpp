#pragma once
#include "board_link.hpp"
#include "dm_motor.hpp"
#include "pid.hpp"
#include <cstdint>

namespace gimbal {
enum class State : uint8_t { Dead, Preparing, Working };
enum Reason : uint32_t {
  NoCommand=1, EmergencyStop=2, Configuration=4, MotorFeedback=8,
  ImuUnavailable=16, StartupGuard=32, PitchLimit=64, Transport=128,
  Timing=256, Stall=512, NeutralRequired=1024, EnablePending=2048, Overspeed=4096, ImuModePending=8192
};
struct ImuFeedback {
  float yaw = 0, pitch = 0, yaw_rate = 0, pitch_rate = 0;
  bool ready = false;
};
struct Config {
  float pitch_min = -.087266463f, pitch_max = .087266463f; // +/- 5 deg
  float pitch_slow_margin = .017453293f; // soft approach distance
  float pitch_target_margin=0; // park inside the hard bounds, in joint radians
  bool pitch_limit_shutdown=true; // false: angle endpoints constrain motion without disabling the healthy servo
  float pitch_limit_position_kp=0,pitch_limit_position_kd=0; // extra POSITION feedback in the end margin
  float pitch_recovery_acceleration=0; // neutral-stick disturbance recovery; 0 uses normal ramp
  float pitch_braking_acceleration=0; // optional faster deceleration and immediate direction release
  float pitch_neutral_rate_limit=0; // neutral restoration reference; 0 keeps the original travel-speed bound
  float max_yaw_rate = .3f, max_pitch_rate = .1f; // rad/s, initial bench ceilings
  float yaw_torque_limit = .3f, pitch_torque_limit = .5f; // joint N*m
  PidConfig yaw_angle{}, yaw_speed{}, pitch_angle{}, pitch_speed{};
  bool require_imu_in_joint = true;
  bool allow_imu_entry_joint_hold=false; // await initial IMU readiness without disabling a healthy joint loop
  bool allow_imu_loss_joint_hold=false; // encoder hold on runtime IMU loss instead of reason 16
  bool timing_trip_enabled=true; // optional software gap trip; runtime watchdog remains independent
  uint32_t startup_guard_ms = 2000, ready_ms = 2000, stall_ms = 500;
  bool yaw_stall_trip_enabled=true; // optional software stationary-load trip; motor health/overspeed are independent
  float yaw_rate_acceleration=0,pitch_rate_acceleration=0;
  float yaw_recovery_acceleration=0; // neutral yaw stick: 0 uses the ordinary motion ramp
  float yaw_relative_rate_limit=0; // IMU outer-loop output budget relative to the rotating chassis; 0 disables
  float yaw_position_rate_limit=0; // 0 uses max_yaw_rate; otherwise a separate pose-correction ceiling
  bool yaw_manual_rate_tracking=false; // feed forward commanded rate and damp its tracking error while turning
  bool yaw_manual_velocity_mode=false; // stick held: velocity PI; neutral: position hold
  float yaw_manual_speed_ki=0,yaw_manual_integral_limit=0; // manual-only load rejection; speed P/D are shared
  float yaw_neutral_rate_limit=0; // optional neutral-only correction ceiling; 0 uses yaw_position_rate_limit
  float yaw_braking_acceleration=0; // 0: old ramp; positive: faster deceleration and immediate sign release
  float yaw_friction=0,pitch_friction=0,pitch_reverse_friction=-1;
  float yaw_friction_transition=.03f,pitch_friction_transition=.03f,pitch_gravity=0;
  float yaw_neutral_friction_error=0; // rad; 0 preserves full rate-based friction at neutral
  float yaw_load_torque_limit=0,yaw_load_build_rate=0; // optional slow neutral load bias [N*m], [N*m/s]; 0 limit disables
  bool pitch_start_at_configured_angle=false;
  float pitch_start_angle=0; // joint radians, used only when the configured-start switch is enabled
  bool pitch_imu_enabled=true; // false: IMU yaw stabilization + motor-coordinate pitch loop
  uint32_t max_control_gap_ms=1;
};
struct ControlDiagnostics {
  PidTerms yaw_angle{},yaw_speed{},pitch_angle{},pitch_speed{};
  float yaw_rate=0,pitch_rate=0;
  float yaw_load_torque=0;
};
struct Input {
  Command command{};
  damiao::JointFeedback yaw{}, pitch{};
  ImuFeedback imu{};
  bool configuration_ready = false, transport_ok = true;
};
struct Output {
  State state = State::Dead;
  uint32_t ready_count = 0, reasons = NoCommand;
  bool drive = false;
  float yaw_torque = 0, pitch_torque = 0;
  float yaw_target = 0, pitch_target = 0;
  Mode active_mode=Mode::Joint; // actual control frame, distinct from requested mode
};
// Pure controller: no HAL, CAN, sleeping, automatic homing or watchdog feeding.
class Controller {
 public:
  explicit Controller(Config config = {});
  Output update(const Input&, uint32_t now);
  void stop();
  State state() const { return state_; }
  ControlDiagnostics diagnostics() const {
    return {yaw_angle_.terms(),yaw_manual_active_ ? yaw_manual_speed_.terms() : yaw_speed_.terms(),pitch_angle_.terms(),pitch_speed_.terms(),yaw_ramped_,pitch_ramped_,yaw_load_torque_};
  }
 private:
  void resetPids();
  void captureTargets(const Input&,bool preserve_pitch=false);
  Config config_;
  Pid yaw_angle_, yaw_speed_, pitch_angle_, pitch_speed_;
  Pid yaw_manual_speed_;
  bool yaw_manual_active_=false,pitch_manual_active_=false;
  State state_ = State::Dead;
  Mode mode_ = Mode::Joint;
  uint32_t boot_ms_ = 0, last_ms_ = 0, ready_count_ = 0, stall_count_ = 0;
  float yaw_target_ = 0, pitch_target_ = 0;
  bool clock_started_ = false;
  float yaw_ramped_=0,pitch_ramped_=0;
  float chassis_yaw_rate_estimate_=0;
  bool chassis_yaw_rate_seen_=false;
  float yaw_load_bias_=0,yaw_load_torque_=0;
  uint32_t yaw_load_dwell_ms_=0;
};
}
