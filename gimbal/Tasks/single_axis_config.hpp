#pragma once
#include "single_axis_test.hpp"
namespace gimbal::single_axis_config {
inline constexpr bool kEnabled=false; // false: integrated dual-axis upper-board entry
// Change only with motor power removed, then rebuild: Yaw=CAN1/3, Pitch=CAN2/9.
inline constexpr Axis kSelectedAxis=Axis::Pitch;
// Keep joint-positive pitch as nose up, opposite to the raw motor angle.
inline constexpr int kYawDirection=1;
inline constexpr int kPitchDirection=-1;
inline constexpr int kDirection=kSelectedAxis==Axis::Yaw ? kYawDirection : kPitchDirection;
inline constexpr float kFeedbackWrap=6.283185307f;
inline constexpr float kYawPositionRateLimit=3.0f; // pose-correction ceiling, independent of manual target speed
inline constexpr float kYawNeutralRateLimit=9.0f; // neutral-stick anti-disturbance correction only; manual target remains 2 rad/s
inline constexpr float kPitchNeutralRateLimit=6.0f; // neutral pitch restoration only; manual target remains 0.85 rad/s
inline constexpr float kYawLoadTorqueLimit=0.f; // disabled after added yaw vibration; 0 bypasses the load-bias path
inline constexpr float kYawLoadTorqueBuildRate=1.5f; // N*m/s, after 150 ms of persistent low-speed error
inline constexpr SingleAxisConfig MakeConfig(Axis axis) {
  SingleAxisConfig c; c.axis=axis;
  if(axis==Axis::Pitch) {
    c.mode=SingleAxisMode::Position; // pitch enters position control with a zero-angle target
    c.max_rate_acceleration=3.0f; // rad/s^2; pitch motion reference acceleration
    // Limits around the motor's absolute horizontal zero; no software zero capture.
    // Raw motor coordinates supplied by the user: nose up -15 deg, nose down +4.5 deg.
    constexpr float motor_min=-25.0f*0.017453293f;
    constexpr float motor_max=  2.5f*0.017453293f;
    c.pitch_min=kPitchDirection==1 ? motor_min : -motor_max;
    c.pitch_max=kPitchDirection==1 ? motor_max : -motor_min;
    c.limit_margin=5.0f*0.017453293f; // begin slowing 5 motor degrees before a limit
    c.max_rate=0.85f; c.torque_limit=28.0f; c.overspeed=4.0f;
    c.friction_torque=0.6f; // positive target rate: nose-up friction compensation [N*m]
    c.friction_torque_reverse=1.6f; // negative target rate: nose-down magnitude, subtracted from gravity bias
    c.pitch_gravity_torque=7.1f; // Positive joint torque lifts; kPitchDirection converts it to negative motor torque.
    c.angle_pid={300.0f,0,13.50f,kPitchNeutralRateLimit>c.max_rate ? kPitchNeutralRateLimit : c.max_rate,0}; // pitch P trial: reduce position gain; retain damping and the neutral budget
    c.angle_pid.variable_kp=false; // fixed Kp: no small/large-error gain scheduling
    c.angle_pid.error_deadband=.10f*0.017453293f;
    c.angle_pid.deadband_transition=.04f*0.017453293f;
    c.angle_pid.measurement_filter_tau=0; // P/I use current position without the former 12 ms lag
    c.angle_pid.derivative_from_rate=true;
    c.angle_pid.derivative_filter_tau=.010f; // motor velocity damping; filter D only
    c.angle_pid.derivative_deadband_rate=.015f; // restore full D while approaching; fade only below 0.00375 rad/s near neutral
    c.angle_pid.approach_braking_acceleration=6.f; // shape return motion only; stationary/away push retains full P
    c.speed_pid={4.0f,0.05f,0,c.torque_limit,1.0f}; // P-only; gravity/friction feedforward is added separately
    c.speed_pid.measurement_filter_tau=.003f;
  } else {
    // Yaw bench settings: joint speed [rad/s] and joint torque [N*m].
    c.mode=SingleAxisMode::Position; // cascaded position + velocity control; Velocity bypasses outer loop
    c.max_rate_acceleration=2.0f; // rad/s^2, limits startup/reversal speed-reference changes
    c.max_rate=2.00f;
    c.torque_limit=7.50f; // user-confirmed allowable torque; commanded speed limits are unchanged
    c.overspeed=14.00f; // measured relative-speed protection; commanded motion speeds stay unchanged
    // Fixed friction feedforward magnitude, joint N*m. Tune this first;
    // 0 disables it. Sign follows target velocity, blending to zero near rest.
    c.friction_torque=.12f;
    c.friction_transition_rate=.03f; // rad/s, zero-crossing transition width
    // Sole tuning location: Kp, Ki, Kd, output limit, integral limit.
    c.angle_pid={675.0f, 0.0f, 16.00f, kYawNeutralRateLimit, 0.0f}; // integrated manual correction keeps its separate 3 rad/s bound
    c.angle_pid.variable_kp=true;
    c.angle_pid.small_error_kp=70.f; // yaw near-target P trial: reduce stationary correction without lowering large-error gain
    c.angle_pid.small_error_range=.3f*0.017453293f;
    c.angle_pid.full_gain_error=1.2f*0.017453293f;
    c.angle_pid.variable_kd=true;
    c.angle_pid.small_error_kd=5.0f; // independent yaw damping knob near target; large-error Kd stays 16
    c.angle_pid.small_d_error_range=.3f*0.017453293f;
    c.angle_pid.full_d_error=1.2f*0.017453293f;
    c.angle_pid.error_deadband=.07f*0.017453293f; // yaw neutral quiet band, degrees converted to radians
    c.angle_pid.deadband_transition=.03f*0.017453293f;
    c.angle_pid.measurement_filter_tau=0; // P/I use current position without the former 15 ms lag
    c.angle_pid.derivative_from_rate=true;
    c.angle_pid.derivative_filter_tau=.006f; // same-frame yaw rate, less delay before damping acts
    c.angle_pid.approach_braking_acceleration=3.f; // earlier return braking, full P at rest or when pushed away
    c.speed_pid={0.75f, 0.0f, 0.0f, c.torque_limit, 0.05f}; // stronger effort at the same speed-reference ceiling
    // Optional velocity feedback low-pass, seconds. 0 restores raw feedback.
    c.speed_pid.measurement_filter_tau=.003f;
  }
  return c;
}
inline constexpr SingleAxisConfig MakeConfig() { return MakeConfig(kSelectedAxis); }
inline constexpr SingleAxisConfig kControl=MakeConfig();
}
