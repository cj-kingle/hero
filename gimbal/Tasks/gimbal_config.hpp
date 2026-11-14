#pragma once
#include "gimbal_task.hpp"
#include "gimbal_dual_axis.hpp"
#include "single_axis_config.hpp"
#include "imu_config.hpp"

namespace gimbal::config {
// Topology fixes control IDs/buses, NOT the feedback IDs or encoding ranges.
// Fill these from each actual motor; do not change the motor's saved parameters
// simply to match this file. No movement is permitted while unconfirmed.
inline constexpr damiao::MotorConfig kYaw{
    .control_id=3, .feedback_id=0xffff, .limits={}, .direction=0,
    .motor_per_joint=1, .feedback_wrap=6.283185307f, .confirmed=false};
inline constexpr damiao::MotorConfig kPitch{
    .control_id=9, .feedback_id=0xffff, .limits={}, .direction=0,
    .motor_per_joint=1, .feedback_wrap=6.283185307f, .confirmed=false};
// Body-IMU mounting matrix is configured in imu_task.cpp. Confirm before drive.
inline constexpr bool kImuMountingConfirmed = imu_config::kMountingConfirmed;
inline constexpr float kPitchStartupMotorDeg=-17.5f; // raw absolute motor angle after preparation
inline constexpr Config MakeControl() {
  const auto y=single_axis_config::MakeConfig(Axis::Yaw);
  const auto p=single_axis_config::MakeConfig(Axis::Pitch);
  Config c;
  c.yaw_angle=y.angle_pid; c.yaw_speed=y.speed_pid;
  c.pitch_angle=p.angle_pid; c.pitch_speed=p.speed_pid;
  c.max_yaw_rate=y.max_rate; c.max_pitch_rate=p.max_rate;
  c.pitch_neutral_rate_limit=single_axis_config::kPitchNeutralRateLimit;
  c.yaw_position_rate_limit=single_axis_config::kYawPositionRateLimit; // manual target speed remains y.max_rate
  c.yaw_neutral_rate_limit=single_axis_config::kYawNeutralRateLimit;
  c.yaw_manual_velocity_mode=true; // all modes: fixed rate while held, position hold after release
  c.yaw_manual_speed_ki=1.5f;c.yaw_manual_integral_limit=1.5f; // faster bounded load rejection while turning; release resets this integrator
  c.yaw_torque_limit=y.torque_limit; c.pitch_torque_limit=p.torque_limit;
  c.yaw_load_torque_limit=single_axis_config::kYawLoadTorqueLimit;
  c.yaw_load_build_rate=single_axis_config::kYawLoadTorqueBuildRate;
  c.pitch_min=p.pitch_min; c.pitch_max=p.pitch_max; c.pitch_slow_margin=p.limit_margin;
  c.pitch_target_margin=.5f*0.017453293f; // stop 0.5 motor/joint degrees inside each nominal limit
  c.pitch_limit_shutdown=false; // angle limit is a soft hold/recovery boundary, not a dual-axis shutdown
  c.pitch_limit_position_kp=80.f; // rad/s per rad penetration into the inner end margin
  c.pitch_limit_position_kd=4.f; // additional position-loop damping near the endpoint
  c.pitch_recovery_acceleration=40.f; // rad/s^2; neutral-stick position recovery only
  c.yaw_rate_acceleration=y.max_rate_acceleration; c.pitch_rate_acceleration=p.max_rate_acceleration;
  c.yaw_relative_rate_limit=0; // no IMU/motor-difference limiter may modulate a continuous turn
  c.yaw_recovery_acceleration=24.f; // rad/s^2, establish restoring effort sooner at neutral
  c.yaw_braking_acceleration=48.f; // rad/s^2; faster opposite-direction braking reference
  c.pitch_braking_acceleration=32.f; // rad/s^2; release stale outward references promptly
  c.yaw_friction=y.friction_torque; c.pitch_friction=p.friction_torque;
  c.pitch_reverse_friction=p.friction_torque_reverse; c.pitch_gravity=p.pitch_gravity_torque;
  c.yaw_friction_transition=y.friction_transition_rate;
  c.yaw_neutral_friction_error=.5f*0.017453293f; // fade neutral friction inside 0.5 deg error
  c.pitch_friction_transition=p.friction_transition_rate;
  c.pitch_start_at_configured_angle=true;
  c.pitch_start_angle=kPitchStartupMotorDeg*0.017453293f*
      single_axis_config::kPitchDirection/kPitch.motor_per_joint;
  c.pitch_imu_enabled=imu_config::kPitchImuEnabled;
  c.max_control_gap_ms=19; // CAN retry budget stays inside the independent 20 ms task watchdog
  c.stall_ms=1500; // require sustained blocked motion rather than a brief load transient
  c.yaw_stall_trip_enabled=false; // user disabled the yaw software-stall whole-gimbal stop; retain other protections
  c.require_imu_in_joint=false; // healthy motor feedback can hold while awaiting IMU
  c.allow_imu_entry_joint_hold=true; // encoder control is available while IMU mode is pending
  c.allow_imu_loss_joint_hold=true; // IMU loss selects encoder control without latching whole-gimbal off
  c.timing_trip_enabled=false; // no reason-256 latch; retain the independent foreground watchdog
  return c;
}
inline constexpr Config kControl=MakeControl();
inline constexpr DualAxisConfig MakeDual() {
  DualAxisConfig c; c.control=kControl;
  c.yaw.direction=single_axis_config::kYawDirection;
  c.pitch.direction=single_axis_config::kPitchDirection;
  c.yaw.absolute_position=c.pitch.absolute_position=true;
  c.yaw.continuous_absolute=true;
  c.yaw_overspeed=single_axis_config::MakeConfig(Axis::Yaw).overspeed;
  c.yaw_overspeed_confirm_ms=8;
  c.pitch_overspeed=single_axis_config::MakeConfig(Axis::Pitch).overspeed;
  c.overspeed_trip_enabled=false; // disable integrated software reason 4096 on both axes
  c.imu_mounting_confirmed=kImuMountingConfirmed;
  return c;
}
inline constexpr DualAxisConfig kDual=MakeDual();
}
