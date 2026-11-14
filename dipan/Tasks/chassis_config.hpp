#pragma once
#include "chassis_task.hpp"
#include "wheel_config.hpp"
namespace chassis::config {
inline constexpr bool kYawRateControlEnabled=true;
inline constexpr bool kImuLossTripEnabled=false; // use planner rate without IMU feedback while sensors are unavailable
inline constexpr bool kWheelOverspeedTripEnabled=false; // speed telemetry no longer latches integrated chassis off
inline constexpr uint32_t kMaxControlGapMs=19;
inline constexpr uint32_t kCanBusyGraceMs=19; // brief mailbox congestion; persistent TX failure still stops
// Optional alignment. Saved Flash calibration takes precedence; without one,
// this configured offset is usable immediately and never blocks preparation.
inline constexpr float kYawForwardOffsetRad=-40.56541434f * 0.01745329252f;
// Physical wheel rpm AFTER the 14:1 reduction. Integrated driving is tuned here;
// Shared wheel gains and gear ratio are in wheel_config.hpp.
inline constexpr float kTranslationWheelRpm=210.f; // full-stick Depart/Follow/Gyro translation
inline constexpr float kNormalMaxWheelRpm=700.f;
inline constexpr float kGyroMaxWheelRpm=700.f;
inline constexpr float kNormalMaxMotorRpm=kNormalMaxWheelRpm*wheel_config::kGearRatio;
inline constexpr float kGyroMaxMotorRpm=kGyroMaxWheelRpm*wheel_config::kGearRatio;
inline constexpr float kOverspeedMotorRpm=10000.f; // dormant integrated trip threshold; command cap is 9800 rotor rpm
// Mode-specific acceleration; wheel speed PID gains/current caps stay unchanged.
inline constexpr float kNormalCurrentLimitRaw=5000.f; // C620: about 6.10 A per motor
inline constexpr float kGyroCurrentLimitRaw=5000.f; // same 6.10 A budget as integrated normal mode
inline constexpr float kNormalRampRpmPerSecond=4000.f;
inline constexpr float kFollowBrakingRampRpmPerSecond=9000.f; // wheel braking keeps up with the faster follow reference deceleration
inline constexpr float kGyroRampRpmPerSecond=3200.f;
static_assert(kFollowBrakingRampRpmPerSecond>=kNormalRampRpmPerSecond);
// Independent chassis angular targets and rate-controller correction budgets.
// Upper yaw relative-speed protection must cover correction budget + world yaw.
inline constexpr float kFollowMaxYawRate=4.f;
inline constexpr float kGyroYawRate=3.1f;
inline constexpr float kFollowYawRateOutputLimit=4.f;
inline constexpr float kGyroYawRateOutputLimit=3.1f; // independent gyro inverse-kinematic yaw ceiling
static_assert(kNormalCurrentLimitRaw>0 && kNormalCurrentLimitRaw<=16384.f);
static_assert(kGyroCurrentLimitRaw>0 && kGyroCurrentLimitRaw<=16384.f);
inline constexpr MecanumConfig MakeWheels() {
  MecanumConfig c;
  c.max_motor_rpm=kNormalMaxMotorRpm;
  c.total_reduction=wheel_config::kGearRatio;
  return c;
}
inline constexpr MecanumConfig kWheels=MakeWheels();
inline constexpr ChassisConfig MakeMotion() {
  ChassisConfig c;
  c.max_speed=kTranslationWheelRpm *
      (2.f*3.14159265358979323846f*kWheels.wheel_radius_m)/60.f;
  c.max_yaw_rate=kFollowMaxYawRate; c.gyro_rate=kGyroYawRate;
  c.acceleration=1.8f; c.yaw_acceleration=4.f;
  c.gyro_acceleration=1.4f; c.gyro_yaw_acceleration=1.8f;
  c.follow_braking_acceleration=3.6f; // extend arrival slowdown distance while retaining the stronger braking ramp
  c.follow_deceleration=6.0f; // gentler follow reference braking, independent of the 4 rad/s ceiling
  c.follow_braking_delay=.06f; // reserve 60 ms of actual closing travel before arrival
  c.follow_yaw_feedforward=2.f; // match upper's current full-stick world-yaw target rate [rad/s]
  c.follow_stop_angle=.3f*0.01745329252f;
  c.follow_start_angle=.6f*0.01745329252f;
  return c;
}
inline constexpr ChassisConfig kMotion=MakeMotion();
inline constexpr PidConfig kFollowPid{8.f,0,0,kFollowMaxYawRate,0};
// Output is the desired kinematic yaw rate [rad/s], with target-rate feedforward.
// Initial trial gains; tune against measured chassis yaw rate after mounting checks.
inline constexpr PidConfig kYawRatePid{.5f,.2f,0,kFollowYawRateOutputLimit,.05f};
inline constexpr PidConfig kGyroYawRatePid{.5f,.2f,0,kGyroYawRateOutputLimit,.05f};
static_assert(kFollowYawRateOutputLimit>=kFollowMaxYawRate);
static_assert(kGyroYawRateOutputLimit>=kGyroYawRate);
static_assert(kTranslationWheelRpm>0 && kTranslationWheelRpm<=kNormalMaxWheelRpm);
static_assert(kNormalMaxMotorRpm>0 && kNormalMaxMotorRpm<kOverspeedMotorRpm);
static_assert(kGyroMaxMotorRpm>0 && kGyroMaxMotorRpm<kOverspeedMotorRpm);
inline constexpr float kZeroYawRateDeadband=.01f; // rad/s, near-zero gyro noise when target rate is zero
}
