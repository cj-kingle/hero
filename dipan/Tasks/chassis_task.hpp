#pragma once
#include "remote_command.hpp"
#include "pid.hpp"
#include <limits>
namespace chassis {
struct Velocity { float forward = 0, left = 0, yaw_rate = 0; };
struct ChassisConfig {
  float max_speed = 0.5f;       // m/s; initial low-speed command limit
  float max_yaw_rate = 1.0f;    // rad/s
  float gyro_rate = 1.0f;
  float acceleration = 0.5f;   // m/s^2
  float yaw_acceleration = 1.0f;
  float gyro_acceleration=0,gyro_yaw_acceleration=0; // 0: use the common limit
  float follow_braking_acceleration=0; // rad/s^2; 0 preserves the ordinary ramp
  float follow_yaw_feedforward=0; // world heading rate per unit yaw stick, Follow only
  PidConfig follow_pid{2.f,0,0,1.f,0};
  float follow_stop_angle=0,follow_start_angle=0;
  float follow_deceleration=0; // follow reference deceleration; 0 reuses follow_braking_acceleration
  float follow_braking_delay=0; // seconds of measured closing motion reserved for actuator response
};
// High-level calculation only: no CAN frames, current commands or motor outputs.
class ChassisController {
 public:
  explicit ChassisController(ChassisConfig config = {}) : config_(config),follow_(config.follow_pid) {}
  Velocity update(const RemoteCommand& command, bool working,
                  float gimbal_yaw_relative_to_chassis, float dt,
                  float measured_yaw_rate=std::numeric_limits<float>::quiet_NaN());
  void reset();
 private:
  ChassisConfig config_;
  Pid follow_{{2.0f, 0, 0, 1.0f, 0}};
  Velocity previous_;
  float previous_angle_=0;
  Mode previous_mode_ = Mode::Depart;
  bool follow_active_=false;
};
}

#include <array>
#include <cstdint>

namespace chassis {
// X roller layout, +x forward, +y left, +yaw CCW viewed from above.
// Array order is CAN ID order: LF=1, LR=2, RR=3, RF=4.
enum class WheelIndex : unsigned { LeftFront, LeftRear, RightRear, RightFront };
// Measured on this vehicle: positive raw rotor rpm drives IDs 1/2 forward,
// and IDs 3/4 backward. Shared by single-wheel tests and inverse kinematics.
inline constexpr std::array<int8_t, 4> kWheelMotorDirections{{1, 1, -1, -1}};
struct MecanumConfig {
  float wheel_radius_m = 0.07786f;
  float wheelbase_m = 0.55f;    // full front-rear distance between wheel centers
  float track_width_m = 0.45f;  // full left-right distance between wheel centers
  // Course vehicle has a modified drivetrain: user confirmed TOTAL rotor/wheel=14.
  // Stock P19 manual's 3591/187 is not the ratio of this vehicle.
  float total_reduction = 14.0f;
  float max_motor_rpm = 300.0f; // proportional desaturation; 0 disables the limit
  // +1: positive rotor feedback drives this wheel forward; -1: backward.
  // 0 means UNKNOWN when supplying an uncalibrated configuration.
  std::array<int8_t, 4> motor_direction = kWheelMotorDirections;
};
struct WheelSpeedTargets {
  bool valid = false;               // finite inputs and valid geometry/configuration
  bool motor_targets_ready = false; // all 4 motor directions have been confirmed
  float scale = 1.0f;               // common speed-limit factor, in (0,1]
  std::array<float, 4> wheel_rpm{};  // positive = that wheel rolling chassis-forward
  std::array<float, 4> motor_rpm{};  // C620 rotor rpm; all zero if mapping is incomplete
};

// Pure inverse kinematics. FourWheelTest calls it; this function has no CAN/PID output.
// k=(wheelbase+track)/2. Wheel rim speeds [m/s] before limiting:
// LF=vx-vy-k*w, LR=vx+vy-k*w, RR=vx-vy+k*w, RF=vx+vy+k*w.
// wheel_rpm = rim_speed * 60/(2*pi*r).
// motor_rpm[i] = direction[i] * total_reduction * wheel_rpm[i].
// When any wheel exceeds max_motor_rpm, scale ALL wheels equally to preserve twist.
WheelSpeedTargets SolveMecanumX(const Velocity& body_velocity,
                                const MecanumConfig& config = {});
}
