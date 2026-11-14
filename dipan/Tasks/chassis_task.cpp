#include "chassis_task.hpp"
#include <cmath>
#include <algorithm>
namespace chassis {
namespace {
constexpr float pi = 3.14159265358979323846f;
float Ramp(float target, float old, float delta) {
  return old + std::clamp(target - old, -delta, delta);
}
}
void ChassisController::reset() { previous_ = {}; previous_angle_=0;follow_.reset();follow_active_=false; }
Velocity ChassisController::update(const RemoteCommand& command, bool working,
                                  float angle, float dt,float measured_yaw_rate) {
  if (!working || !command.online || command.emergency_stop || !std::isfinite(angle) ||
      !std::isfinite(config_.max_speed) || config_.max_speed <= 0 ||
      !std::isfinite(config_.max_yaw_rate) || config_.max_yaw_rate <= 0 ||
      !std::isfinite(config_.gyro_rate) || config_.gyro_rate == 0 ||
      !std::isfinite(config_.acceleration) || config_.acceleration <= 0 ||
      !std::isfinite(config_.yaw_acceleration) || config_.yaw_acceleration <= 0 ||
      !std::isfinite(config_.gyro_acceleration) || config_.gyro_acceleration<0 ||
      !std::isfinite(config_.gyro_yaw_acceleration) || config_.gyro_yaw_acceleration<0 ||
      !std::isfinite(config_.follow_braking_acceleration) || config_.follow_braking_acceleration<0 ||
      !std::isfinite(config_.follow_deceleration) || config_.follow_deceleration<0 ||
      !std::isfinite(config_.follow_braking_delay) || config_.follow_braking_delay<0 ||
      !std::isfinite(config_.follow_yaw_feedforward) || config_.follow_yaw_feedforward<0 ||
      !std::isfinite(config_.follow_stop_angle) || config_.follow_stop_angle<0 ||
      !std::isfinite(config_.follow_start_angle) || config_.follow_start_angle<config_.follow_stop_angle ||
      config_.follow_start_angle>=pi ||
      !std::isfinite(command.forward) || !std::isfinite(command.left) ||
      !std::isfinite(command.yaw) || !std::isfinite(dt) || dt <= 0 || dt >= 0.020f) {
    reset(); return {};
  }
  if (command.mode != previous_mode_) {follow_.reset();follow_active_=false;}
  previous_mode_ = command.mode;
  angle = std::remainder(angle, 2 * pi);
  const float norm = std::max(1.0f, std::hypot(command.forward, command.left));
  const float x = command.forward * config_.max_speed / norm;
  const float y = command.left * config_.max_speed / norm;
  // +x forward, +y left, +yaw counterclockwise. Input is in gimbal coordinates.
  Velocity target{std::cos(angle) * x - std::sin(angle) * y,
                  std::sin(angle) * x + std::cos(angle) * y, 0};
  switch (command.mode) {
    case Mode::Depart: target.yaw_rate = 0; break;
    case Mode::Follow:
      if(follow_active_ && std::fabs(angle)<=config_.follow_stop_angle) {follow_active_=false;follow_.reset();}
      if(!follow_active_ && std::fabs(angle)>=config_.follow_start_angle) follow_active_=true;
      target.yaw_rate=follow_active_ ? follow_.update(angle,0,dt) : 0;
      target.yaw_rate+=command.yaw*config_.follow_yaw_feedforward;
      if(config_.follow_braking_acceleration>0 && command.yaw==0) {
        // Brake to alignment after the heading target stops, not while the
        // operator is deliberately turning it. The rate/acceleration caps apply below.
        // v^2 <= 2*a*distance: start slowing before the alignment deadband.
        // The bound shapes the reference; actual stopping still depends on traction.
        // Reserve response travel from actual closing speed. No fixed brake
        // timer: slowing motion continuously releases this reserve, so the
        // position loop can finish alignment instead of waiting at zero.
        const float closing=std::isfinite(measured_yaw_rate) && angle*measured_yaw_rate>0 ?
            std::fabs(measured_yaw_rate) : 0.f;
        const float distance=std::max(0.f,std::fabs(angle)-config_.follow_stop_angle-
            closing*config_.follow_braking_delay);
        const float brake_rate=std::sqrt(2.f*config_.follow_braking_acceleration*distance);
        target.yaw_rate=std::clamp(target.yaw_rate,-brake_rate,brake_rate);
      }
      break;
    case Mode::Gyro: target.yaw_rate = config_.gyro_rate; break;
    default: reset(); return {};
  }
  const float yaw_limit=command.mode==Mode::Gyro ?
      std::fabs(config_.gyro_rate) : config_.max_yaw_rate;
  target.yaw_rate = std::clamp(target.yaw_rate, -yaw_limit, yaw_limit);
  const float acceleration=command.mode==Mode::Gyro && config_.gyro_acceleration>0 ?
      config_.gyro_acceleration : config_.acceleration;
  const float yaw_acceleration=command.mode==Mode::Gyro && config_.gyro_yaw_acceleration>0 ?
      config_.gyro_yaw_acceleration : config_.yaw_acceleration;
  if(command.mode==Mode::Gyro) {
    // Slew the operator's translation in the gimbal frame. Rotating the body
    // coordinates is a frame change, not a new joystick acceleration request.
    const float old_x=std::cos(previous_angle_)*previous_.forward+std::sin(previous_angle_)*previous_.left;
    const float old_y=-std::sin(previous_angle_)*previous_.forward+std::cos(previous_angle_)*previous_.left;
    const float dx=x-old_x,dy=y-old_y;
    const float length=std::hypot(dx,dy);
    const float fraction=length>acceleration*dt ? acceleration*dt/length : 1.f;
    const float ramp_x=old_x+fraction*dx,ramp_y=old_y+fraction*dy;
    target.forward=std::cos(angle)*ramp_x-std::sin(angle)*ramp_y;
    target.left=std::sin(angle)*ramp_x+std::cos(angle)*ramp_y;
  } else {
    target.forward = Ramp(target.forward, previous_.forward, acceleration * dt);
    target.left = Ramp(target.left, previous_.left, acceleration * dt);
  }
  float yaw_slew_acceleration=yaw_acceleration;
  const float follow_deceleration=config_.follow_deceleration>0 ?
      config_.follow_deceleration : config_.follow_braking_acceleration;
  if(command.mode==Mode::Follow && follow_deceleration>0 &&
      (target.yaw_rate*previous_.yaw_rate<0 ||
       std::fabs(target.yaw_rate)<std::fabs(previous_.yaw_rate)))
    yaw_slew_acceleration=std::max(yaw_acceleration,follow_deceleration);
  target.yaw_rate = Ramp(target.yaw_rate, previous_.yaw_rate, yaw_slew_acceleration * dt);
  previous_ = target;
  previous_angle_=angle;
  return target;
}
}

#include "chassis_task.hpp"
#include <algorithm>
#include <cmath>

namespace chassis {
WheelSpeedTargets SolveMecanumX(const Velocity& v, const MecanumConfig& c) {
  WheelSpeedTargets result;
  if (!std::isfinite(v.forward) || !std::isfinite(v.left) || !std::isfinite(v.yaw_rate) ||
      !std::isfinite(c.wheel_radius_m) || c.wheel_radius_m <= 0 ||
      !std::isfinite(c.wheelbase_m) || c.wheelbase_m <= 0 ||
      !std::isfinite(c.track_width_m) || c.track_width_m <= 0 ||
      !std::isfinite(c.total_reduction) || c.total_reduction <= 0 ||
      !std::isfinite(c.max_motor_rpm) || c.max_motor_rpm < 0) return result;
  bool directions_ready = true;
  for (int sign : c.motor_direction) {
    if (sign < -1 || sign > 1) return result;
    if (sign == 0) directions_ready = false;
  }
  constexpr float pi = 3.14159265358979323846f;
  const float k = (c.wheelbase_m + c.track_width_m) * 0.5f;
  const float rotation = k * v.yaw_rate;
  const float conversion = 60.0f / (2.0f * pi * c.wheel_radius_m);
  const std::array<float, 4> rim{{v.forward - v.left - rotation,
                                v.forward + v.left - rotation,
                                v.forward - v.left + rotation,
                                v.forward + v.left + rotation}};
  float peak_motor_rpm = 0;
  for (unsigned i = 0; i < 4; ++i) {
    result.wheel_rpm[i] = rim[i] * conversion;
    const float rotor = result.wheel_rpm[i] * c.total_reduction;
    if (!std::isfinite(result.wheel_rpm[i]) || !std::isfinite(rotor)) return {};
    peak_motor_rpm = std::max(peak_motor_rpm, std::fabs(rotor));
  }
  if (c.max_motor_rpm > 0 && peak_motor_rpm > c.max_motor_rpm)
    result.scale = c.max_motor_rpm / peak_motor_rpm;
  for (unsigned i = 0; i < 4; ++i) {
    result.wheel_rpm[i] *= result.scale;
    if (directions_ready)
      result.motor_rpm[i] = result.wheel_rpm[i] * c.total_reduction * c.motor_direction[i];
  }
  result.valid = true;
  result.motor_targets_ready = directions_ready;
  return result;
}
}
