#include "pid.hpp"
#include <cmath>
#include <algorithm>
namespace gimbal {
bool ValidPidErrorShaping(const PidConfig& c) {
  return std::isfinite(c.small_error_kp) && c.small_error_kp>=0 &&
      std::isfinite(c.small_error_range) && c.small_error_range>=0 &&
      std::isfinite(c.full_gain_error) && c.full_gain_error>=0 &&
      std::isfinite(c.error_deadband) && c.error_deadband>=0 &&
      std::isfinite(c.deadband_transition) && c.deadband_transition>=0 &&
      std::isfinite(c.derivative_deadband_rate) && c.derivative_deadband_rate>=0 &&
      (c.derivative_deadband_rate==0 || (c.derivative_from_rate && c.error_deadband>0)) &&
      (c.error_deadband==0 || c.deadband_transition>0) &&
      (!c.variable_kp || (c.small_error_kp<=c.kp &&
       c.full_gain_error>c.small_error_range &&
       c.small_error_range>=c.error_deadband+c.deadband_transition)) &&
      std::isfinite(c.small_error_kd) && c.small_error_kd>=0 &&
      std::isfinite(c.small_d_error_range) && c.small_d_error_range>=0 &&
      std::isfinite(c.full_d_error) && c.full_d_error>=0 &&
      (!c.variable_kd || (c.small_error_kd<=c.kd && c.full_d_error>c.small_d_error_range));
}
void Pid::reset() { integral_ = previous_measurement_ = filtered_derivative_ = 0; initialized_ = false; terms_={}; }
float Pid::update(float target, float measurement, float dt, float feedforward,
    float measurement_rate, bool target_stationary, float target_rate) {
  if (!std::isfinite(target) || !std::isfinite(measurement) ||
      !std::isfinite(dt) || dt <= 0 || dt > 0.1f || !std::isfinite(feedforward) ||
      !std::isfinite(config_.kp) || !std::isfinite(config_.ki) || !ValidPidErrorShaping(config_) ||
      !std::isfinite(config_.kd) || !std::isfinite(config_.integral_limit) ||
      config_.integral_limit < 0 ||
      !std::isfinite(config_.measurement_filter_tau) || config_.measurement_filter_tau<0 ||
      !std::isfinite(config_.derivative_filter_tau) || config_.derivative_filter_tau<0 ||
      !std::isfinite(config_.approach_braking_acceleration) || config_.approach_braking_acceleration<0 ||
      (config_.approach_braking_acceleration>0 && !config_.derivative_from_rate) ||
      (config_.derivative_from_rate && !std::isfinite(measurement_rate)) ||
      !std::isfinite(target_rate) || (target_rate!=0 && !config_.derivative_from_rate) ||
      !std::isfinite(config_.output_limit) || config_.output_limit <= 0) {
    reset(); return 0;
  }
  // First sample starts at the current measurement, avoiding a synthetic zero
  // on enable/reset. dt is measured in seconds; tau=0 preserves the old path.
  const float used=initialized_ && config_.measurement_filter_tau>0 ?
      previous_measurement_ + dt/(config_.measurement_filter_tau+dt)*(measurement-previous_measurement_) :
      measurement;
  if(!std::isfinite(used)) { reset(); return 0; }
  const float error = target - used;
  const float abs_error=std::fabs(error);
  const auto smooth=[](float x) {x=std::clamp(x,0.f,1.f);return x*x*(3.f-2.f*x);};
  const float effective_kp=config_.variable_kp ?
      config_.small_error_kp+(config_.kp-config_.small_error_kp)*
          smooth((abs_error-config_.small_error_range)/(config_.full_gain_error-config_.small_error_range)) : config_.kp;
  // Quiet-band shaping is only for stationary holding. A slowly moving target
  // must retain continuous P/I effort or it advances through the band and
  // produces stick-slip steps. Feedforward remains independent.
  const float error_blend=target_stationary && config_.error_deadband>0 ?
      smooth((abs_error-config_.error_deadband)/config_.deadband_transition) : 1.f;
  const float control_error=error*error_blend;
  const float proportional=effective_kp*control_error;
  // Keep full static/away-direction stiffness. Only when the measured motion
  // is returning toward the target, reduce its drive before it reaches zero
  // error. Restrict only the toward-target reference, not opposite-direction
  // D braking; apply these bounds inside the anti-windup decision as well.
  float lower_limit=-config_.output_limit,upper_limit=config_.output_limit;
  if(target_stationary && config_.approach_braking_acceleration>0 && std::fabs(measurement_rate)>.02f &&
      error*measurement_rate>0) {
    const float approach_limit=std::sqrt(2.f*config_.approach_braking_acceleration*std::fabs(error));
    if(error>0)upper_limit=std::min(upper_limit,approach_limit);
    else lower_limit=std::max(lower_limit,-approach_limit);
  }
  // Use an explicitly supplied target rate for moving-target damping. Never
  // differentiate a target step; the default zero keeps measurement damping.
  const float derivative = config_.derivative_from_rate ? target_rate-measurement_rate :
      (initialized_ ? -(used - previous_measurement_) / dt : 0);
  const float damping=initialized_ && config_.derivative_filter_tau>0 ?
      filtered_derivative_+dt/(config_.derivative_filter_tau+dt)*(derivative-filtered_derivative_) : derivative;
  if(!std::isfinite(damping)) { reset(); return 0; }
  filtered_derivative_=damping;
  // Only quiet neutral holding may fade D. Keep the filter running, and use
  // both raw and filtered rate so motion restores damping without waiting for
  // the low-pass and braking does not disappear while the filter settles.
  float damping_blend=1.f;
  if(target_stationary && config_.derivative_deadband_rate>0) {
    const float rate=std::max(std::fabs(measurement_rate),std::fabs(damping));
    const float full_rate=config_.derivative_deadband_rate;
    const float motion_blend=smooth((rate-.25f*full_rate)/(.75f*full_rate));
    damping_blend=std::max(error_blend,motion_blend);
  }
  // Keep the velocity filter running even when small-error D is zero. Smooth
  // the gain independently of P so crossing the band does not switch D abruptly.
  const float effective_kd=config_.variable_kd ?
      config_.small_error_kd+(config_.kd-config_.small_error_kd)*
          smooth((abs_error-config_.small_d_error_range)/(config_.full_d_error-config_.small_d_error_range)) : config_.kd;
  const float derivative_output=effective_kd*damping*damping_blend;
  previous_measurement_ = used; initialized_ = true;
  const float limit = std::fabs(config_.integral_limit);
  const float candidate = std::clamp(integral_ + config_.ki * control_error * dt, -limit, limit);
  // Include feedforward in the saturation test, so adding it cannot silently
  // wind up the integrator against the final actuator limit.
  const float raw = proportional + candidate + derivative_output + feedforward;
  if (!std::isfinite(raw)) { reset(); return 0; }
  if ((raw <= upper_limit && raw >= lower_limit) ||
      (raw > upper_limit && error < 0) ||
      (raw < lower_limit && error > 0)) integral_ = candidate;
  terms_.p=proportional; terms_.i=integral_; terms_.d=derivative_output;
  terms_.effective_kp=effective_kp;terms_.control_error=control_error;
  terms_.feedforward=feedforward;
  terms_.measurement=used;
  terms_.output=std::clamp(proportional + integral_ + derivative_output + feedforward,
                    lower_limit, upper_limit);
  return terms_.output;
}
}
