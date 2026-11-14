#include "gimbal_task.hpp"
#include <algorithm>
#include <cmath>
namespace gimbal {
namespace {
bool ValidPid(const PidConfig& c) {
  return std::isfinite(c.kp) && c.kp > 0 && std::isfinite(c.ki) && c.ki >= 0 &&
      std::isfinite(c.kd) && c.kd >= 0 && std::isfinite(c.output_limit) &&
      c.output_limit > 0 && std::isfinite(c.integral_limit) && c.integral_limit >= 0 &&
      std::isfinite(c.measurement_filter_tau) && c.measurement_filter_tau>=0 &&
      std::isfinite(c.derivative_filter_tau) && c.derivative_filter_tau>=0 &&
      std::isfinite(c.approach_braking_acceleration) && c.approach_braking_acceleration>=0 &&
      (c.approach_braking_acceleration==0 || c.derivative_from_rate) && ValidPidErrorShaping(c);
}
bool Valid(const Config& c) {
  const float reverse=c.pitch_reverse_friction==-1 ? c.pitch_friction : c.pitch_reverse_friction;
  return std::isfinite(c.pitch_min) && std::isfinite(c.pitch_max) && c.pitch_min < 0 &&
      c.pitch_max > 0 && std::isfinite(c.pitch_slow_margin) && c.pitch_slow_margin > 0 &&
      c.pitch_slow_margin < (c.pitch_max-c.pitch_min)*.5f &&
      std::isfinite(c.pitch_target_margin) && c.pitch_target_margin>=0 &&
      c.pitch_target_margin<c.pitch_slow_margin &&
      std::isfinite(c.pitch_limit_position_kp) && c.pitch_limit_position_kp>=0 &&
      std::isfinite(c.pitch_limit_position_kd) && c.pitch_limit_position_kd>=0 &&
      c.pitch_min+c.pitch_target_margin<0 && c.pitch_max-c.pitch_target_margin>0 &&
      std::isfinite(c.pitch_start_angle) &&
      (!c.pitch_start_at_configured_angle ||
       (c.pitch_start_angle>=c.pitch_min+c.pitch_target_margin &&
        c.pitch_start_angle<=c.pitch_max-c.pitch_target_margin)) &&
      std::isfinite(c.pitch_recovery_acceleration) && c.pitch_recovery_acceleration>=0 &&
      std::isfinite(c.pitch_braking_acceleration) && c.pitch_braking_acceleration>=0 &&
      std::isfinite(c.max_yaw_rate) && c.max_yaw_rate > 0 &&
      std::isfinite(c.max_pitch_rate) && c.max_pitch_rate > 0 &&
      std::isfinite(c.pitch_neutral_rate_limit) && c.pitch_neutral_rate_limit>=0 &&
      (c.pitch_neutral_rate_limit==0 || c.pitch_neutral_rate_limit<=c.pitch_angle.output_limit) &&
      std::isfinite(c.yaw_torque_limit) && c.yaw_torque_limit > 0 &&
      std::isfinite(c.yaw_load_torque_limit) && c.yaw_load_torque_limit>=0 && c.yaw_load_torque_limit<=c.yaw_torque_limit &&
      std::isfinite(c.yaw_load_build_rate) && c.yaw_load_build_rate>=0 &&
      (c.yaw_load_torque_limit==0 || c.yaw_load_build_rate>0) &&
      std::isfinite(c.pitch_torque_limit) && c.pitch_torque_limit > 0 &&
      c.startup_guard_ms >= 2000 && c.ready_ms >= 2000 && c.stall_ms > 0 &&
      c.max_control_gap_ms>=1 && c.max_control_gap_ms<20 &&
      ValidPid(c.yaw_angle) && ValidPid(c.yaw_speed) &&
      ValidPid(c.pitch_angle) && ValidPid(c.pitch_speed) &&
      std::isfinite(c.yaw_rate_acceleration) && c.yaw_rate_acceleration>=0 &&
      std::isfinite(c.yaw_relative_rate_limit) && c.yaw_relative_rate_limit>=0 &&
      std::isfinite(c.yaw_position_rate_limit) && c.yaw_position_rate_limit>=0 &&
      (!c.yaw_manual_rate_tracking || c.yaw_angle.derivative_from_rate) &&
      std::isfinite(c.yaw_manual_speed_ki) && c.yaw_manual_speed_ki>=0 &&
      std::isfinite(c.yaw_manual_integral_limit) && c.yaw_manual_integral_limit>=0 &&
      c.yaw_manual_integral_limit<=c.yaw_torque_limit &&
      std::isfinite(c.yaw_neutral_rate_limit) && c.yaw_neutral_rate_limit>=0 &&
      (c.yaw_neutral_rate_limit==0 || c.yaw_neutral_rate_limit<=c.yaw_angle.output_limit) &&
      std::isfinite(c.yaw_recovery_acceleration) && c.yaw_recovery_acceleration>=0 &&
      std::isfinite(c.yaw_braking_acceleration) && c.yaw_braking_acceleration>=0 &&
      std::isfinite(c.pitch_rate_acceleration) && c.pitch_rate_acceleration>=0 &&
      std::isfinite(c.yaw_friction) && c.yaw_friction>=0 && c.yaw_friction<=c.yaw_torque_limit &&
      std::isfinite(c.pitch_friction) && c.pitch_friction>=0 && c.pitch_friction<=c.pitch_torque_limit &&
      std::isfinite(reverse) && reverse>=0 && reverse<=c.pitch_torque_limit &&
      std::isfinite(c.pitch_gravity) && std::fabs(c.pitch_gravity)<=c.pitch_torque_limit &&
      std::isfinite(c.yaw_friction_transition) && c.yaw_friction_transition>=0 &&
      std::isfinite(c.yaw_neutral_friction_error) && c.yaw_neutral_friction_error>=0 &&
      std::isfinite(c.pitch_friction_transition) && c.pitch_friction_transition>=0 &&
      (c.yaw_friction==0 || c.yaw_friction_transition>0) &&
      ((c.pitch_friction==0 && reverse==0) || c.pitch_friction_transition>0);
}
bool FiniteJoint(const damiao::JointFeedback& j) {
  return j.online && j.healthy && j.zeroed && std::isfinite(j.position) &&
      std::isfinite(j.velocity) && std::isfinite(j.torque);
}
bool FiniteImu(const ImuFeedback& i,bool pitch) {
  return i.ready && std::isfinite(i.yaw) && std::isfinite(i.yaw_rate) &&
      (!pitch || (std::isfinite(i.pitch) && std::isfinite(i.pitch_rate)));
}
}
namespace {
PidConfig ManualYawSpeedConfig(const Config& c) {
  auto speed=c.yaw_speed;speed.ki=c.yaw_manual_speed_ki;
  speed.integral_limit=c.yaw_manual_integral_limit;return speed;
}
}
Controller::Controller(Config c) : config_(c), yaw_angle_(c.yaw_angle), yaw_speed_(c.yaw_speed),
    pitch_angle_(c.pitch_angle), pitch_speed_(c.pitch_speed),yaw_manual_speed_(ManualYawSpeedConfig(c)) {}
void Controller::resetPids() {
  yaw_angle_.reset(); yaw_speed_.reset(); pitch_angle_.reset(); pitch_speed_.reset();
  yaw_manual_speed_.reset();yaw_manual_active_=pitch_manual_active_=false;
  stall_count_ = 0;
  yaw_ramped_=pitch_ramped_=0;
  chassis_yaw_rate_estimate_=0;chassis_yaw_rate_seen_=false;
  yaw_load_bias_=yaw_load_torque_=0;yaw_load_dwell_ms_=0;
}
void Controller::stop() { state_ = State::Dead; ready_count_ = 0; resetPids(); }
void Controller::captureTargets(const Input& in,bool preserve_pitch) {
  mode_ = in.command.mode;
  yaw_target_ = mode_ == Mode::Joint ? in.yaw.position : in.imu.yaw;
  if(!preserve_pitch)
    pitch_target_ = mode_ == Mode::Joint || !config_.pitch_imu_enabled ?
        (config_.pitch_start_at_configured_angle ? config_.pitch_start_angle : in.pitch.position) : in.imu.pitch;
  resetPids();
}
Output Controller::update(const Input& in, uint32_t now) {
  Output out;
  const uint32_t elapsed=clock_started_ ? uint32_t(now-last_ms_) : 1u;
  const bool consecutive = clock_started_ && elapsed>=1u && elapsed<=config_.max_control_gap_ms;
  if (!clock_started_) { boot_ms_ = now; clock_started_ = true; }
  last_ms_ = now;
  auto cmd=in.command;
  // An initial request for IMU mode is not proof that the IMU is ready. Keep
  // the established encoder loop (or prepare it normally) until it is usable.
  // Runtime loss may also select encoder hold when explicitly configured.
  const bool imu_pending=config_.allow_imu_entry_joint_hold && !config_.require_imu_in_joint &&
      cmd.mode==Mode::Imu && !FiniteImu(in.imu,config_.pitch_imu_enabled) &&
      (config_.allow_imu_loss_joint_hold || state_!=State::Working || mode_!=Mode::Imu);
  if(imu_pending)cmd.mode=Mode::Joint;
  // Recover inertial control at neutral, rather than repeatedly switching rate
  // feedback frames during a held turn when IMU readiness flickers.
  if(state_==State::Working && config_.yaw_manual_velocity_mode && cmd.yaw!=0 &&
      mode_==Mode::Joint && cmd.mode==Mode::Imu && !config_.pitch_imu_enabled)
    cmd.mode=Mode::Joint;
  if (!cmd.online || cmd.emergency_stop) {
    stop(); out.reasons = !cmd.online ? NoCommand : EmergencyStop; return out;
  }
  uint32_t reason = 0;
  if (!Valid(config_) || !in.configuration_ready ||
      (cmd.mode != Mode::Joint && cmd.mode != Mode::Imu) ||
      !std::isfinite(cmd.yaw) || !std::isfinite(cmd.pitch) ||
      std::fabs(cmd.yaw) > 1 || std::fabs(cmd.pitch) > 1) reason |= Configuration;
  if (!FiniteJoint(in.yaw) || !FiniteJoint(in.pitch)) reason |= MotorFeedback;
  if ((config_.require_imu_in_joint || cmd.mode == Mode::Imu) && !FiniteImu(in.imu,config_.pitch_imu_enabled)) reason |= ImuUnavailable;
  if (uint32_t(now-boot_ms_) < config_.startup_guard_ms) reason |= StartupGuard;
  if (!in.transport_ok) reason |= Transport;
  if (config_.timing_trip_enabled && !consecutive) reason |= Timing;
  // The integrated pitch endpoints are soft servo limits: clamp the target
  // and outward control effort below, keeping gravity support and inward
  // recovery even if a hand pushes past an endpoint. Other faults still stop.
  if(config_.pitch_limit_shutdown && std::isfinite(in.pitch.position) &&
      (in.pitch.position<config_.pitch_min-.002f || in.pitch.position>config_.pitch_max+.002f))
    reason|=PitchLimit;
  if (state_ == State::Dead) { state_ = State::Preparing; ready_count_ = 0; resetPids(); }
  if (reason) { state_ = State::Preparing; ready_count_ = 0; resetPids(); }
  else if (state_ == State::Preparing) {
    if (!cmd.centered || cmd.yaw != 0 || cmd.pitch != 0) { ready_count_ = 0; reason |= NeutralRequired; }
    else if (++ready_count_ >= config_.ready_ms) {
      ready_count_ = config_.ready_ms; state_ = State::Working;
      auto capture=in;capture.command.mode=cmd.mode;captureTargets(capture);
    }
  }
  out.state = state_; out.ready_count = ready_count_;
  out.reasons = reason | (imu_pending ? uint32_t(ImuModePending) : 0u);
  if (state_ != State::Working) return out;
  if (mode_ != cmd.mode) {
    const bool continuous_turn=config_.yaw_manual_velocity_mode && cmd.yaw!=0 && yaw_manual_active_;
    const float previous_rate=yaw_ramped_;
    auto capture=in;capture.command=cmd;
    captureTargets(capture,!config_.pitch_imu_enabled);
    if(continuous_turn)yaw_ramped_=previous_rate;
  }
  out.active_mode=mode_;
  if (!in.yaw.enabled || !in.pitch.enabled) {
    // Enable handshake is common to both axes: do not let one move first.
    auto capture=in;capture.command.mode=cmd.mode;captureTargets(capture);
    out.active_mode=mode_;out.drive=true;
    out.reasons=EnablePending | (imu_pending ? uint32_t(ImuModePending) : 0u);
    out.yaw_target = yaw_target_; out.pitch_target = pitch_target_;
    return out;
  }
  // Timing-disabled operation still bounds PID integration after a delayed
  // callback. The independent runtime watchdog owns persistent task loss.
  const float dt=float(config_.timing_trip_enabled ? elapsed : std::clamp<uint32_t>(elapsed,1u,19u))*.001f;
  const float yaw_measured = mode_ == Mode::Joint ? in.yaw.position : in.imu.yaw;
  const bool pitch_imu=mode_==Mode::Imu && config_.pitch_imu_enabled;
  const float pitch_measured = pitch_imu ? in.imu.pitch : in.pitch.position;
  const float yaw_rate = mode_ == Mode::Joint ? in.yaw.velocity : in.imu.yaw_rate;
  const float pitch_rate = pitch_imu ? in.imu.pitch_rate : in.pitch.velocity;
  const bool manual_pitch=cmd.pitch!=0;
  if(pitch_manual_active_ && !manual_pitch) {
    pitch_target_=pitch_measured; // release holds here instead of chasing an integrated target ahead of the linkage
    pitch_ramped_=0; // discard the stale same-direction rate reference before braking
  }
  pitch_manual_active_=manual_pitch;
  const bool manual_yaw=config_.yaw_manual_velocity_mode && cmd.yaw!=0;
  if(manual_yaw!=yaw_manual_active_) {
    if(yaw_manual_active_ && !manual_yaw)yaw_target_=yaw_measured; // release holds the current heading, never catches up to an old moving target
    yaw_angle_.reset();yaw_speed_.reset();yaw_manual_speed_.reset();
    yaw_manual_active_=manual_yaw;
  }
  yaw_target_ += cmd.yaw * config_.max_yaw_rate * dt;
  pitch_target_ += cmd.pitch * config_.max_pitch_rate * dt;
  // Even in IMU mode, derive the admissible target from CURRENT joint feedback.
  const float soft_min=config_.pitch_min+config_.pitch_target_margin;
  const float soft_max=config_.pitch_max-config_.pitch_target_margin;
  const float lower = pitch_measured + soft_min - in.pitch.position;
  const float upper = pitch_measured + soft_max - in.pitch.position;
  pitch_target_ = std::clamp(pitch_target_, lower, upper);
  const float manual_yaw_position_limit=config_.yaw_position_rate_limit>0 ?
      config_.yaw_position_rate_limit : config_.max_yaw_rate;
  const float yaw_position_limit=cmd.yaw==0 && config_.yaw_neutral_rate_limit>0 ?
      config_.yaw_neutral_rate_limit : manual_yaw_position_limit;
  float yaw_lower=-yaw_position_limit,yaw_upper=yaw_position_limit;
  if(mode_==Mode::Imu && config_.yaw_relative_rate_limit>0) {
    // world yaw rate = chassis yaw rate + relative motor yaw rate. Restrict
    // the POSITION-loop reference, so faster chassis spin cannot request a
    // relative motor rate above the unchanged motor protection threshold.
    const float observed=in.imu.yaw_rate-in.yaw.velocity;
    if(!chassis_yaw_rate_seen_) {chassis_yaw_rate_estimate_=observed;chassis_yaw_rate_seen_=true;}
    else chassis_yaw_rate_estimate_+=dt/(.02f+dt)*(observed-chassis_yaw_rate_estimate_);
    yaw_lower=std::clamp(chassis_yaw_rate_estimate_-config_.yaw_relative_rate_limit,
                         -yaw_position_limit,yaw_position_limit);
    yaw_upper=std::clamp(chassis_yaw_rate_estimate_+config_.yaw_relative_rate_limit,
                         -yaw_position_limit,yaw_position_limit);
  }
  // World yaw damping uses world yaw rate; motor-coordinate pitch uses motor rate.
  // P/I can use current position while only D receives its own low-pass.
  const float yaw_command_rate=config_.yaw_manual_rate_tracking ? cmd.yaw*config_.max_yaw_rate : 0.f;
  float yv=manual_yaw ? cmd.yaw*config_.max_yaw_rate :
      yaw_angle_.update(yaw_target_,yaw_measured,dt,yaw_command_rate,yaw_rate,cmd.yaw==0,yaw_command_rate);
  yv=std::clamp(yv,yaw_lower,yaw_upper);
  float pv=pitch_angle_.update(pitch_target_,pitch_measured,dt,0,pitch_rate,cmd.pitch==0);
  // Progressive position spring in the last target-margin before each nominal
  // endpoint. It changes the outer loop only; the speed PID stays untouched.
  const float lower_penetration=std::max(0.f,soft_min-in.pitch.position);
  const float upper_penetration=std::max(0.f,in.pitch.position-soft_max);
  const float penetration=std::max(lower_penetration,upper_penetration);
  const float edge_blend=std::clamp(penetration/std::max(config_.pitch_target_margin,.001f),0.f,1.f);
  pv+=config_.pitch_limit_position_kp*(lower_penetration-upper_penetration)-
      config_.pitch_limit_position_kd*pitch_rate*edge_blend;
  const bool pitch_restoring=cmd.pitch==0 && config_.pitch_neutral_rate_limit>0;
  const float pitch_position_limit=pitch_restoring ? config_.pitch_neutral_rate_limit : config_.max_pitch_rate;
  pv=std::clamp(pv,-pitch_position_limit,pitch_position_limit);
  const float up_distance=std::max(0.f,soft_max-in.pitch.position);
  const float down_distance=std::max(0.f,in.pitch.position-soft_min);
  auto approach_speed=[&](float distance,float outward_rate) {
    const float linear=pitch_position_limit*std::clamp(distance/config_.pitch_slow_margin,0.f,1.f);
    if(config_.pitch_rate_acceleration<=0)return linear;
    const float stopping=std::sqrt(2.f*config_.pitch_rate_acceleration*distance);
    const float motion_limit=std::min(linear,stopping);
    if(!pitch_restoring || stopping<=0)return motion_limit;
    // A stationary loaded shaft needs torque authority, not an artificially
    // low speed request computed from unused travel distance. Blend toward
    // the ORIGINAL stopping-distance limit as actual outward speed rises.
    // The linear end margin remains active even at rest and is zero at/beyond
    // the endpoint. Use JOINT rate for joint travel, including IMU pitch mode.
    const float x=std::clamp(std::max(outward_rate,0.f)/stopping,0.f,1.f);
    const float moving=x*x*(3.f-2.f*x);
    return linear+moving*(motion_limit-linear);
  };
  const float up_limit=approach_speed(up_distance,in.pitch.velocity);
  const float down_limit=approach_speed(down_distance,-in.pitch.velocity);
  auto slew=[dt](float target,float current,float acceleration) {
    return acceleration>0 ? current+std::clamp(target-current,-acceleration*dt,acceleration*dt) : target;
  };
  const float yaw_acceleration=cmd.yaw==0 && config_.yaw_recovery_acceleration>0 ?
      config_.yaw_recovery_acceleration : config_.yaw_rate_acceleration;
  float yaw_slew_acceleration=yaw_acceleration;
  if(config_.yaw_braking_acceleration>0) {
    // At neutral, promptly release an outward reference for position braking.
    // During a manual turn, a transient P/D sign change is not a new stick
    // reversal: decelerate continuously instead of resetting the reference.
    const bool reversing=yv*yaw_ramped_<0;
    if(reversing && cmd.yaw==0)yaw_ramped_=0;
    if(reversing || (std::fabs(yaw_rate)>.02f && yv*yaw_rate<0) ||
        std::fabs(yv)<std::fabs(yaw_ramped_))
      yaw_slew_acceleration=std::max(yaw_acceleration,config_.yaw_braking_acceleration);
  }
  yaw_ramped_=slew(yv,yaw_ramped_,yaw_slew_acceleration);
  yaw_ramped_=std::clamp(yaw_ramped_,yaw_lower,yaw_upper);
  // Keep commanded motion smooth, but do not make a hand disturbance wait
  // behind the very slow manual-motion ramp before feedback can restore it.
  const float pitch_acceleration=cmd.pitch==0 && config_.pitch_recovery_acceleration>0 ?
      config_.pitch_recovery_acceleration : config_.pitch_rate_acceleration;
  float pitch_slew_acceleration=pitch_acceleration;
  if(config_.pitch_braking_acceleration>0) {
    if(pv*pitch_ramped_<0)pitch_ramped_=0;
    if(std::fabs(pv)<std::fabs(pitch_ramped_))
      pitch_slew_acceleration=std::max(pitch_acceleration,config_.pitch_braking_acceleration);
  }
  pitch_ramped_=slew(pv,pitch_ramped_,pitch_slew_acceleration);
  pitch_ramped_=std::clamp(pitch_ramped_,-down_limit,up_limit);
  const bool pitch_pushing_outward_at_limit=
      (cmd.pitch<0 && in.pitch.position<=soft_min) ||
      (cmd.pitch>0 && in.pitch.position>=soft_max);
  if(pitch_pushing_outward_at_limit) {
    // Do not retain either loop's state while the soft endpoint rejects the
    // command. Releasing or reversing the stick then starts from zero effort.
    pitch_ramped_=0;
    pitch_angle_.reset();
    pitch_speed_.reset();
  }
  yv=yaw_ramped_; pv=pitch_ramped_;
  const auto friction=[](float rate,float magnitude,float transition) {
    return magnitude>0 ? magnitude*std::clamp(rate/transition,-1.f,1.f) : 0.f;
  };
  const float pitch_reverse=config_.pitch_reverse_friction==-1 ? config_.pitch_friction : config_.pitch_reverse_friction;
  // At neutral, derivative/noise can reverse the speed reference even when the
  // angle is already aligned. Fade only friction assist near that alignment;
  // position/speed feedback and active braking remain fully available.
  const float yaw_friction_scale=cmd.yaw==0 && config_.yaw_neutral_friction_error>0 ?
      std::clamp(std::fabs(yaw_target_-yaw_measured)/config_.yaw_neutral_friction_error,0.f,1.f) : 1.f;
  const float yf=friction(yv,config_.yaw_friction,config_.yaw_friction_transition)*yaw_friction_scale;
  // Friction feedforward assists deliberate motion. At neutral, let symmetric
  // position/speed feedback restore the pose without a direction-dependent bias.
  const float pf=config_.pitch_gravity+(cmd.pitch==0 ? 0.f :
      friction(pv,pv<0 ? pitch_reverse : config_.pitch_friction,config_.pitch_friction_transition));
  // Slow, bounded load compensation is separate from the unchanged speed-loop
  // gains. Use the CONTROLLED frame rate: base counter-rotation is not a world
  // heading movement. Never carry an old bias across commands/error reversal.
  const float yaw_error=yaw_target_-yaw_measured;
  const float load_error=.3f*.017453293f;
  if(config_.yaw_load_torque_limit<=0 || cmd.yaw!=0 || yaw_error*yaw_load_bias_<0) {
    yaw_load_bias_=0;yaw_load_dwell_ms_=0;
  } else {
    const bool persistent=std::fabs(yaw_error)>load_error && std::fabs(yaw_rate)<.02f;
    yaw_load_dwell_ms_=persistent ? std::min<uint32_t>(150u,yaw_load_dwell_ms_+elapsed) : 0;
    const float wanted=persistent && yaw_load_dwell_ms_>=150u ?
        std::copysign(config_.yaw_load_torque_limit,yaw_error) : 0.f;
    const float rate=wanted==0 ? 30.f : config_.yaw_load_build_rate;
    yaw_load_bias_+=std::clamp(wanted-yaw_load_bias_,-rate*dt,rate*dt);
  }
  const auto smooth=[](float x) {x=std::clamp(x,0.f,1.f);return x*x*(3.f-2.f*x);};
  const float quiet=std::max(config_.yaw_angle.error_deadband,0.f);
  const float error_blend=smooth((std::fabs(yaw_error)-quiet)/std::max(load_error-quiet,.00001f));
  const float closing_rate=yaw_error*yaw_rate>0 ? std::fabs(yaw_rate) : 0.f;
  const float closing_blend=1.f-smooth((closing_rate-.005f)/.015f);
  yaw_load_torque_=yaw_load_bias_*error_blend*closing_blend;
  // Include the bias in the existing speed PID anti-windup and final torque cap.
  auto& yaw_speed_controller=manual_yaw ? yaw_manual_speed_ : yaw_speed_;
  out.yaw_torque = std::clamp(yaw_speed_controller.update(yv, yaw_rate, dt,yf+yaw_load_torque_),
                             -config_.yaw_torque_limit, config_.yaw_torque_limit);
  out.pitch_torque = std::clamp(pitch_speed_.update(pv, pitch_rate, dt,pf),
                               -config_.pitch_torque_limit, config_.pitch_torque_limit);
  // Clip outward CONTROL effort relative to the gravity equilibrium. Cutting
  // the whole torque to zero also removes load support and causes drop/reapply
  // cycling. Inward braking effort and the gravity bias remain available.
  const float requested_pitch_torque=out.pitch_torque;
  if(in.pitch.position>=config_.pitch_max)
    out.pitch_torque=std::min(out.pitch_torque,config_.pitch_gravity);
  if(in.pitch.position<=config_.pitch_min)
    out.pitch_torque=std::max(out.pitch_torque,config_.pitch_gravity);
  if(out.pitch_torque!=requested_pitch_torque) { pitch_angle_.reset();pitch_speed_.reset(); }
  const bool stalled = (config_.yaw_stall_trip_enabled &&
      std::fabs(out.yaw_torque) >= .8f*config_.yaw_torque_limit &&
      std::fabs(in.yaw.velocity)<.01f && std::fabs(yaw_target_-yaw_measured) > .02f) ||
      (std::fabs(out.pitch_torque) >= .8f*config_.pitch_torque_limit &&
      std::fabs(in.pitch.velocity) < .01f && std::fabs(pitch_target_-pitch_measured) > .02f);
  stall_count_ = stalled ? stall_count_+elapsed : 0;
  if (stall_count_ >= config_.stall_ms) {
    state_ = State::Preparing; ready_count_ = 0; resetPids();
    out = {}; out.state = state_; out.reasons = Stall; return out;
  }
  if (!std::isfinite(out.yaw_torque) || !std::isfinite(out.pitch_torque)) {
    stop(); out = {}; out.reasons = Configuration; return out;
  }
  out.drive = true; out.yaw_target = yaw_target_; out.pitch_target = pitch_target_;
  return out;
}
}
