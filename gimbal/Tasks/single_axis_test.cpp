#include "single_axis_test.hpp"
#include <algorithm>
#include <cmath>
namespace gimbal {
namespace {
bool ValidPid(const PidConfig& c) {
  return std::isfinite(c.kp) && c.kp>0 && std::isfinite(c.ki) && c.ki>=0 &&
      std::isfinite(c.kd) && c.kd>=0 && std::isfinite(c.output_limit) && c.output_limit>0 &&
      std::isfinite(c.integral_limit) && c.integral_limit>=0 &&
      std::isfinite(c.measurement_filter_tau) && c.measurement_filter_tau>=0 &&
      std::isfinite(c.derivative_filter_tau) && c.derivative_filter_tau>=0 &&
      std::isfinite(c.approach_braking_acceleration) && c.approach_braking_acceleration>=0 &&
      (c.approach_braking_acceleration==0 || c.derivative_from_rate) && ValidPidErrorShaping(c);
}
}
void SingleAxisTest::stop() {
  state_=State::Dead; ready_=stall_=0; angle_.reset(); speed_.reset();
  rate_command_=torque_command_=ramped_rate_=0;
}
Output SingleAxisTest::update(const Command& cmd,const damiao::JointFeedback& fb,
    bool configured,bool transport_ok,uint32_t now) {
  Output out;
  rate_command_=torque_command_=0;
  const bool consecutive=started_ && uint32_t(now-last_)==1u;
  if(!started_) { boot_=now; started_=true; }
  last_=now;
  if(!cmd.online || cmd.emergency_stop) {
    stop(); latched_reason_=0; out.reasons=!cmd.online ? NoCommand : EmergencyStop; return out;
  }
  uint32_t reasons=0;
  const float reverse_friction=config_.friction_torque_reverse==-1 ?
      config_.friction_torque : config_.friction_torque_reverse;
  const float command=config_.axis==Axis::Yaw ? cmd.yaw : cmd.pitch;
  if(!configured || cmd.mode!=Mode::Joint || !std::isfinite(command) || std::fabs(command)>1 ||
      (config_.mode!=SingleAxisMode::Position && config_.mode!=SingleAxisMode::Velocity) ||
      !std::isfinite(config_.max_rate) || config_.max_rate<=0 ||
      !std::isfinite(config_.max_rate_acceleration) || config_.max_rate_acceleration<0 ||
      !std::isfinite(config_.torque_limit) || config_.torque_limit<=0 ||
      (config_.mode==SingleAxisMode::Position && !ValidPid(config_.angle_pid)) || !ValidPid(config_.speed_pid) ||
      config_.speed_pid.output_limit>config_.torque_limit ||
      !std::isfinite(config_.friction_torque) || config_.friction_torque<0 ||
      config_.friction_torque>config_.torque_limit ||
      !std::isfinite(reverse_friction) || reverse_friction<0 || reverse_friction>config_.torque_limit ||
      !std::isfinite(config_.friction_transition_rate) || config_.friction_transition_rate<0 ||
      ((config_.friction_torque>0 || reverse_friction>0) && config_.friction_transition_rate==0) ||
      (config_.axis==Axis::Pitch && (!std::isfinite(config_.pitch_gravity_torque) ||
          std::fabs(config_.pitch_gravity_torque)>config_.torque_limit)) ||
      !std::isfinite(config_.overspeed) || config_.overspeed<=config_.max_rate ||
      !std::isfinite(config_.pitch_min) || config_.pitch_min>0 ||
      !std::isfinite(config_.pitch_max) || config_.pitch_max<0 ||
      config_.pitch_min>=config_.pitch_max ||
      !std::isfinite(config_.limit_margin) || config_.limit_margin<=0 ||
      config_.limit_margin>=(config_.pitch_max-config_.pitch_min)*.5f) reasons|=Configuration;
  if(!fb.online || !fb.healthy || !fb.zeroed || !std::isfinite(fb.position) ||
      !std::isfinite(fb.velocity)) reasons|=MotorFeedback;
  if(!transport_ok) reasons|=Transport;
  if(uint32_t(now-boot_)<2000u) reasons|=StartupGuard;
  if(!consecutive) reasons|=Timing;
  if(std::fabs(fb.velocity)>config_.overspeed) latched_reason_|=Overspeed;
  if(config_.axis==Axis::Pitch && (fb.position<config_.pitch_min-.002f ||
      fb.position>config_.pitch_max+.002f)) reasons|=PitchLimit;
  reasons|=latched_reason_;
  if(state_==State::Dead) { state_=State::Preparing; ready_=0; }
  if(reasons) { state_=State::Preparing; ready_=stall_=0; ramped_rate_=0; angle_.reset(); speed_.reset(); }
  else if(state_==State::Preparing) {
    if(!cmd.centered || command!=0) { ready_=0; reasons|=NeutralRequired; }
    else if(++ready_>=2000u) {
      ready_=2000; state_=State::Working;
      target_=(config_.axis==Axis::Pitch && config_.mode==SingleAxisMode::Position) ? 0.f : fb.position;
    }
  }
  out.state=state_; out.ready_count=ready_; out.reasons=reasons;
  if(state_!=State::Working) return out;
  if(!fb.enabled) {
    target_=(config_.axis==Axis::Pitch && config_.mode==SingleAxisMode::Position) ? 0.f : fb.position;
    ramped_rate_=0; angle_.reset(); speed_.reset(); out.drive=true; out.reasons=EnablePending;
    if(config_.axis==Axis::Pitch) out.pitch_target=target_; else out.yaw_target=target_;
    return out;
  }
  float desired=0;
  if(config_.mode==SingleAxisMode::Velocity) {
    // Direct RC velocity command: no position target accumulation or outer PID.
    target_=fb.position; angle_.reset(); desired=command*config_.max_rate;
  } else {
    target_+=command*config_.max_rate*.001f;
    if(config_.axis==Axis::Pitch) target_=std::clamp(target_,config_.pitch_min,config_.pitch_max);
    desired=std::clamp(angle_.update(target_,fb.position,.001f,0,fb.velocity,command==0),-config_.max_rate,config_.max_rate);
  }
  if(config_.max_rate_acceleration>0) {
    const float step=config_.max_rate_acceleration*.001f;
    ramped_rate_+=std::clamp(desired-ramped_rate_,-step,step);
  } else ramped_rate_=desired;
  float rate=ramped_rate_;
  if(config_.axis==Axis::Pitch) {
    const float up=std::clamp((config_.pitch_max-fb.position)/config_.limit_margin,0.f,1.f);
    const float down=std::clamp((fb.position-config_.pitch_min)/config_.limit_margin,0.f,1.f);
    rate=std::clamp(rate,-config_.max_rate*down,config_.max_rate*up);
  }
  // Mechanical pitch bounds may cut a reference immediately; never allow the
  // smoothing state to keep an outward command alive beyond the safe window.
  ramped_rate_=rate;
  rate_command_=rate;
  // Zero amplitude disables the entire path, including the division. Both
  // feedforward parameters may be zero for pure PID tuning.
  const float friction_magnitude=rate<0 ? reverse_friction : config_.friction_torque;
  const float friction=friction_magnitude>0 ? friction_magnitude *
      std::clamp(rate/config_.friction_transition_rate,-1.f,1.f) : 0.f;
  // Gravity keeps its sign at rest and during reversal; unlike friction it does
  // not follow the requested velocity. Pid includes BOTH terms in anti-windup.
  const float gravity=config_.axis==Axis::Pitch ? config_.pitch_gravity_torque : 0.f;
  const float feedforward=friction+gravity;
  float torque=std::clamp(speed_.update(rate,fb.velocity,.001f,feedforward),
      -config_.torque_limit,config_.torque_limit);
  if(config_.axis==Axis::Pitch && ((fb.position>=config_.pitch_max && torque>0) ||
      (fb.position<=config_.pitch_min && torque<0))) { torque=0; angle_.reset(); speed_.reset(); }
  const bool stalled=std::fabs(torque)>=.8f*config_.torque_limit &&
      std::fabs(fb.velocity)<.01f && (config_.mode==SingleAxisMode::Velocity ?
          std::fabs(rate)>.01f : std::fabs(target_-fb.position)>.02f);
  stall_=stalled ? stall_+1 : 0;
  if(stall_>=500u || !std::isfinite(torque)) {
    stop(); latched_reason_|=Stall; out={}; out.reasons=Stall; return out;
  }
  out.drive=true;
  torque_command_=torque;
  if(config_.axis==Axis::Yaw) { out.yaw_torque=torque; out.yaw_target=target_; }
  else { out.pitch_torque=torque; out.pitch_target=target_; }
  return out;
}
}
