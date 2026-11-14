#include "gimbal_dual_axis.hpp"
#include <cmath>

namespace gimbal {
namespace {
bool StationaryDisabled(const damiao::JointFeedback& fb) {
  return fb.online && fb.healthy && !fb.enabled && std::isfinite(fb.velocity) &&
      std::fabs(fb.velocity)<=.05f;
}
}
DualAxisGimbal::DualAxisGimbal(DualAxisConfig config)
    : config_(config),controller_(config.control) {
  damiao::MotorConfig y,p;
  y.control_id=3; p.control_id=9;
  yaw_=damiao::Motor(y); pitch_=damiao::DM4340(p);
}
bool DualAxisGimbal::nextParameterQuery(uint32_t now,ParameterQuery& q) {
  if(queried_ && uint32_t(now-last_query_)<20u) return false;
  for(unsigned attempt=0;attempt<2;++attempt) {
    const bool p=next_pitch_; next_pitch_=!next_pitch_;
    auto& reader=p ? pitch_parameters_ : yaw_parameters_;
    if(reader.complete()) continue;
    q.bus=p ? AxisBus::Can2 : AxisBus::Can1; q.id=0x7ff;
    q.data=reader.request(query_slots_[p ? 1 : 0]++);
    queried_=true; last_query_=now; return true;
  }
  return false;
}
void DualAxisGimbal::applyParameters(AxisBus bus) {
  const bool p=bus==AxisBus::Can2;
  auto& reader=p ? pitch_parameters_ : yaw_parameters_;
  auto& motor=p ? pitch_ : yaw_;
  const auto& mounting=p ? config_.pitch : config_.yaw;
  if(!reader.ready() || motor.configured()) return;
  auto c=reader.motorConfig(mounting.direction,mounting.feedback_wrap);
  c.motor_per_joint=mounting.motor_per_joint;
  c.absolute_position=mounting.absolute_position;
  c.continuous_absolute=mounting.continuous_absolute;
  // CAN1 also carries board commands. A shared arbitration ID cannot be decoded
  // reliably as both a motor reply and a board command.
  if(!p && c.feedback_id==kBoardCommandId) return;
  if(p) pitch_=damiao::DM4340(c); else yaw_=damiao::Motor(c);
}
bool DualAxisGimbal::onCan(AxisBus bus,uint16_t id,const uint8_t* data,size_t n,uint32_t now) {
  if(bus==AxisBus::Can1 && id==kBoardCommandId) {
    Command decoded;
    const bool valid=DecodeCommand(data,n,decoded);
    const bool accepted=commands_.receive(data,n,now);
    if(!valid || !decoded.online || decoded.emergency_stop) {
      stop_pending_=true;
      if(valid) {latched_reason_=0;yaw_enabled_seen_=pitch_enabled_seen_=false;yaw_overspeed_seen_=false;} // explicit remote stop/offline clears fault latch
      if(valid && !accepted) commands_.invalidate(); // repeated stop still wins
    }
    return accepted;
  }
  const bool p=bus==AxisBus::Can2;
  auto& reader=p ? pitch_parameters_ : yaw_parameters_;
  auto& axis=p ? pitch_ : yaw_;
  if(data && n==8 && data[2]==0x33 && data[0]==(p ? 9 : 3) && data[1]==0) {
    const bool accepted=reader.receive(id,data,n);
    if(accepted) applyParameters(bus);
    return accepted;
  }
  if(id!=axis.config().feedback_id) return false;
  const bool accepted=p ? pitch_.onFeedback(id,data,n,now) : yaw_.onFeedback(id,data,n,now);
  if(!accepted) {
    stop_pending_=true;
    if((yaw_enabled_seen_ || pitch_enabled_seen_) && last_.drive)latched_reason_|=MotorFeedback;
  }
  return accepted;
}
const damiao::Motor& DualAxisGimbal::motor(AxisBus bus) const {
  return bus==AxisBus::Can1 ? yaw_ : pitch_;
}
uint32_t DualAxisGimbal::parameterMask(AxisBus bus) const {
  return bus==AxisBus::Can1 ? yaw_parameters_.mask() : pitch_parameters_.mask();
}
bool DualAxisGimbal::parametersReady() const {
  return yaw_parameters_.ready() && pitch_parameters_.ready() && yaw_.configured() && pitch_.configured();
}
bool DualAxisGimbal::configurationReady() const {
  const auto& c=config_.control;
  return parametersReady() &&
      std::isfinite(config_.yaw_overspeed) && config_.yaw_overspeed>c.max_yaw_rate && config_.yaw_overspeed_confirm_ms<20u &&
      std::isfinite(config_.pitch_overspeed) && config_.pitch_overspeed>c.max_pitch_rate &&
      // A holding correction reference is a torque request through the speed
      // loop, not permission to exceed the independent measured-speed trip.
      c.yaw_torque_limit<=yaw_.config().limits.torque*yaw_.config().motor_per_joint &&
      c.pitch_torque_limit<=pitch_.config().limits.torque*pitch_.config().motor_per_joint &&
      c.yaw_speed.output_limit<=c.yaw_torque_limit && c.pitch_speed.output_limit<=c.pitch_torque_limit;
}
bool DualAxisGimbal::captureZeros(uint32_t now) {
  const auto command=commands_.snapshot(now);
  if(command.online && !command.emergency_stop) return false;
  if(!parametersReady() || !StationaryDisabled(yaw_.joint(now)) ||
      !StationaryDisabled(pitch_.joint(now)) ||
      std::fabs(yaw_.raw().velocity)>.05f || std::fabs(pitch_.raw().velocity)>.05f) return false;
  // Both checks above precede either mutation: partial zero capture is avoided.
  const bool y=yaw_.config().absolute_position || yaw_.captureZero(now);
  const bool p=pitch_.config().absolute_position || pitch_.captureZero(now);
  controller_.stop(); last_={}; mode_seen_=false;
  return y && p;
}
Output DualAxisGimbal::update(uint32_t now,const ImuFeedback& imu,bool transport_ok) {
  const bool was_running=(yaw_enabled_seen_ || pitch_enabled_seen_) && last_.drive && last_.state==State::Working;
  Input in;
  in.command=commands_.snapshot(now);
  in.yaw=yaw_.joint(now); in.pitch=pitch_.joint(now);
  in.imu=imu; in.imu.ready=in.imu.ready && config_.imu_mounting_confirmed;
  in.configuration_ready=configurationReady(); in.transport_ok=transport_ok;
  if(stop_pending_) {
    if(was_running && !latched_reason_ && in.command.online && !in.command.emergency_stop)latched_reason_|=Transport;
    stop_pending_=false; controller_.stop(); mode_seen_=false;
    in.command.emergency_stop=true;
  }
  if(was_running && in.command.online && !in.command.emergency_stop &&
      ((yaw_enabled_seen_ && !in.yaw.enabled) || (pitch_enabled_seen_ && !in.pitch.enabled)))latched_reason_|=MotorFeedback;
  // Keep raw feedback as the safety source. Qualify modest yaw spikes, but
  // stop immediately for >=1.5x threshold or any pitch overspeed.
  const bool speed_check=config_.overspeed_trip_enabled && in.command.online && !in.command.emergency_stop;
  const bool yaw_fast=speed_check && in.yaw.healthy &&
      std::fabs(in.yaw.velocity)>config_.yaw_overspeed;
  bool yaw_trip=false;
  if(yaw_fast) {
    if(!yaw_overspeed_seen_){yaw_overspeed_seen_=true;yaw_overspeed_since_=now;}
    yaw_trip=std::fabs(in.yaw.velocity)>=1.5f*config_.yaw_overspeed ||
        uint32_t(now-yaw_overspeed_since_)>=config_.yaw_overspeed_confirm_ms;
  } else yaw_overspeed_seen_=false;
  if(yaw_trip || (speed_check && in.pitch.healthy && std::fabs(in.pitch.velocity)>config_.pitch_overspeed))
    latched_reason_|=Overspeed;
  if(latched_reason_) {
    controller_.stop(); last_={}; last_.reasons=latched_reason_; return last_;
  }
  last_=controller_.update(in,now);
  constexpr uint32_t runtime_faults=Configuration|MotorFeedback|ImuUnavailable|PitchLimit|Transport|Timing|Stall;
  if(was_running && (last_.reasons&runtime_faults)) {
    latched_reason_|=last_.reasons&runtime_faults;
    controller_.stop();last_={};last_.reasons=latched_reason_;return last_;
  }
  if(last_.reasons&Stall) latched_reason_|=Stall;
  if(last_.drive) {
    yaw_enabled_seen_=yaw_enabled_seen_ || in.yaw.enabled;
    pitch_enabled_seen_=pitch_enabled_seen_ || in.pitch.enabled;
  }
  if(last_.state==State::Working) {
    if(mode_seen_ && last_mode_!=last_.active_mode) {
      // Controller captures the new frame and resets PIDs. Suppress the first
      // motor effort too, including braking effort caused by a nonzero rate.
      last_.yaw_torque=last_.pitch_torque=0;
    }
    last_mode_=last_.active_mode; mode_seen_=true;
  } else mode_seen_=false;
  return last_;
}
}
