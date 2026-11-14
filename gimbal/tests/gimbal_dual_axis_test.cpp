#include "gimbal_dual_axis.hpp"
#include "gimbal_config.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
using namespace gimbal;
void CAN_Send_Msg(CAN_HandleTypeDef*,uint8_t*,uint32_t,uint8_t) {
  assert(false && "Pure dual-axis controller must not transmit CAN by itself");
}
namespace {
DualAxisConfig Configured() {
  DualAxisConfig c;
  c.yaw.direction=1; c.pitch.direction=-1; c.imu_mounting_confirmed=true;
  c.control.yaw_angle={2,0,0,.3f,0}; c.control.pitch_angle={2,0,0,.1f,0};
  c.control.yaw_speed={.1f,0,0,.3f,0}; c.control.pitch_speed={.1f,0,0,.5f,0};
  return c;
}
damiao::Payload Feedback(unsigned id,bool enabled,float p,float v,damiao::Limits l) {
  auto map=[](float x,float bound,unsigned bits) {
    return unsigned(std::lround((x+bound)/(2*bound)*float((1u<<bits)-1)));
  };
  const auto pi=map(p,l.position,16),vi=map(v,l.velocity,12),ti=map(0,l.torque,12);
  return {uint8_t((enabled ? 0x10 : 0)|id),uint8_t(pi>>8),uint8_t(pi),uint8_t(vi>>4),
      uint8_t((vi<<4)|(ti>>8)),uint8_t(ti),25,25};
}
struct Fixture {
  DualAxisGimbal module;
  Command cmd;
  ImuFeedback imu;
  uint32_t now=0;
  uint8_t sequence=0;
  bool enabled=false;
  float yaw=0,pitch=0,yaw_velocity=0,pitch_velocity=0;
  float yaw_feedback_wrap=6.283185307f;
  Output out;
  explicit Fixture(DualAxisConfig c=Configured()) : module(c) {
    cmd.online=true; cmd.emergency_stop=true; cmd.centered=true; imu.ready=true;
    if(c.control.pitch_start_at_configured_angle)pitch=c.control.pitch_start_angle;
  }
  bool command(bool advance=true) {
    if(advance) ++sequence;
    cmd.sequence=sequence;
    damiao::Payload data; assert(EncodeCommand(cmd,data));
    return module.onCan(AxisBus::Can1,kBoardCommandId,data.data(),data.size(),now);
  }
  void feedback() {
    auto y=Feedback(3,enabled,std::remainder(yaw,yaw_feedback_wrap),yaw_velocity,{12.5f,30,10});
    auto p=Feedback(9,enabled,-pitch,-pitch_velocity,{12.5f,10,28});
    assert(module.onCan(AxisBus::Can1,0x13,y.data(),8,now));
    assert(module.onCan(AxisBus::Can2,0x19,p.data(),8,now));
  }
  void discover() {
    for(;now<260;++now) {
      ParameterQuery q;
      if(!module.nextParameterQuery(now,q)) continue;
      auto r=q.data; assert(q.id==0x7ff && r[2]==0x33);
      const bool p=q.bus==AxisBus::Can2;
      uint32_t bits=0;
      const float pm=12.5f,vm=p ? 10.f : 30.f,tm=p ? 28.f : 10.f;
      switch(r[3]) {
        case 7: bits=p ? 0x19 : 0x13; break;
        case 8: bits=p ? 9 : 3; break;
        case 10: bits=1; break;
        case 21: std::memcpy(&bits,&pm,4); break;
        case 22: std::memcpy(&bits,&vm,4); break;
        case 23: std::memcpy(&bits,&tm,4); break;
        default: assert(false);
      }
      for(unsigned i=0;i<4;++i) r[4+i]=uint8_t(bits>>(8*i));
      assert(module.onCan(q.bus,p ? 0x19 : 0x13,r.data(),8,now));
    }
    assert(module.parameterMask(AxisBus::Can1)==63 && module.parameterMask(AxisBus::Can2)==63);
  }
  void step(bool send=true) {
    ++now; feedback(); if(send && now%10==0) command();
    out=module.update(now,imu);
  }
  void ready() {
    discover(); feedback(); command();
    assert(module.parametersReady() && module.captureZeros(now));
    cmd.emergency_stop=false; command();
    for(unsigned n=0;n<4100;++n) step();
    assert(out.drive && out.reasons==EnablePending && out.yaw_torque==0 && out.pitch_torque==0);
    enabled=true; step(); assert(out.drive && out.state==State::Working);
  }
};
void DiscoveryAndZero() {
  Fixture f; f.ready();
  assert(f.module.motor(AxisBus::Can1).config().control_id==3);
  assert(f.module.motor(AxisBus::Can2).config().control_id==9);
  assert(f.module.motor(AxisBus::Can1).config().limits.velocity==30);
  assert(f.module.motor(AxisBus::Can2).config().limits.velocity==10);
  assert(!f.module.captureZeros(f.now)); // release command prohibits zero capture
  f.cmd.emergency_stop=true; f.command();
  assert(!f.module.captureZeros(f.now)); // enabled feedback still prohibits zero
  f.enabled=false; f.feedback(); assert(f.module.captureZeros(f.now));

  Fixture unknown(DualAxisConfig{}); unknown.discover();
  assert(!unknown.module.parametersReady() && !unknown.module.captureZeros(unknown.now));
  ParameterQuery q; assert(!f.module.nextParameterQuery(f.now,q));

  auto geared=Configured(); geared.pitch.motor_per_joint=2;
  Fixture atomic(geared); atomic.discover(); atomic.yaw=1; atomic.pitch=.2f;
  atomic.pitch_velocity=.08f; atomic.feedback(); atomic.command();
  assert(!atomic.module.captureZeros(atomic.now)); // joint speed alone would pass, raw speed must not
  assert(!atomic.module.motor(AxisBus::Can1).joint(atomic.now).zeroed);
  assert(!atomic.module.motor(AxisBus::Can2).joint(atomic.now).zeroed);
  ++atomic.now; atomic.pitch_velocity=0; atomic.feedback();
  assert(atomic.module.captureZeros(atomic.now));
}
void ModesAndYawTurns() {
  Fixture f; f.ready();
  f.cmd.mode=Mode::Imu; f.imu.yaw=8; f.imu.pitch=.4f; f.imu.yaw_rate=.1f;
  f.command(); f.step(false);
  assert(f.out.drive && f.out.yaw_torque==0 && f.out.pitch_torque==0);
  assert(std::fabs(f.out.yaw_target-8)<.0001f);
  f.cmd.mode=Mode::Joint; f.command(); f.step(false);
  assert(f.out.drive && f.out.yaw_torque==0 && f.out.pitch_torque==0);
  assert(std::fabs(f.out.yaw_target-f.module.motor(AxisBus::Can1).joint(f.now).position)<.0001f);

  Fixture turn; turn.ready(); turn.cmd.yaw=1; turn.cmd.centered=false; turn.command();
  for(unsigned n=0;n<24000;++n) {
    turn.yaw=turn.out.yaw_target; turn.yaw_velocity=.3f;
    turn.step(); assert(turn.out.drive);
  }
  assert(turn.out.yaw_target>6.283185307f); // one full turn crossed without wrapping target
}
void IntegratedYawWhileChassisTurns() {
  // Use the actual integrated tuning, including the separate pitch gravity bias.
  for(auto mode : {Mode::Joint,Mode::Imu}) for(float input : {-1.f,1.f})
      for(float chassis_rate : {.8f,5.2f,5.7f,6.5f}) {
    Fixture f(config::kDual);f.ready();f.yaw_feedback_wrap=25.f;
    f.cmd.mode=mode;f.command();f.step(false);
    f.cmd.yaw=input;f.cmd.centered=false;f.command();
    float body_yaw=0;
    for(unsigned n=0;n<32000;++n) {
      const float body_rate=mode==Mode::Imu ? chassis_rate : 0.f;
      body_yaw+=body_rate*.001f;
      // Ideal tracking fixture exercises frame/range boundaries, not plant tuning.
      const float old=f.yaw;
      f.yaw=f.out.yaw_target-body_yaw;
      f.yaw_velocity=(f.yaw-old)/.001f;
      f.imu.yaw=f.out.yaw_target;f.imu.yaw_rate=input*config::kControl.max_yaw_rate;
      f.step();
      assert(f.out.drive && f.out.state==State::Working && f.module.faultReason()==0);
    }
  }
}
void ContinuousYawDiscontinuityDoesNotLatchStop() {
  for(float wrap : {6.283185307f,25.f}) for(float sign : {-1.f,1.f}) {
    Fixture f(config::kDual);f.ready();f.yaw_feedback_wrap=wrap;
    f.cmd.mode=Mode::Imu;f.command();f.step(false);
    // Synthetic relative shaft motion: the chassis spins while world yaw is held.
    // Exercise multiple encoder/native wraps and one finite 0.12 rad discontinuity.
    for(unsigned n=0;n<16000;++n) {
      f.yaw=sign*(3.1f*float(n+1)*.001f+(n>=9000 ? .12f : 0.f));
      f.yaw_velocity=sign*3.1f;
      f.imu.yaw=0;f.imu.yaw_rate=0;
      f.step();
      assert(f.out.drive && f.out.state==State::Working && f.module.faultReason()==0);
      assert(f.module.motor(AxisBus::Can1).joint(f.now).healthy);
    }
    assert(sign*f.module.motor(AxisBus::Can1).joint(f.now).position>6.f*6.283185307f);
  }
}
void Protection() {
  // Excess relative speed still trips under the actual integrated tuning.
  // Do not force ideal world-target tracking in the fixture beyond this limit.
  auto strict=config::kDual;strict.overspeed_trip_enabled=true;
  Fixture overspeed(strict);overspeed.ready();
  overspeed.yaw_velocity=config::kDual.yaw_overspeed+.2f;overspeed.step();
  assert(overspeed.out.drive); // one modest spike is qualified, not a permanent shutdown
  overspeed.yaw_velocity=0;overspeed.step();assert(overspeed.out.drive);
  overspeed.yaw_velocity=config::kDual.yaw_overspeed+.2f;
  for(unsigned i=0;i<config::kDual.yaw_overspeed_confirm_ms;++i){overspeed.step();assert(overspeed.out.drive);}
  overspeed.step();
  assert(!overspeed.out.drive && (overspeed.out.reasons&Overspeed));
  Fixture extreme(strict);extreme.ready();
  extreme.yaw_velocity=1.5f*config::kDual.yaw_overspeed+.2f;extreme.step();
  assert(!extreme.out.drive && (extreme.out.reasons&Overspeed));
  Fixture partial;partial.discover();partial.feedback();partial.command();assert(partial.module.captureZeros(partial.now));
  partial.cmd.emergency_stop=false;partial.command();for(unsigned n=0;n<4100;++n)partial.step();
  auto yaw_on=Feedback(3,true,0,0,{12.5f,30,10});
  assert(partial.module.onCan(AxisBus::Can1,0x13,yaw_on.data(),8,partial.now));
  partial.out=partial.module.update(partial.now+1,partial.imu);
  ++partial.now;partial.step();assert(!partial.out.drive && (partial.out.reasons&MotorFeedback)); // detect drop even before both axes enable
  Fixture disabled;disabled.ready();disabled.enabled=false;disabled.step();
  assert(!disabled.out.drive && (disabled.out.reasons&MotorFeedback));
  disabled.enabled=true;
  for(unsigned n=0;n<5000;++n){disabled.step();assert(!disabled.out.drive && (disabled.out.reasons&MotorFeedback));}
  disabled.cmd.emergency_stop=true;disabled.command();disabled.step(false);
  disabled.cmd.emergency_stop=false;disabled.command();
  for(unsigned n=0;n<2100;++n)disabled.step();
  assert(disabled.out.drive); // deliberate stop/release acknowledges, never automatic retry
  Fixture imu_lost;imu_lost.ready();imu_lost.imu.ready=false;imu_lost.step();
  assert(!imu_lost.out.drive && (imu_lost.out.reasons&ImuUnavailable));
  imu_lost.imu.ready=true;
  for(unsigned n=0;n<5000;++n){imu_lost.step();assert(!imu_lost.out.drive);}
  Fixture f; f.ready();
  f.yaw_velocity=1; f.step();
  assert(!f.out.drive && (f.out.reasons&Overspeed));
  f.yaw_velocity=0;
  for(unsigned n=0;n<100;++n) { f.step(); assert(!f.out.drive && (f.out.reasons&Overspeed)); }
  f.cmd.emergency_stop=true; f.command(); f.step(false);
  assert(!f.out.drive && f.out.reasons==EmergencyStop);

  Fixture offline; offline.ready();
  for(unsigned n=0;n<101;++n) offline.step(false);
  assert(!offline.out.drive && (offline.out.reasons&NoCommand));

  Fixture duplicate; duplicate.ready(); duplicate.cmd.emergency_stop=true;
  assert(!duplicate.command(false)); duplicate.step(false);
  assert(!duplicate.out.drive && duplicate.out.yaw_torque==0 && duplicate.out.pitch_torque==0);

  Fixture limit; limit.ready(); limit.cmd.pitch=1; limit.cmd.centered=false; limit.command();
  for(unsigned n=0;n<100;++n) { limit.pitch+=.00087266463f; limit.step(); }
  assert(limit.out.pitch_torque<=0); // joint limit applies in all attitude modes
  for(unsigned n=0;n<20;++n) { limit.pitch+=.0005f; limit.step(); }
  assert(!limit.out.drive && (limit.out.reasons&PitchLimit));

  auto c=Configured(); c.imu_mounting_confirmed=false;
  Fixture imu(c); imu.discover(); imu.feedback(); imu.command(); assert(imu.module.captureZeros(imu.now));
  imu.cmd.emergency_stop=false; imu.command();
  for(unsigned n=0;n<4100;++n) imu.step();
  assert(!imu.out.drive && (imu.out.reasons&ImuUnavailable));

  c=Configured(); c.control.yaw_torque_limit=.01f;
  c.control.yaw_speed={100,0,0,.01f,0}; c.control.yaw_angle.kp=100;
  Fixture stall(c); stall.ready(); stall.cmd.yaw=1; stall.cmd.centered=false; stall.command();
  for(unsigned n=0;n<1500;++n) stall.step();
  assert(!stall.out.drive && (stall.out.reasons&Stall));
  stall.cmd.yaw=0; stall.cmd.centered=true; stall.command();
  for(unsigned n=0;n<2200;++n) stall.step();
  assert(!stall.out.drive && (stall.out.reasons&Stall)); // no automatic restart after a stall
}
}
int main() {
  DiscoveryAndZero(); ModesAndYawTurns(); IntegratedYawWhileChassisTurns(); ContinuousYawDiscontinuityDoesNotLatchStop(); Protection();
  std::cout<<"Dormant dual-axis discovery, zero capture, handover, yaw turns and protection passed\n";
}
