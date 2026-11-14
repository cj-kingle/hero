#include "gimbal_task.hpp"
#include "gimbal_config.hpp"
#include "board_link.hpp"
#include "gyro_bias.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace gimbal;
namespace {
Config TestConfig() {
  Config c;
  c.yaw_angle={2,0,0,.3f,0}; c.pitch_angle={2,0,0,.1f,0};
  c.yaw_speed={.1f,0,0,.3f,0}; c.pitch_speed={.1f,0,0,.5f,0};
  return c;
}
Input ReadyInput() {
  Input in;
  in.command.online=true; in.command.emergency_stop=false; in.command.centered=true;
  in.yaw.online=in.yaw.healthy=in.yaw.zeroed=in.yaw.enabled=true;
  in.pitch=in.yaw; in.imu.ready=true;
  in.configuration_ready=true; return in;
}
void LinkAndWatchdog() {
  Command c; c.online=true; c.emergency_stop=false; c.yaw=-.5f; c.pitch=.25f;
  c.sequence=254; c.mode=Mode::Imu;
  std::array<uint8_t,8> packet;
  assert(EncodeCommand(c,packet));
  Command decoded; assert(DecodeCommand(packet.data(),8,decoded));
  assert(decoded.yaw==-.5f && decoded.pitch==.25f && decoded.mode==Mode::Imu);
  CommandReceiver r; WatchdogGate dog;
  assert(r.receive(packet.data(),8,0));
  assert(!dog.mayFeed(true,true,0)); // No foreground progress, despite RX IRQ.
  dog.completedControl(0);
  assert(dog.mayFeed(true,r.snapshot(0).online,0));
  assert(!r.receive(packet.data(),8,14));
  assert(!r.receive(packet.data(),8,100)); // Timeout must not revive a frozen sequence.
  assert(!r.snapshot(100).online);
  assert(!dog.mayFeed(true,true,20));
  c.sequence=255; assert(EncodeCommand(c,packet)); assert(r.receive(packet.data(),8,101));
  c.sequence=0; assert(EncodeCommand(c,packet)); assert(r.receive(packet.data(),8,115));
  packet[3]^=1;
  assert(!r.receive(packet.data(),8,116) && !r.snapshot(116).online);
  c.sequence=1; c.emergency_stop=true; c.yaw=c.pitch=0;
  assert(EncodeCommand(c,packet) && r.receive(packet.data(),8,117));
  dog.completedControl(117);
  assert(dog.mayFeed(true,r.snapshot(117).online,117)); // Valid estop keeps code responsive.
  c.sequence=2; c.online=false;
  assert(EncodeCommand(c,packet) && r.receive(packet.data(),8,118));
  assert(!dog.mayFeed(true,r.snapshot(118).online,118));
  assert(!dog.mayFeed(false,true,118));
}
void StatesAndSafety() {
  Controller ctrl(TestConfig()); auto in=ReadyInput();
  for (uint32_t t=0;t<3999;++t) {
    const auto o=ctrl.update(in,t);
    assert(o.state==State::Preparing && !o.drive && o.yaw_torque==0 && o.pitch_torque==0);
  }
  auto o=ctrl.update(in,3999);
  assert(o.state==State::Working && o.drive && o.ready_count==2000);
  in.pitch.enabled=false; in.command.centered=false; in.command.yaw=1;
  o=ctrl.update(in,4000);
  assert(o.drive && (o.reasons&EnablePending) && o.yaw_torque==0 && o.pitch_torque==0);
  in.pitch.enabled=true;
  in.command.centered=false; in.command.yaw=1;
  o=ctrl.update(in,4001); assert(o.drive && o.yaw_torque>0);
  in.command.emergency_stop=true;
  o=ctrl.update(in,4002); assert(o.state==State::Dead && !o.drive && o.yaw_torque==0);
  in.command.emergency_stop=false; in.command.centered=true; in.command.yaw=0;
  for(uint32_t t=4003;t<6002;++t) assert(ctrl.update(in,t).state==State::Preparing);
  assert(ctrl.update(in,6002).state==State::Working);
  in.pitch.online=false;
  o=ctrl.update(in,6003); assert(o.state==State::Preparing && !o.drive && (o.reasons&MotorFeedback));
  in.pitch.online=true;
  assert(ctrl.update(in,6005).ready_count==0); // Scheduler gap resets readiness.
  in.command.online=false;
  assert(ctrl.update(in,6006).state==State::Dead);
  Controller unconfigured; in=ReadyInput();
  for(uint32_t t=0;t<7000;++t) assert(!unconfigured.update(in,t).drive);
}
void ModesLimitsAndContinuousYaw() {
  Controller ctrl(TestConfig()); auto in=ReadyInput();
  for(uint32_t t=0;t<4000;++t) ctrl.update(in,t);
  in.command.mode=Mode::Imu; in.imu.yaw=100; in.imu.pitch=.4f;
  auto o=ctrl.update(in,4000);
  assert(o.drive && o.yaw_target==100 && o.pitch_target==.4f && o.yaw_torque==0 && o.pitch_torque==0);
  in.pitch.position=.08f; in.command.pitch=1; in.command.centered=false;
  for(uint32_t t=4001;t<4500;++t) {
    o=ctrl.update(in,t); assert(o.drive);
    assert(o.pitch_target<=in.imu.pitch+.087266463f-in.pitch.position+.00001f);
  }
  in.pitch.position=.087266463f;
  o=ctrl.update(in,4500); assert(o.pitch_torque<=0);
  in.pitch.position=.1f;
  o=ctrl.update(in,4501); assert(!o.drive && (o.reasons&PitchLimit));
  Controller continuous(TestConfig()); in=ReadyInput();
  for(uint32_t t=0;t<4000;++t) continuous.update(in,t);
  in.command.yaw=1; in.command.centered=false;
  for(uint32_t t=4000;t<30000;++t) {
    o=continuous.update(in,t); assert(o.drive);
    in.yaw.position=o.yaw_target; // Ideal follower lets the target pass multiple turns.
  }
  assert(o.yaw_target>6.283185307f);
}
void HybridYawOnlyImu() {
  auto c=TestConfig();c.pitch_imu_enabled=false;
  Controller ctrl(c);auto in=ReadyInput();in.pitch.position=.02f;
  Output o;
  for(uint32_t t=0;t<4000;++t)o=ctrl.update(in,t);
  const float held=o.pitch_target;
  in.command.mode=Mode::Imu;in.imu.yaw=1;
  in.imu.pitch=.8f;in.imu.pitch_rate=100;
  o=ctrl.update(in,4000);
  assert(o.drive && o.yaw_target==1 && o.pitch_target==held && o.pitch_torque==0);
  in.imu.pitch=std::numeric_limits<float>::quiet_NaN();
  o=ctrl.update(in,4001);assert(o.drive && o.pitch_torque==0); // unused pitch IMU cannot command linkage
  in.pitch.position=.03f;o=ctrl.update(in,4002);assert(o.pitch_torque<0);
  in.imu.yaw=1.1f;o=ctrl.update(in,4003);assert(o.yaw_torque<0);
  in.imu.ready=false;o=ctrl.update(in,4004);assert(!o.drive && (o.reasons&ImuUnavailable));
}
void BoundedControlJitter() {
  auto c=TestConfig();c.max_control_gap_ms=5;
  Controller ctrl(c);auto in=ReadyInput();uint32_t now=0;Output out;
  for(unsigned n=0;n<4500;++n){now+=n%31==0?2u:1u;out=ctrl.update(in,now);}
  assert(out.state==State::Working && out.drive);
  const float previous=out.yaw_target;
  in.command.yaw=1;in.command.centered=false;now+=2;out=ctrl.update(in,now);
  assert(out.drive && std::fabs(out.yaw_target-previous-c.max_yaw_rate*.002f)<.000001f);
  now+=6;out=ctrl.update(in,now);assert(!out.drive && (out.reasons&Timing));
}
void PitchNeutralRecoveryAndGravityAtLimits() {
  auto c=TestConfig();c.pitch_imu_enabled=false;c.pitch_start_at_configured_angle=true;
  c.pitch_min=-.078539816f;c.pitch_max=.436332313f;
  c.pitch_target_margin=.008726646f;c.pitch_slow_margin=.052359878f;
  c.max_pitch_rate=.5f;c.pitch_torque_limit=20;
  c.pitch_angle={2.5f,0,0,.5f,0};c.pitch_speed={8,0,0,20,1};
  c.pitch_rate_acceleration=.2f;c.pitch_recovery_acceleration=2.f;
  c.pitch_gravity=7.1f;c.pitch_friction=.6f;c.pitch_reverse_friction=1.6f;
  float restoring[2]{};
  for(unsigned side=0;side<2;++side) {
    Controller ctrl(c);auto in=ReadyInput();Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.pitch.position=side ? .02f : -.02f;
    for(uint32_t t=4000;t<4030;++t)out=ctrl.update(in,t);
    restoring[side]=out.pitch_torque-c.pitch_gravity;
    assert(out.drive && std::fabs(ctrl.diagnostics().pitch_rate)>.049f);
  }
  assert(restoring[0]>0 && restoring[1]<0);
  assert(std::fabs(restoring[0]+restoring[1])<1e-4f); // equal feedback stiffness around known gravity
  Controller ctrl(c);auto in=ReadyInput();Output out;
  for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
  in.pitch.position=c.pitch_max-.005f;in.command.pitch=1;in.command.centered=false;
  for(uint32_t t=4000;t<4600;++t) {
    out=ctrl.update(in,t);assert(out.drive);
    assert(out.pitch_target<=c.pitch_max-c.pitch_target_margin+1e-6f);
    assert(ctrl.diagnostics().pitch_rate<=0); // no outward velocity inside the parking margin
  }
  in.pitch.position=c.pitch_max;out=ctrl.update(in,4600);
  assert(out.drive && out.pitch_torque>0 && out.pitch_torque<=c.pitch_gravity);
  // A genuine hard-bound overrun still stops the controller.
  in.pitch.position=c.pitch_max+.003f;out=ctrl.update(in,4601);
  assert(!out.drive && (out.reasons&PitchLimit));
}
void YawRecoveryHasItsOwnRamp() {
  for(auto mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=TestConfig();c.yaw_rate_acceleration=2;c.yaw_recovery_acceleration=4;
    Controller ctrl(c);auto in=ReadyInput();in.command.mode=mode;Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.yaw.position=in.imu.yaw=sign*.05f;
    for(uint32_t t=4000;t<4010;++t)out=ctrl.update(in,t);
    assert(out.drive && sign*out.yaw_torque<0);
    assert(std::fabs(ctrl.diagnostics().yaw_rate+sign*.04f)<1e-5f);
    // Deliberate stick motion still obeys the ordinary startup/reversal ramp.
    in.command.yaw=sign;in.command.centered=false;
    const float before=ctrl.diagnostics().yaw_rate;
    out=ctrl.update(in,4010);
    assert(out.drive && std::fabs(ctrl.diagnostics().yaw_rate-before)<=.002001f);
  }
}
void YawBrakingDoesNotKeepTheOldDirection() {
  for(auto mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=TestConfig();c.yaw_rate_acceleration=2;c.yaw_recovery_acceleration=4;
    c.yaw_braking_acceleration=12;c.yaw_friction=.12f;
    Controller ctrl(c);auto in=ReadyInput();in.command.mode=mode;Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.command.yaw=sign;in.command.centered=false;
    for(uint32_t t=4000;t<4200;++t)out=ctrl.update(in,t);
    assert(sign*ctrl.diagnostics().yaw_rate>.1f);
    // Target has already been passed while actual motion continues outward.
    in.command.yaw=0;in.command.centered=true;
    in.yaw.position=in.imu.yaw=sign*.2f;
    in.yaw.velocity=in.imu.yaw_rate=sign*.2f;
    out=ctrl.update(in,4200);
    assert(out.drive && sign*ctrl.diagnostics().yaw_rate<0 && sign*out.yaw_torque<0);
    assert(std::fabs(ctrl.diagnostics().yaw_rate+sign*.012f)<1e-5f); // reversal must use braking=12, not recovery=4
    assert(sign*ctrl.diagnostics().yaw_speed.feedforward<=0);
  }
}
void ContinuousManualYawDoesNotResetOnFeedbackReversal() {
  for(Mode mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=TestConfig();c.yaw_angle={20,0,1,3,0};
    c.yaw_angle.derivative_from_rate=true;
    c.yaw_rate_acceleration=2;c.yaw_braking_acceleration=48;
    Controller ctrl(c);auto in=ReadyInput();in.command.mode=mode;Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.command.yaw=sign;in.command.centered=false;
    for(uint32_t t=4000;t<4100;++t)out=ctrl.update(in,t);
    const float before=ctrl.diagnostics().yaw_rate;
    const float pitch_torque=out.pitch_torque;
    assert(sign*before>.1f);
    // A brief measured-rate/D correction reverses while the stick stays held.
    in.yaw.velocity=in.imu.yaw_rate=sign*.8f;
    out=ctrl.update(in,4100);const auto d=ctrl.diagnostics();
    assert(out.drive && sign*d.yaw_angle.output<0);
    assert(sign*d.yaw_rate>0); // no instantaneous reference reset to zero
    assert(std::fabs(d.yaw_rate-before)<=c.yaw_braking_acceleration*.001f+1e-6f);
    assert(out.pitch_torque==pitch_torque);
    in.command.emergency_stop=true;out=ctrl.update(in,4101);
    assert(!out.drive && out.yaw_torque==0);
  }
}
void InitialImuRequestWaitsInJointHold() {
  auto c=TestConfig();c.require_imu_in_joint=false;c.allow_imu_entry_joint_hold=true;
  Controller ctrl(c);auto in=ReadyInput();in.command.mode=Mode::Imu;in.imu.ready=false;
  in.yaw.position=.2f;Output out;
  for(uint32_t t=0;t<4100;++t)out=ctrl.update(in,t);
  assert(out.drive && out.active_mode==Mode::Joint && (out.reasons&ImuModePending));
  assert(std::fabs(out.yaw_target-.2f)<1e-5f);
  in.imu.ready=true;in.imu.yaw=1.7f;out=ctrl.update(in,4100);
  assert(out.drive && out.active_mode==Mode::Imu && std::fabs(out.yaw_target-1.7f)<1e-5f);
  in.imu.ready=false;out=ctrl.update(in,4101);
  assert(!out.drive && (out.reasons&ImuUnavailable));
}
void PitchBrakingKeepsGravitySupport() {
  for(float sign : {-1.f,1.f}) {
    auto c=TestConfig();c.pitch_imu_enabled=false;
    c.pitch_min=-.4f;c.pitch_max=.4f;c.pitch_slow_margin=.05f;
    c.max_pitch_rate=.5f;c.pitch_torque_limit=28;c.pitch_gravity=7.1f;
    c.pitch_angle={6,0,0,.5f,0};c.pitch_speed={14,0,0,28,1};
    c.pitch_rate_acceleration=1;c.pitch_recovery_acceleration=8;c.pitch_braking_acceleration=16;
    Controller ctrl(c);auto in=ReadyInput();Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.command.pitch=sign;in.command.centered=false;
    for(uint32_t t=4000;t<4200;++t)out=ctrl.update(in,t);
    assert(sign*ctrl.diagnostics().pitch_rate>.1f);
    // Releasing the stick captures the measured pose instead of chasing the
    // integrated target that was left ahead of the linkage. D then brakes in
    // either direction relative to the known gravity equilibrium.
    in.command.pitch=0;in.command.centered=true;
    in.pitch.position=sign*.02f;in.pitch.velocity=sign*.2f;
    out=ctrl.update(in,4200);
    assert(std::fabs(out.pitch_target-in.pitch.position)<1e-6f);
    assert(out.drive && sign*ctrl.diagnostics().pitch_rate<=0);
    assert(sign*(out.pitch_torque-c.pitch_gravity)<0);
    assert(std::fabs(out.pitch_torque)<=c.pitch_torque_limit);
  }
}
void PitchSoftLimitKeepsEnabled() {
  for(float sign : {-1.f,1.f}) {
    auto c=TestConfig();c.pitch_imu_enabled=false;c.pitch_start_at_configured_angle=true;
    c.pitch_min=-.078539816f;c.pitch_max=.436332313f;c.pitch_slow_margin=.087266463f;
    c.pitch_target_margin=.008726646f;c.pitch_limit_shutdown=false;
    c.max_pitch_rate=2;c.pitch_torque_limit=28;c.pitch_gravity=7.1f;
    c.pitch_angle={10,0,.8f,2,0};c.pitch_angle.measurement_filter_tau=.005f;
    c.pitch_speed={8,0,0,28,1};c.pitch_speed.measurement_filter_tau=.003f;
    c.pitch_rate_acceleration=3;c.pitch_recovery_acceleration=8;c.pitch_braking_acceleration=16;
    Controller ctrl(c);auto in=ReadyInput();Output out;uint32_t time=0;
    for(;time<4000;++time)out=ctrl.update(in,time);
    const float beyond=sign>0 ? c.pitch_max+.02f : c.pitch_min-.02f;
    const unsigned steps=unsigned(std::ceil(std::fabs(beyond)/.0005f));
    for(unsigned n=1;n<=steps;++n) {
      in.pitch.position=beyond*float(n)/float(steps);in.pitch.velocity=sign*.5f;
      out=ctrl.update(in,time++);assert(out.drive && !(out.reasons&PitchLimit));
    }
    in.pitch.velocity=0;
    for(unsigned n=0;n<1000;++n) {
      out=ctrl.update(in,time++);assert(out.drive && !(out.reasons&PitchLimit));
      assert(sign*(out.pitch_torque-c.pitch_gravity)<=0);
      assert(out.pitch_target>=c.pitch_min+c.pitch_target_margin-1e-5f);
      assert(out.pitch_target<=c.pitch_max-c.pitch_target_margin+1e-5f);
    }
    in.command.emergency_stop=true;out=ctrl.update(in,time++);
    assert(!out.drive && (out.reasons&EmergencyStop));
  }
}
void PitchEndpointPositionSpringGetsStronger() {
  for(float sign : {-1.f,1.f}) {
    float restoring[2]{};
    for(unsigned enabled=0;enabled<2;++enabled) {
      auto c=TestConfig();c.pitch_imu_enabled=false;c.pitch_limit_shutdown=false;
      c.pitch_min=-.3f;c.pitch_max=.3f;c.pitch_target_margin=.01f;c.pitch_slow_margin=.08f;
      c.max_pitch_rate=2;c.pitch_gravity=7.1f;c.pitch_torque_limit=28;
      c.pitch_angle={10,0,0,2,0};c.pitch_speed={8,0,0,28,1};
      c.pitch_rate_acceleration=3;c.pitch_recovery_acceleration=8;
      c.pitch_limit_position_kp=enabled?30.f:0.f;c.pitch_limit_position_kd=enabled?1.8f:0.f;
      Controller ctrl(c);auto in=ReadyInput();Output out;
      in.pitch.position=sign*(c.pitch_max-c.pitch_target_margin);
      for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
      in.pitch.position=sign*c.pitch_max;
      for(uint32_t t=4000;t<4200;++t){out=ctrl.update(in,t);assert(out.drive);}
      restoring[enabled]=std::fabs(out.pitch_torque-c.pitch_gravity);
      assert(sign*(out.pitch_torque-c.pitch_gravity)<0);
      if(enabled) {
        in.pitch.position=sign*(c.pitch_max+.005f);
        for(uint32_t t=4200;t<4400;++t){out=ctrl.update(in,t);assert(out.drive);}
        assert(std::fabs(out.pitch_torque-c.pitch_gravity)>1.4f*restoring[enabled]);
      }
    }
    assert(restoring[1]>3.f*restoring[0]);
  }
}
void YawPositionOutputReservesRelativeSpeedBudget() {
  for(float sign : {-1.f,1.f}) for(float chassis_rate : {2.8f,5.4f,6.4f}) {
    auto c=TestConfig();c.max_yaw_rate=1.2f;c.yaw_relative_rate_limit=7.f;
    c.yaw_angle={12,0,1,1.2f,0};c.yaw_speed={.75f,.1f,0,.8f,.05f};c.yaw_torque_limit=.8f;
    c.yaw_rate_acceleration=2;c.yaw_recovery_acceleration=4;c.yaw_braking_acceleration=12;
    Controller ctrl(c);auto in=ReadyInput();in.command.mode=Mode::Imu;
    in.imu.yaw_rate=0;in.yaw.velocity=-sign*chassis_rate;Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.command.yaw=-sign;in.command.centered=false;
    for(uint32_t t=4000;t<4700;++t) {
      out=ctrl.update(in,t);assert(out.drive);
      const float relative=ctrl.diagnostics().yaw_rate-sign*chassis_rate;
      assert(std::fabs(relative)<=7.00001f);
    }
    const float available=c.yaw_relative_rate_limit-chassis_rate;
    const float expected=-sign*(available<c.max_yaw_rate ? available : c.max_yaw_rate);
    assert(std::fabs(ctrl.diagnostics().yaw_rate-expected)<1e-4f);
  }
}
void TenMillisecondControlJitter() {
  auto c=TestConfig();c.max_control_gap_ms=10;Controller ctrl(c);auto in=ReadyInput();Output out;
  for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
  in.command.yaw=1;in.command.centered=false;
  const float target=out.yaw_target;out=ctrl.update(in,4007);
  assert(out.drive && std::fabs(out.yaw_target-target-c.max_yaw_rate*.008f)<1e-5f);
  out=ctrl.update(in,4018);assert(!out.drive && (out.reasons&Timing));
}
void IntegratedImuLossAndTimingDoNotLatchStop() {
  auto c=config::kControl;Controller ctrl(c);auto in=ReadyInput();Output out;
  in.command.mode=Mode::Imu;in.pitch.position=c.pitch_start_angle;
  uint32_t t=0;for(;t<4000;++t)out=ctrl.update(in,t);
  assert(out.drive && out.active_mode==Mode::Imu);
  in.command.yaw=.5f;in.command.centered=false;
  for(unsigned gap : {11u,19u,50u}) {
    t+=gap;in.imu.ready=false;
    in.imu.yaw=std::numeric_limits<float>::quiet_NaN();
    out=ctrl.update(in,t);
    assert(out.drive && out.active_mode==Mode::Joint);
    assert(!(out.reasons&(ImuUnavailable|Timing)) && (out.reasons&ImuModePending));
    assert(std::isfinite(out.yaw_torque) && std::isfinite(out.pitch_torque));
    in.imu.ready=true;in.imu.yaw=.2f;out=ctrl.update(in,++t);
    assert(out.drive && out.active_mode==Mode::Joint); // held turn does not switch back on each recovered sample
    in.command.yaw=0;in.command.centered=true;out=ctrl.update(in,++t);
    assert(out.drive && out.active_mode==Mode::Imu);
    in.command.yaw=.5f;in.command.centered=false;out=ctrl.update(in,++t);
  }
  // The pure controller bounds dt; the runtime watchdog separately owns a
  // real 20 ms foreground outage (covered by gimbal_runtime_test).
  in.command.emergency_stop=true;out=ctrl.update(in,++t);
  assert(!out.drive && out.reasons==EmergencyStop);
}
void StallAndClockWrap() {
  Config c=TestConfig(); c.yaw_angle.kp=100; c.yaw_speed.kp=100; c.yaw_torque_limit=.01f;
  Controller stalled(c); auto in=ReadyInput();
  for(uint32_t t=0;t<4000;++t) stalled.update(in,t);
  in.command.yaw=1; in.command.centered=false;
  bool detected=false;
  for(uint32_t t=4000;t<6000;++t) {
    auto o=stalled.update(in,t);
    if(o.reasons&Stall) { detected=true; assert(!o.drive && o.state==State::Preparing); }
  }
  assert(detected);
  Controller wrap(TestConfig()); in=ReadyInput();
  const uint32_t start=0xfffffc00u;
  Output o;
  for(uint32_t n=0;n<4000;++n) o=wrap.update(in,start+n);
  assert(o.state==State::Working);
  GyroBias bias; const float a[3]{0,0,9.80665f}, g[3]{.01f,-.01f,.005f};
  for(uint32_t t=0;t<1999;++t) assert(!bias.update(a,g,t));
  assert(bias.update(a,g,1999));
  assert(std::fabs(bias.bias()[0]-.01f)<.00001f);
  bias.reset();
  for(uint32_t t=0;t<1000;++t) assert(!bias.update(a,g,t));
  assert(!bias.update(a,g,1001) && bias.samples()==1); // Missing calibration sample.
}
void YawSoftwareStallCanBeDisabledWithoutDisablingPitch() {
  // The head can be carried by a rotating base while the relative encoder
  // rate is zero. Exercise both base directions and both yaw-stick directions
  // for longer than the stall window, including multiple world-yaw revolutions.
  for(float spin : {-3.1f,3.1f}) for(float stick : {-1.f,1.f}) {
    auto c=config::kControl;c.yaw_torque_limit=7.5f;
    c.yaw_manual_velocity_mode=false; // keep this legacy stall-detector fixture at saturated position-correction load
    c.yaw_speed={1.f,.1f,0,7.5f,.05f};c.stall_ms=1500;c.yaw_stall_trip_enabled=false;
    Controller ctrl(c);auto in=ReadyInput();Output out;
    in.command.mode=Mode::Imu;in.pitch.position=c.pitch_start_angle;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.command.yaw=stick;in.command.centered=false;
    unsigned loaded=0;
    for(uint32_t t=4000;t<8000;++t) {
      in.imu.yaw=spin*float(t-3999)*.001f;in.imu.yaw_rate=spin;
      out=ctrl.update(in,t);
      assert(out.drive && out.state==State::Working && !(out.reasons&Stall));
      loaded+=std::fabs(out.yaw_torque)>=.8f*c.yaw_torque_limit;
      assert(std::fabs(out.yaw_torque)<=c.yaw_torque_limit);
    }
    assert(loaded>c.stall_ms); // the former relative-only detector would stop here
  }
  // A disabled yaw trip must remain disabled even when BOTH frames are still.
  auto c=config::kControl;c.yaw_speed.kp=10;c.stall_ms=100;
  c.yaw_stall_trip_enabled=false;
  Controller holding(c);auto held=ReadyInput();held.command.mode=Mode::Imu;
  held.pitch.position=c.pitch_start_angle;
  for(uint32_t t=0;t<4000;++t)holding.update(held,t);
  held.command.yaw=1;held.command.centered=false;
  for(uint32_t t=4000;t<5000;++t)assert(holding.update(held,t).drive);
  // Re-enabling the optional detector must still catch a real stationary load.
  c.yaw_stall_trip_enabled=true;
  Controller blocked(c);auto in=ReadyInput();Output out;
  in.command.mode=Mode::Imu;in.pitch.position=c.pitch_start_angle;
  for(uint32_t t=0;t<4000;++t)out=blocked.update(in,t);
  in.command.yaw=1;in.command.centered=false;bool detected=false;
  for(uint32_t t=4000;t<5000;++t) {
    out=blocked.update(in,t);
    if(out.reasons&Stall){detected=true;assert(!out.drive);break;}
  }
  assert(detected);
  // Disabling Yaw's detector must never disable Pitch's independent trip.
  c.yaw_stall_trip_enabled=false;c.pitch_speed.kp=30; // force a saturated Pitch load despite its approach-speed bound
  Controller pitch_blocked(c);in=ReadyInput();
  in.command.mode=Mode::Imu;in.pitch.position=c.pitch_start_angle;
  for(uint32_t t=0;t<4000;++t)pitch_blocked.update(in,t);
  in.pitch.position-=.03f;detected=false;
  for(uint32_t t=4000;t<5000;++t) {
    out=pitch_blocked.update(in,t);
    if(out.reasons&Stall){detected=true;assert(!out.drive);break;}
  }
  assert(detected);
}
void VariableYawPositionGainIsContinuousAndPreservesDamping() {
  {
    auto c=single_axis_config::MakeConfig(Axis::Yaw).angle_pid;
    c.variable_kd=false; // this test isolates variable P and fixed damping; scheduled D has its own regression
    assert(c.variable_kp && ValidPidErrorShaping(c));
    c.output_limit=1000000;c.kd=0;c.approach_braking_acceleration=0;
    const auto p_at=[&](float error) {Pid pid(c);return pid.update(error,0,.001f,0,0);};
    assert(p_at(.5f*c.error_deadband)==0);
    Pid low(c);low.update(.9f*c.small_error_range,0,.001f,0,0,false);
    assert(low.terms().effective_kp==c.small_error_kp);
    Pid high(c);const float large=1.1f*c.full_gain_error;
    assert(std::fabs(high.update(large,0,.001f,0,0)-c.kp*large)<1e-5f);
    assert(high.terms().effective_kp==c.kp);
    float previous=0;
    for(unsigned n=0;n<=1000;++n) {
      const float error=2*c.full_gain_error*float(n)/1000.f;
      const float p=p_at(error);
      assert(p>=previous-1e-6f && std::fabs(p+p_at(-error))<1e-5f);previous=p;
    }
    for(float boundary : {c.error_deadband,c.error_deadband+c.deadband_transition,
        c.small_error_range,c.full_gain_error})
      assert(std::fabs(p_at(boundary+1e-7f)-p_at(boundary-1e-7f))<.01f);
    c.kd=2;c.ki=10;c.integral_limit=1;c.derivative_from_rate=true;
    Pid hold(c);
    for(unsigned n=0;n<500;++n)hold.update(.5f*c.error_deadband,0,.001f,7.1f,.1f);
    assert(hold.terms().p==0 && hold.terms().i==0); // no hidden windup within the quiet band
    assert(std::fabs(hold.terms().d+.2f)<1e-5f && hold.terms().feedforward==7.1f);
    assert(std::fabs(hold.terms().output-6.9f)<1e-5f); // damping and load support survive
    Pid moving(c);const float creeping=.5f*c.error_deadband;
    moving.update(creeping,0,.001f,0,0,false);
    assert(std::fabs(moving.terms().control_error-creeping)<1e-7f);
    assert(std::fabs(moving.terms().p-c.small_error_kp*creeping)<1e-6f); // small stick motion never waits for deadband accumulation
    auto invalid=c;invalid.full_gain_error=invalid.small_error_range;
    assert(!ValidPidErrorShaping(invalid));Pid bad(invalid);assert(bad.update(1,0,.001f,0,0)==0);
  }
  Pid fixed(PidConfig{2,0,0,100,0});assert(fixed.update(1,0,.001f)==2);
  auto c=config::kControl;Controller ctrl(c);auto in=ReadyInput();Output out;
  in.pitch.position=c.pitch_start_angle;
  for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
  in.yaw.position=.5f*c.yaw_angle.error_deadband;
  in.pitch.position=c.pitch_start_angle+.5f*c.pitch_angle.error_deadband;
  for(uint32_t t=4000;t<4300;++t)out=ctrl.update(in,t);
  assert(out.drive && std::fabs(out.yaw_torque)<1e-5f);
  assert(std::fabs(out.pitch_torque-c.pitch_gravity)<1e-5f);
}
void PitchDerivativeDeadbandKeepsMotionDamping() {
  auto c=config::kControl.pitch_angle;
  assert(c.derivative_deadband_rate==.015f);
  assert(config::kControl.yaw_angle.derivative_deadband_rate==0);
  auto original=c;original.derivative_deadband_rate=0;
  const auto d_at=[&](float error,float rate,bool stationary=true) {
    Pid pid(c);pid.update(error,0,.001f,0,rate,stationary);return pid.terms().d;
  };
  for(float sign : {-1.f,1.f}) {
    Pid quiet(c);quiet.update(0,0,.001f,0,sign*.2f*c.derivative_deadband_rate);
    assert(quiet.terms().d==0 && quiet.terms().output==0);
    const float mid=sign*.5f*c.derivative_deadband_rate;
    assert(std::fabs(d_at(0,mid))<std::fabs(c.kd*mid));
    assert(d_at(0,mid)*mid<0); // the transition still opposes motion
    assert(std::fabs(d_at(0,sign*.05f)+c.kd*sign*.05f)<1e-6f);
    const float outside=c.error_deadband+c.deadband_transition+1e-6f;
    assert(std::fabs(d_at(outside,mid)+c.kd*mid)<1e-6f);
    assert(std::fabs(d_at(0,mid,false)+c.kd*mid)<1e-6f); // manual movement bypasses the quiet gate
  }
  for(float boundary : {.25f*c.derivative_deadband_rate,c.derivative_deadband_rate})
    assert(std::fabs(d_at(0,boundary+1e-7f)-d_at(0,boundary-1e-7f))<1e-4f);
  for(float boundary : {c.error_deadband,c.error_deadband+c.deadband_transition})
    assert(std::fabs(d_at(boundary+1e-7f,.01f)-d_at(boundary-1e-7f,.01f))<1e-4f);
  Pid gated(c),baseline(original);
  gated.update(0,0,.001f,0,0);baseline.update(0,0,.001f,0,0);
  gated.update(0,0,.001f,0,.1f);baseline.update(0,0,.001f,0,.1f);
  assert(gated.terms().d==baseline.terms().d); // raw motion immediately restores the original filtered D
  for(int n=0;n<1000;++n)gated.update(0,0,.001f,0,0);
  assert(gated.terms().d==0 && gated.terms().output==0);
  gated.reset();gated.update(0,0,.001f,0,0);
  assert(gated.terms().d==0);
  for(float bad : {-1.f,std::numeric_limits<float>::quiet_NaN()}) {
    auto invalid=c;invalid.derivative_deadband_rate=bad;
    assert(!ValidPidErrorShaping(invalid));
    Pid pid(invalid);assert(pid.update(0,0,.001f,0,0)==0);
  }
  auto invalid=c;invalid.derivative_from_rate=false;assert(!ValidPidErrorShaping(invalid));
}
void NeutralYawCorrectionBudgetKeepsManualMotionUnchanged() {
  for(Mode mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=config::kControl;c.yaw_load_torque_limit=0;
    assert(c.yaw_neutral_rate_limit==9.f && c.max_yaw_rate==2.f && c.yaw_position_rate_limit==3.f);
    Controller ctrl(c);auto in=ReadyInput();Output out;
    in.command.mode=mode;in.pitch.position=c.pitch_start_angle;
    const float base_rate=mode==Mode::Imu ? 3.1f : 0.f;
    in.yaw.velocity=-base_rate;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    const float target=out.yaw_target;
    if(mode==Mode::Imu)in.imu.yaw=-sign*.05f;else in.yaw.position=-sign*.05f;
    for(uint32_t t=4000;t<4500;++t) {
      out=ctrl.update(in,t);assert(out.drive && out.yaw_target==target);
      if(c.yaw_relative_rate_limit>0)
        assert(std::fabs(ctrl.diagnostics().yaw_rate-base_rate)<=c.yaw_relative_rate_limit+1e-4f);
      assert(std::fabs(out.yaw_torque)<=c.yaw_torque_limit);
    }
    const float available=c.yaw_relative_rate_limit>0 ?
        std::min(c.yaw_neutral_rate_limit,c.yaw_relative_rate_limit+sign*base_rate) : c.yaw_neutral_rate_limit;
    assert(std::fabs(ctrl.diagnostics().yaw_rate-sign*available)<1e-4f);
    assert(sign*out.yaw_torque>=std::min(c.yaw_torque_limit,c.yaw_speed.kp*available)-1e-4f);
    in.command.yaw=sign;in.command.centered=false;
    for(uint32_t t=4500;t<5500;++t) {
      out=ctrl.update(in,t);assert(out.drive);
      assert(std::fabs(ctrl.diagnostics().yaw_rate)<=3.0001f);
      assert(ctrl.diagnostics().yaw_load_torque==0);
    }
    assert(std::fabs(out.yaw_target-target-sign*2.f)<.001f); // one second of stick still advances by 2 rad
  }
}
void PitchNeutralBudgetKeepsManualSpeedAndBoundaryBraking() {
  auto configured=config::kControl;
  // Pin this regression's original holding budget independently of live tuning.
  configured.pitch_neutral_rate_limit=12.f;configured.pitch_angle.output_limit=12.f;
  assert(configured.max_pitch_rate==.85f);
  for(Mode mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=configured;auto old=c;old.pitch_neutral_rate_limit=0;
    Controller ctrl(c),baseline(old);auto in=ReadyInput();Output out,previous;
    in.command.mode=mode;in.pitch.position=c.pitch_start_angle;
    for(uint32_t t=0;t<4000;++t){out=ctrl.update(in,t);previous=baseline.update(in,t);}
    const float target=out.pitch_target;
    in.pitch.position-=sign*.06f;
    for(uint32_t t=4000;t<4700;++t) {
      out=ctrl.update(in,t);previous=baseline.update(in,t);
      assert(out.drive && previous.drive && out.pitch_target==target);
      assert(out.yaw_torque==previous.yaw_torque && out.yaw_target==previous.yaw_target);
      assert(std::fabs(out.pitch_torque)<=c.pitch_torque_limit);
    }
    assert(std::fabs(ctrl.diagnostics().pitch_rate-sign*12.f)<1e-4f);
    assert(sign*(out.pitch_torque-previous.pitch_torque)>1.f);
    in.command.pitch=sign;in.command.centered=false;
    for(uint32_t t=4700;t<4750;++t) {
      out=ctrl.update(in,t);assert(out.drive);
      assert(std::fabs(ctrl.diagnostics().pitch_rate)<=.85001f);
    }
    assert(std::fabs(out.pitch_target-target-sign*.85f*.05f)<1e-5f);

    // Isolate the travel governor from position-loop arrival braking and D.
    c.pitch_angle.kd=0;c.pitch_angle.approach_braking_acceleration=0;
    Controller edge(c);in=ReadyInput();in.command.mode=mode;in.pitch.position=c.pitch_start_angle;
    for(uint32_t t=0;t<4000;++t)edge.update(in,t);
    const float soft=sign>0 ? c.pitch_max-c.pitch_target_margin : c.pitch_min+c.pitch_target_margin;
    const float distance=.01f;
    in.pitch.position=soft-sign*distance;
    // Request outward motion even though it is close to the endpoint.
    in.command.pitch=sign;in.command.centered=false;
    for(uint32_t t=4000;t<5000;++t)edge.update(in,t);
    in.command.pitch=0;in.command.centered=true;
    const float stopping=std::sqrt(2.f*c.pitch_rate_acceleration*distance);
    float rates[2];uint32_t time=5000;
    for(int sample=0;sample<2;++sample) {
      in.pitch.velocity=sign*(stopping+(sample ? .0001f : -.0001f));
      for(int n=0;n<200;++n) {out=edge.update(in,time++);assert(out.drive);}
      rates[sample]=edge.diagnostics().pitch_rate;
      assert(sign*rates[sample]<=stopping+1e-4f);
    }
    assert(std::fabs(rates[0]-rates[1])<.001f);
    in.pitch.position=sign>0 ? c.pitch_max+.001f : c.pitch_min-.001f;
    in.pitch.velocity=0;out=edge.update(in,time);
    assert(out.drive && !(out.reasons&PitchLimit));
    assert(sign*edge.diagnostics().pitch_rate<=0);
    assert(sign*(out.pitch_torque-c.pitch_gravity)<=0);
  }
  // The higher holding reference must not silently disable the existing stall trip.
  auto blocked_config=configured;blocked_config.pitch_speed.kp=30; // exercise a saturated stationary load independently of tuning
  Controller blocked(blocked_config);auto in=ReadyInput();Output out;
  in.pitch.position=configured.pitch_start_angle;
  for(uint32_t t=0;t<4000;++t)blocked.update(in,t);
  in.pitch.position-=.04f;bool tripped=false;
  for(uint32_t t=4000;t<6000;++t) {
    out=blocked.update(in,t);
    if(out.reasons&Stall){tripped=true;assert(!out.drive);break;}
  }
  assert(tripped);
}
void SlowYawLoadCompensationBuildsAndReleasesWithoutSpeedGainChanges() {
  for(Mode mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=config::kControl;c.yaw_load_torque_limit=1.5f;c.yaw_load_build_rate=1.5f;
    c.yaw_neutral_rate_limit=0;c.yaw_angle.output_limit=c.yaw_position_rate_limit; // isolate this dormant compensation test from the new neutral budget
    auto plain=c;plain.yaw_load_torque_limit=0;
    Controller loaded(c),baseline(plain);auto in=ReadyInput();Output a,b;
    in.command.mode=mode;in.pitch.position=c.pitch_start_angle;
    for(uint32_t t=0;t<4000;++t){a=loaded.update(in,t);b=baseline.update(in,t);}
    if(mode==Mode::Imu){in.imu.yaw=-sign*.02f;in.yaw.velocity=-3.1f;} // base rotates, world heading stays held
    else in.yaw.position=-sign*.02f;
    float previous=0;
    for(uint32_t t=4000;t<5200;++t) {
      a=loaded.update(in,t);b=baseline.update(in,t);assert(a.drive && b.drive);
      const auto d=loaded.diagnostics();
      assert(a.pitch_torque==b.pitch_torque && a.pitch_target==b.pitch_target);
      assert(a.yaw_target==b.yaw_target && std::fabs(d.yaw_rate)<=c.yaw_position_rate_limit);
      assert(std::fabs(a.yaw_torque)<=c.yaw_torque_limit);
      if(t<4149)assert(d.yaw_load_torque==0); // reject a short-lived disturbance
      assert(std::fabs(d.yaw_load_torque-previous)<=c.yaw_load_build_rate*.001f+1e-5f);
      previous=d.yaw_load_torque;
    }
    assert(sign*(a.yaw_torque-b.yaw_torque)>1.49f); // real additional actuator effort, not a higher speed target
    if(mode==Mode::Imu)in.imu.yaw_rate=sign*.05f;else in.yaw.velocity=sign*.05f;
    a=loaded.update(in,5200);
    assert(a.drive && loaded.diagnostics().yaw_load_torque==0); // release when the load lets the head return
    if(mode==Mode::Imu)in.imu.yaw_rate=0;else in.yaw.velocity=0;
    for(uint32_t t=5201;t<6500;++t)a=loaded.update(in,t);
    assert(sign*loaded.diagnostics().yaw_load_torque>1.49f);
    if(mode==Mode::Imu)in.imu.yaw=sign*.02f;else in.yaw.position=sign*.02f;
    a=loaded.update(in,6500);assert(a.drive && loaded.diagnostics().yaw_load_torque==0); // no stale bias after error reversal
    // A manual turn must not inherit holding compensation.
    in.command.yaw=sign;in.command.centered=false;
    for(uint32_t t=6501;t<6700;++t) {
      a=loaded.update(in,t);assert(a.drive && loaded.diagnostics().yaw_load_torque==0);
    }
    in.command.emergency_stop=true;a=loaded.update(in,6700);
    assert(!a.drive && loaded.diagnostics().yaw_load_torque==0);
  }
}
void ManualYawRateReferenceIndependentOfChassisAndAngle() {
  for(Mode mode : {Mode::Joint,Mode::Imu}) for(float stick : {-1.f,-.25f,.25f,1.f}) {
    auto c=config::kControl;Controller ctrl(c);auto in=ReadyInput();Output out;
    in.command.mode=mode;in.pitch.position=c.pitch_start_angle;
    uint32_t t=0;for(;t<4000;++t)out=ctrl.update(in,t);
    const float rate=stick*c.max_yaw_rate;
    in.command.yaw=stick;in.command.centered=false;
    float world_yaw=out.yaw_target,body_yaw=0;
    for(unsigned n=0;n<1800;++n) {
      const float body_rate=mode==Mode::Imu ? (n%400<200 ? 3.1f : 1.f) : 0.f;
      body_yaw+=body_rate*.001f;world_yaw+=rate*.001f;
      in.yaw.position=world_yaw-body_yaw;in.yaw.velocity=rate-body_rate;
      in.imu.yaw=world_yaw;in.imu.yaw_rate=rate;
      if(n>1600) {in.yaw.position+=.08f;in.imu.yaw+=.08f;} // angle lag/jump cannot make the velocity reference catch up
      out=ctrl.update(in,t++);const auto d=ctrl.diagnostics();
      assert(out.drive && d.yaw_angle.feedforward==0);
      assert(std::fabs(d.yaw_angle.d)<1e-6f); // no D drag at the requested world rate, regardless of chassis rotation
      assert(std::fabs(d.yaw_rate)<=c.yaw_position_rate_limit+1e-5f);
      assert(std::fabs(out.pitch_torque-c.pitch_gravity)<1e-5f);
      if(n>1500)assert(std::fabs(d.yaw_rate-rate)<1e-4f);
    }
    const float target=mode==Mode::Imu ? in.imu.yaw : in.yaw.position;
    in.command.yaw=0;in.command.centered=true;
    out=ctrl.update(in,t++);
    assert(out.drive && out.yaw_target==target && ctrl.diagnostics().yaw_angle.feedforward==0);
    in.command.emergency_stop=true;out=ctrl.update(in,t);
    assert(!out.drive && out.yaw_torque==0);
  }
  auto c=config::kControl.yaw_angle;
  c.variable_kd=false; // exercise the explicit target-rate API independently of live near-zero D scheduling
  const float desired=1.f;
  Pid faster(c);faster.update(0,0,.001f,desired,1.2f,false,desired);
  Pid slower(c);slower.update(0,0,.001f,desired,.8f,false,desired);
  assert(faster.terms().d<0 && slower.terms().d>0);
  Pid invalid(c);assert(invalid.update(0,0,.001f,0,0,false,std::numeric_limits<float>::quiet_NaN())==0);
}
void ManualYawVelocityRejectsPersistentLoad() {
  // A synthetic inertial/viscous plant checks the velocity integrator under a
  // load step. It is a software regression, not an identified gimbal model.
  for(Mode mode : {Mode::Joint,Mode::Imu}) for(float sign : {-1.f,1.f}) {
    auto c=config::kControl;Controller ctrl(c);auto in=ReadyInput();Output out;
    in.command.mode=mode;in.pitch.position=c.pitch_start_angle;
    uint32_t t=0;for(;t<4000;++t)out=ctrl.update(in,t);
    in.command.yaw=sign;in.command.centered=false;
    float speed=0,world=0,body=0;
    for(unsigned n=0;n<15000;++n) {
      const float body_rate=mode==Mode::Imu ? (n%1000<500 ? 3.1f : 1.f) : 0.f;
      body+=body_rate*.001f;
      in.yaw.position=world-body;in.yaw.velocity=sign*speed-body_rate;
      in.imu.yaw=world;in.imu.yaw_rate=sign*speed;
      out=ctrl.update(in,t++);assert(out.drive);
      const auto d=ctrl.diagnostics();
      if(n>1500)assert(std::fabs(d.yaw_rate-sign*2.f)<1e-5f);
      assert(std::fabs(d.yaw_speed.i)<=c.yaw_manual_integral_limit+1e-6f);
      assert(std::fabs(out.yaw_torque)<=c.yaw_torque_limit);
      const float load=n>=5000 ? .2f : 0.f;
      speed=std::max(0.f,speed+.001f*(sign*out.yaw_torque-.12f-.1f*speed-load)/.2f);
      world+=sign*speed*.001f;
    }
    assert(std::fabs(speed-2.f)<.01f);
    assert(sign*ctrl.diagnostics().yaw_speed.i>.35f); // steady torque comes from the manual integrator, not position catch-up
    in.command.yaw=0;in.command.centered=true;out=ctrl.update(in,t++);
    assert(out.drive && ctrl.diagnostics().yaw_speed.i==0);
    assert(std::fabs(out.yaw_target-(mode==Mode::Imu ? in.imu.yaw : in.yaw.position))<1e-6f);
  }
}
void IntegratedYawTorqueBudget() {
  Controller ctrl(config::kControl);auto in=ReadyInput();Output out;
  in.pitch.position=config::kControl.pitch_start_angle; // hold pitch at its configured pose while testing yaw
  for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
  assert(out.drive);
  in.command.yaw=1;in.command.centered=false;
  for(uint32_t t=4000;t<5550;++t) {
    out=ctrl.update(in,t);assert(out.drive);
    assert(std::fabs(out.yaw_torque)<=config::kControl.yaw_torque_limit);
  }
  // Real integrated path, stationary feedback: extra budget reaches the
  // actuator request without changing the speed P/I gains or the motor driver.
  assert(out.yaw_torque>1.8f);
  assert(std::fabs(ctrl.diagnostics().yaw_rate-2.f)<1e-4f);
  assert(std::fabs(out.yaw_target-2.f*1.55f)<1e-3f); // manual target still advances at 2 rad/s
}
void YawNeutralFrictionFadesWithoutDisablingFeedback() {
  auto c=TestConfig();c.max_yaw_rate=2;c.yaw_torque_limit=1.5f;
  c.yaw_angle={200,0,.2f,2,0};c.yaw_speed={.75f,.1f,0,1.5f,.05f};
  c.yaw_friction=.12f;c.yaw_friction_transition=.03f;c.yaw_neutral_friction_error=.01f;
  for(float sign : {-1.f,1.f}) {
    Controller ctrl(c);auto in=ReadyInput();Output out;
    for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
    in.yaw.position=sign*.001f;out=ctrl.update(in,4000);
    auto d=ctrl.diagnostics();
    assert(out.drive && sign*out.yaw_torque<0);
    assert(std::fabs(d.yaw_speed.feedforward)<=.012001f); // 10% friction, symmetric restoring feedback
    in.yaw.position=0;out=ctrl.update(in,4001);d=ctrl.diagnostics();
    assert(out.drive && std::fabs(d.yaw_rate)>.03f); // derivative braking is still active
    assert(d.yaw_speed.feedforward==0 && std::fabs(out.yaw_torque)>0);
    in.command.yaw=sign;in.command.centered=false;out=ctrl.update(in,4002);d=ctrl.diagnostics();
    assert(out.drive && sign*d.yaw_speed.feedforward>.119f); // commanded motion retains full assist
  }
}
void ConfiguredPitchStartupAndModeHandover() {
  auto c=config::kControl;Controller ctrl(c);auto in=ReadyInput();Output out;
  in.yaw.position=.4f; // startup yaw must continue to capture the current heading
  for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
  assert(out.drive && std::fabs(out.pitch_target-17.5f*.017453293f)<1e-6f);
  assert(std::fabs(out.yaw_target-.4f)<1e-6f);
  in.pitch.position=out.pitch_target; // ideal following, not a simulated mechanism
  in.command.pitch=.1f;in.command.centered=false;
  out=ctrl.update(in,4000);const float commanded=out.pitch_target;
  assert(commanded>c.pitch_start_angle);
  in.command.pitch=0;in.command.centered=true;in.command.mode=Mode::Imu;
  in.imu.yaw=1.f;out=ctrl.update(in,4001);
  assert(out.drive && std::fabs(out.pitch_target-commanded)<1e-6f);
  in.command.mode=Mode::Joint;out=ctrl.update(in,4002);
  assert(out.drive && std::fabs(out.pitch_target-commanded)<1e-6f);
  in.command.emergency_stop=true;out=ctrl.update(in,4003);assert(!out.drive);
  in.command.emergency_stop=false;
  for(uint32_t t=4004;t<6004;++t)out=ctrl.update(in,t);
  assert(out.drive && std::fabs(out.pitch_target-c.pitch_start_angle)<1e-6f);
  c.pitch_start_angle=c.pitch_max;Controller invalid(c);out=invalid.update(in,0);
  assert(!out.drive && (out.reasons&Configuration)); // never initialize outside the inner soft bounds
}
void RateDampingRejectsAngleQuantization() {
  PidConfig c{240,0,12,2,0};c.derivative_from_rate=true;c.derivative_filter_tau=.010f;
  Pid pid(c);
  // Position-code dithering without physical velocity must not create D kicks.
  for(unsigned n=0;n<1000;++n) {
    const float position=n%2 ? .00019f : -.00019f;
    pid.update(0,position,.001f,0,0);
    assert(pid.terms().d==0);
    assert(std::fabs(pid.terms().p+240*position)<1e-6f); // no P-path filter delay
    assert(std::fabs(pid.terms().output)<.05f);
  }
  pid.update(.1f,0,.001f,0,0);assert(pid.terms().d==0); // no target-step kick
  pid.reset();pid.update(0,0,.001f,0,0);
  float raw_energy=0,filtered_energy=0;
  for(unsigned n=0;n<1000;++n) {
    const float velocity=.1f*std::sin(6.283185307f*100.f*float(n)*.001f);
    pid.update(0,0,.001f,0,velocity);
    if(n>100) {raw_energy+=144*velocity*velocity;filtered_energy+=pid.terms().d*pid.terms().d;}
  }
  assert(filtered_energy<.1f*raw_energy); // synthetic 100 Hz rate noise: D energy attenuation
  pid.reset();pid.update(0,0,.001f,0,.1f);
  assert(std::fabs(pid.terms().d+1.2f)<1e-5f); // reset starts with live rate, not old filter history
  pid.update(0,0,.001f,0,std::numeric_limits<float>::quiet_NaN());
  assert(pid.terms().output==0 && pid.terms().d==0);
  c.derivative_filter_tau=-1;Pid invalid(c);assert(invalid.update(1,0,.001f,0,0)==0);
}
void RateDampingUsesTheActiveCoordinateFrame() {
  auto c=TestConfig();c.pitch_imu_enabled=false;
  c.yaw_angle={2,0,1,.3f,0};c.pitch_angle={2,0,1,.1f,0};
  c.yaw_angle.derivative_from_rate=c.pitch_angle.derivative_from_rate=true;
  c.yaw_angle.derivative_filter_tau=c.pitch_angle.derivative_filter_tau=.004f;
  Controller ctrl(c);auto in=ReadyInput();in.command.mode=Mode::Imu;
  in.yaw.velocity=-5.2f;in.imu.yaw_rate=0;
  in.pitch.velocity=.05f;in.imu.pitch_rate=100;Output out;
  for(uint32_t t=0;t<4000;++t)out=ctrl.update(in,t);
  auto d=ctrl.diagnostics();assert(out.drive);
  assert(d.yaw_angle.d==0); // chassis rotates, world yaw is still
  assert(std::fabs(d.pitch_angle.d+.05f)<1e-6f); // linkage pitch uses motor velocity
  in.imu.yaw_rate=.2f;out=ctrl.update(in,4000);d=ctrl.diagnostics();
  assert(out.drive && std::fabs(d.yaw_angle.d+.04f)<1e-5f);
  in.command.mode=Mode::Joint;out=ctrl.update(in,4001);d=ctrl.diagnostics();
  assert(out.drive && std::fabs(d.yaw_angle.d-5.2f)<1e-5f); // reset filter and change to relative motor rate
}
void ApproachBrakingPreservesHoldingAndOppositeBraking() {
  PidConfig c{100,10,2,2.4f,.1f};c.derivative_from_rate=true;
  c.approach_braking_acceleration=8;
  for(float sign : {-1.f,1.f}) {
    Pid pid(c);const float target=sign*.05f;
    assert(std::fabs(pid.update(target,0,.001f,0,0)-sign*2.4f)<1e-6f); // static holding retains full budget
    pid.reset();
    assert(std::fabs(pid.update(target,0,.001f,0,-sign*.1f)-sign*2.4f)<1e-6f); // pushing away retains full effort
    pid.reset();const float approach=pid.update(target,0,.001f,0,sign*.1f);
    assert(std::fabs(approach-sign*std::sqrt(2.f*8.f*.05f))<1e-6f);
    assert(pid.integral()==0); // approach limiting is part of anti-windup
    pid.reset();
    assert(std::fabs(pid.update(target,0,.001f,0,sign*.1f,false)-sign*2.4f)<1e-6f); // commanded moving target is not arrival-limited
    pid.reset();const float brake=pid.update(target,0,.001f,0,sign*3.f);
    assert(sign*brake<-.9f); // D's reverse braking must not be cut by the positive approach ceiling
  }
  c.derivative_from_rate=false;Pid invalid(c);
  assert(invalid.update(1,0,.001f)==0); // approach control needs an explicitly supplied rate
}
}
int main() { LinkAndWatchdog(); StatesAndSafety(); ModesLimitsAndContinuousYaw(); HybridYawOnlyImu(); BoundedControlJitter(); PitchNeutralRecoveryAndGravityAtLimits(); YawRecoveryHasItsOwnRamp(); YawBrakingDoesNotKeepTheOldDirection(); ContinuousManualYawDoesNotResetOnFeedbackReversal(); InitialImuRequestWaitsInJointHold(); PitchBrakingKeepsGravitySupport(); PitchSoftLimitKeepsEnabled(); PitchEndpointPositionSpringGetsStronger(); YawPositionOutputReservesRelativeSpeedBudget(); TenMillisecondControlJitter(); IntegratedImuLossAndTimingDoNotLatchStop(); StallAndClockWrap(); YawSoftwareStallCanBeDisabledWithoutDisablingPitch(); VariableYawPositionGainIsContinuousAndPreservesDamping(); PitchDerivativeDeadbandKeepsMotionDamping(); SlowYawLoadCompensationBuildsAndReleasesWithoutSpeedGainChanges(); NeutralYawCorrectionBudgetKeepsManualMotionUnchanged(); PitchNeutralBudgetKeepsManualSpeedAndBoundaryBraking(); ManualYawRateReferenceIndependentOfChassisAndAngle(); ManualYawVelocityRejectsPersistentLoad(); IntegratedYawTorqueBudget(); YawNeutralFrictionFadesWithoutDisablingFeedback(); ConfiguredPitchStartupAndModeHandover(); RateDampingRejectsAngleQuantization(); RateDampingUsesTheActiveCoordinateFrame(); ApproachBrakingPreservesHoldingAndOppositeBraking(); std::cout<<"Gimbal states, watchdog, limits and continuous yaw tests passed\n"; }
