#include "single_axis_test.hpp"
#include "single_axis_config.hpp"
#include "dm_parameters.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>
using namespace gimbal;
// Synthetic fixture gains are independent of the firmware tuning entry.
SingleAxisConfig TestAxisConfig() {
  SingleAxisConfig c;
  c.angle_pid={2,0,0,.15f,0}; c.speed_pid={1,0,0,.15f,0};
  return c;
}
void Parameters() {
  damiao::ParameterReader reader(3);
  const uint8_t regs[]{7,8,10,21,22,23};
  const uint32_t values[]{0x13,3,1,0x41480000,0x41f00000,0x41200000};
  for(unsigned i=0;i<6;++i) {
    damiao::Payload reply{3,0,0x33,regs[i],uint8_t(values[i]),uint8_t(values[i]>>8),
        uint8_t(values[i]>>16),uint8_t(values[i]>>24)};
    assert(!reader.receive(0x13,reply.data(),8)); // unsolicited data cannot configure a motor
    const auto query=reader.request(i);
    assert(query[0]==3 && query[1]==0 && query[2]==0x33 && query[3]==regs[i]);
    auto bad=reply; bad[0]=9; assert(!reader.receive(0x13,bad.data(),8));
    assert(!reader.receive(0x13,reply.data(),7));
    assert(reader.receive(0x13,reply.data(),8));
  }
  assert(reader.complete() && reader.ready() && reader.mask()==63);
  const auto config=reader.motorConfig(1,6.283185307f);
  assert(config.confirmed && config.control_id==3 && config.feedback_id==0x13);
  assert(config.limits.position==12.5f && config.limits.velocity==30 && config.limits.torque==10);
  reader.request(2);
  const damiao::Payload wrong_mode{3,0,0x33,10,2,0,0,0};
  assert(reader.receive(0x13,wrong_mode.data(),8) && !reader.ready());
  reader.request(3);
  const damiao::Payload nan{3,0,0x33,21,0,0,0xc0,0x7f};
  assert(!reader.receive(0x13,nan.data(),8));
}
struct Bench {
  SingleAxisConfig config;
  SingleAxisTest controller;
  Command cmd;
  damiao::JointFeedback fb;
  Output out;
  uint32_t now=0;
  explicit Bench(Axis axis=Axis::Yaw) : config(TestAxisConfig()),controller([&] {
    config.axis=axis; return config;
  }()) {
    cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
  }
  void step(bool configured=true,bool transport=true) { out=controller.update(cmd,fb,configured,transport,now++); }
  void prepare() {
    for(unsigned i=0;i<4001;++i) step();
    assert(out.state==State::Working && out.ready_count==2000);
  }
};
int main() {
  Parameters();
  {
    // Reverse friction changes only the negative-rate effort. Gravity never flips.
    auto config=TestAxisConfig(); config.axis=Axis::Pitch;
    config.mode=SingleAxisMode::Velocity;
    config.friction_torque=.02f; config.friction_torque_reverse=.06f;
    config.pitch_gravity_torque=.08f;
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output out; uint32_t now=0;
    auto step=[&] { out=controller.update(cmd,fb,true,true,now++); };
    for(;now<=4000;) step();
    assert(out.drive && std::fabs(out.pitch_torque-.08f)<.00001f);
    cmd.centered=false; cmd.pitch=1; fb.velocity=config.max_rate; step();
    assert(std::fabs(out.pitch_torque-.10f)<.00001f);
    cmd.pitch=-1; fb.velocity=-config.max_rate; step();
    assert(std::fabs(out.pitch_torque-.02f)<.00001f);
    cmd.pitch=-.1f; fb.velocity=-config.max_rate*.1f; step();
    assert(std::fabs(out.pitch_torque-.05f)<.00001f); // same continuous zero-crossing blend
    cmd.pitch=0; fb.velocity=0; step();
    assert(std::fabs(out.pitch_torque-.08f)<.00001f);
    cmd.emergency_stop=true; step();
    assert(!out.drive && out.pitch_torque==0 && controller.diagnostics().speed.feedforward==0);
    config.friction_torque_reverse=.2f;
    SingleAxisTest invalid(config); cmd.emergency_stop=false;
    assert(invalid.update(cmd,fb,true,true,0).reasons&Configuration);
    config.friction_torque=0; config.friction_torque_reverse=.06f; config.friction_transition_rate=0;
    SingleAxisTest invalid_blend(config);
    assert(invalid_blend.update(cmd,fb,true,true,0).reasons&Configuration);
  }
  {
    // Internal nose-up stays positive with motor direction -1. Raw motor
    // limits [-15, +4.5 deg] become internal limits [-4.5, +15 deg].
    auto config=TestAxisConfig(); config.axis=Axis::Pitch;
    config.mode=SingleAxisMode::Position;
    config.pitch_min=-4.5f*.017453293f; config.pitch_max=15.f*.017453293f;
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=true;
    fb.position=.04f; fb.enabled=false;
    Output out; uint32_t now=0;
    auto step=[&] { out=controller.update(cmd,fb,true,true,now++); };
    for(;now<=4000;) step();
    assert(out.drive && out.reasons==EnablePending && out.pitch_target==0 && out.pitch_torque==0);
    fb.enabled=true; step();
    assert(out.pitch_target==0 && out.pitch_torque<0);
    fb.position=-.04f; step();
    assert(out.pitch_target==0 && out.pitch_torque>0);
    cmd.centered=false; cmd.pitch=1; fb.position=0; step();
    assert(out.pitch_target>0);
    for(unsigned i=0;i<2500;++i) { fb.position=out.pitch_target; step(); }
    assert(out.drive && out.pitch_target==config.pitch_max);
    cmd.pitch=0; cmd.centered=true; step();
    assert(out.pitch_target==config.pitch_max); // neutral holds the requested angle
    cmd.pitch=-1; cmd.centered=false;
    for(unsigned i=0;i<3000;++i) { fb.position=out.pitch_target; step(); }
    assert(out.drive && out.pitch_target==config.pitch_min);
    cmd.emergency_stop=true; step();
    assert(!out.drive && out.pitch_torque==0);
    cmd.emergency_stop=false; cmd.pitch=0; cmd.centered=true; fb.position=-.04f;
    for(unsigned i=0;i<2100;++i) step();
    assert(out.drive && out.pitch_target==0 && out.pitch_torque>0);
    fb.position=config.pitch_min-.003f; step();
    assert(!out.drive && (out.reasons&PitchLimit));
  }
  {
    auto config=TestAxisConfig(); config.axis=Axis::Pitch;
    config.mode=SingleAxisMode::Velocity; config.pitch_gravity_torque=.08f;
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output out; uint32_t now=0;
    auto step=[&] { out=controller.update(cmd,fb,true,true,now++); };
    for(;now<=4000;) step();
    assert(out.drive && std::fabs(out.pitch_torque-.08f)<.00001f);
    for(float direction : {1.f,-1.f}) {
      cmd.centered=false; cmd.pitch=direction; fb.velocity=direction*config.max_rate;
      step(); assert(std::fabs(out.pitch_torque-.08f)<.00001f);
    }
    cmd.pitch=0; fb.velocity=0; fb.position=config.pitch_max; step();
    assert(out.pitch_torque==0); // Outward gravity bias never overrides a pitch limit.
    fb.position=0; step(); assert(out.pitch_torque>0);
    fb.enabled=false; step(); assert(out.pitch_torque==0);
    fb.enabled=true; cmd.emergency_stop=true; step();
    assert(!out.drive && out.pitch_torque==0 && controller.diagnostics().speed.feedforward==0);
    config.axis=Axis::Yaw;
    SingleAxisTest yaw(config); cmd.emergency_stop=false; cmd.centered=true;
    for(now=0;now<=4000;++now) out=yaw.update(cmd,fb,true,true,now);
    assert(out.drive && out.yaw_torque==0); // Pitch gravity never applies to yaw.
    config.axis=Axis::Pitch; config.pitch_gravity_torque=-.08f;
    SingleAxisTest negative(config);
    for(now=0;now<=4000;++now) out=negative.update(cmd,fb,true,true,now);
    assert(out.drive && std::fabs(out.pitch_torque+.08f)<.00001f);
    config.pitch_gravity_torque=config.torque_limit+.01f;
    SingleAxisTest invalid(config);
    assert(invalid.update(cmd,fb,true,true,0).reasons&Configuration);
    config.pitch_gravity_torque=.14f; config.speed_pid={.1f,10,0,.15f,.1f};
    SingleAxisTest saturated(config);
    for(now=0;now<=4000;++now) out=saturated.update(cmd,fb,true,true,now);
    cmd.centered=false; cmd.pitch=1;
    out=saturated.update(cmd,fb,true,true,now);
    assert(out.pitch_torque==.15f && saturated.diagnostics().speed.i==0);
  }
  {
    SingleAxisConfig config=TestAxisConfig();
    config.axis=Axis::Pitch; config.mode=SingleAxisMode::Velocity;
    config.max_rate=.05f; config.max_rate_acceleration=.2f;
    config.torque_limit=.2f; config.overspeed=.4f;
    config.speed_pid={1,0,0,.2f,0}; config.angle_pid={};
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output out; uint32_t now=0;
    auto step=[&] { out=controller.update(cmd,fb,true,true,now++); };
    for(;now<=4000;) step();
    assert(out.drive);
    cmd.centered=false; cmd.pitch=1;
    for(unsigned n=0;n<260;++n) step();
    assert(out.drive && out.pitch_torque>0 && out.yaw_torque==0);
    fb.position=config.pitch_max-config.limit_margin*.5f; step();
    assert(controller.diagnostics().target_rate<=config.max_rate*.50001f);
    fb.position=config.pitch_max; step();
    assert(out.drive && out.pitch_torque<=0 && controller.diagnostics().target_rate==0);
    cmd.pitch=-1; step();
    assert(out.drive && out.pitch_torque<0 && controller.diagnostics().target_rate<0);
    cmd.emergency_stop=true; step();
    assert(!out.drive && out.pitch_torque==0 && controller.diagnostics().target_rate==0);
    cmd.emergency_stop=false; cmd.pitch=0; cmd.centered=true; fb.position=0;
    for(unsigned n=0;n<2100;++n) step();
    assert(out.drive);
    fb.position=config.pitch_min-.003f; step();
    assert(!out.drive && (out.reasons&PitchLimit) && out.pitch_torque==0);
  }
  {
    // Software zero may be an endpoint: [0, +15 deg] or [-15 deg, 0].
    for(bool positive : {true,false}) {
      SingleAxisConfig config=TestAxisConfig(); config.axis=Axis::Pitch;
      config.mode=SingleAxisMode::Velocity;
      config.pitch_min=positive ? 0.f : -.2617994f;
      config.pitch_max=positive ? .2617994f : 0.f;
      SingleAxisTest controller(config);
      Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
      damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
      Output out; uint32_t now=0;
      for(;now<=4000;++now) out=controller.update(cmd,fb,true,true,now);
      assert(out.drive && !(out.reasons&Configuration));
      cmd.centered=false; cmd.pitch=positive ? -1.f : 1.f;
      out=controller.update(cmd,fb,true,true,now++);
      assert(out.drive && out.pitch_torque==0 && controller.diagnostics().target_rate==0);
      cmd.pitch=-cmd.pitch;
      out=controller.update(cmd,fb,true,true,now++);
      assert(out.drive && (positive ? out.pitch_torque>0 : out.pitch_torque<0));
      fb.position=positive ? -.003f : .003f;
      out=controller.update(cmd,fb,true,true,now++);
      assert(!out.drive && (out.reasons&PitchLimit));
    }
    auto config=TestAxisConfig(); config.pitch_min=config.pitch_max=0;
    SingleAxisTest invalid(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    assert(invalid.update(cmd,fb,true,true,0).reasons&Configuration);
  }
  {
    SingleAxisConfig config=TestAxisConfig(); config.mode=SingleAxisMode::Velocity;
    config.max_rate=.3f; config.max_rate_acceleration=.6f;
    config.torque_limit=.5f; config.speed_pid={1,0,0,.5f,0};
    config.angle_pid={}; // unused outer loop must not block a velocity-only test
    config.friction_torque=.05f;
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output out; uint32_t now=0;
    auto step=[&] { out=controller.update(cmd,fb,true,true,now++); };
    for(;now<=4000;) step();
    assert(out.drive);
    cmd.yaw=1; cmd.centered=false; fb.position=.5f;
    float previous=0;
    for(unsigned n=0;n<510;++n) {
      step(); const auto d=controller.diagnostics();
      assert(out.drive && d.angle.output==0 && out.yaw_target==fb.position);
      assert(d.target_rate>=previous && d.target_rate-previous<=.000601f);
      fb.velocity=d.target_rate; previous=d.target_rate;
    }
    assert(std::fabs(previous-.3f)<.00001f);
    cmd.yaw=-1;
    for(unsigned n=0;n<1010;++n) {
      step(); const auto d=controller.diagnostics();
      assert(out.drive && d.target_rate<=previous && previous-d.target_rate<=.000601f);
      fb.velocity=d.target_rate; previous=d.target_rate;
    }
    assert(std::fabs(previous+.3f)<.00001f);
    cmd.yaw=0; cmd.centered=true;
    for(unsigned n=0;n<510;++n) { step(); fb.velocity=controller.diagnostics().target_rate; }
    step(); assert(controller.diagnostics().target_rate==0 && std::fabs(out.yaw_torque)<.00001f);
    cmd.yaw=1; cmd.centered=false; step(); assert(controller.diagnostics().target_rate>0);
    cmd.emergency_stop=true; step();
    assert(!out.drive && out.yaw_torque==0 && controller.diagnostics().target_rate==0);

    config.max_rate=.15f; config.max_rate_acceleration=2;
    config.torque_limit=.05f; config.speed_pid={10,0,0,.05f,0}; config.friction_torque=0;
    SingleAxisTest stalled(config); now=0; cmd.yaw=0; cmd.centered=true; cmd.emergency_stop=false;
    fb.position=fb.velocity=0;
    for(;now<=4000;++now) out=stalled.update(cmd,fb,true,true,now);
    cmd.yaw=1; cmd.centered=false;
    for(unsigned n=0;n<1000;++n) out=stalled.update(cmd,fb,true,true,now++);
    assert(!out.drive && (out.reasons&Stall)); // speed mode retains stall detection without angle error
  }
  {
    Pid filtered({1,0,0,10,0,.003f});
    assert(filtered.update(0,0,.001f)==0);
    assert(std::fabs(filtered.update(0,1,.001f)+.25f)<.0001f);
    assert(std::fabs(filtered.terms().measurement-.25f)<.0001f);
    assert(std::fabs(filtered.update(0,1,.001f)+.4375f)<.0001f);
    for(unsigned i=0;i<30;++i) filtered.update(0,1,.001f);
    assert(std::fabs(filtered.terms().measurement-1)<.001f); // no DC/steady error from filter
    filtered.reset();
    assert(filtered.update(0,-2,.001f)==2); // reset starts at live measurement, not zero
    Pid raw({1,0,0,10,0,0});
    assert(raw.update(0,0,.001f)==0 && raw.update(0,1,.001f)==-1);
    Pid invalid({1,0,0,10,0,-1}); assert(invalid.update(1,0,.001f)==0);
    filtered.reset(); float raw_energy=0,filtered_energy=0;
    for(unsigned i=0;i<1000;++i) {
      const float x=.1f*std::sin(6.283185307f*100.f*float(i)*.001f);
      filtered.update(0,x,.001f);
      if(i>100) { raw_energy+=x*x; filtered_energy+=filtered.terms().measurement*filtered.terms().measurement; }
    }
    assert(filtered_energy<.25f*raw_energy); // 100 Hz input: RMS amplitude attenuated by >2x
  }
  {
    Pid p({2,3,.5f,10,4});
    assert(std::fabs(p.update(1,0,.1f)-2.3f)<.0001f);
    assert(p.terms().p==2 && std::fabs(p.terms().i-.3f)<.0001f && p.terms().d==0);
    assert(std::fabs(p.update(1,.2f,.1f)-1.14f)<.0001f);
    assert(std::fabs(p.terms().d+1)<.0001f);
    assert(p.update(100,.2f,.1f)==10);
    assert(std::fabs(p.terms().i-.54f)<.0001f); // report accepted integral after anti-windup
    p.reset(); assert(p.terms().p==0 && p.terms().i==0 && p.terms().d==0 && p.terms().output==0);
  }
  {
    Pid total({.1f,1,0,1,1});
    // PID alone is below saturation, but PID + feedforward is above it:
    // the integrator must not accumulate into this combined output limit.
    assert(total.update(.5f,0,.1f,.95f)==1);
    assert(total.integral()==0 && total.terms().feedforward==.95f);
    assert(std::fabs(total.update(-.5f,0,.1f,.95f)-.85f)<.0001f);
    assert(std::fabs(total.integral()+.05f)<.0001f);
    total.reset(); assert(total.terms().feedforward==0);
    assert(total.update(1,0,.001f,std::numeric_limits<float>::quiet_NaN())==0);
    assert(total.terms().output==0 && total.terms().feedforward==0);
  }
  {
    SingleAxisConfig config=TestAxisConfig(); config.friction_torque=.08f; config.friction_transition_rate=.02f;
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output out; uint32_t now=0;
    for(;now<=4000;++now) out=controller.update(cmd,fb,true,true,now);
    auto step=[&] { out=controller.update(cmd,fb,true,true,now++); };
    assert(out.drive);
    fb.position=-.05f; fb.velocity=.1f; step(); // zero speed error, positive friction effort
    assert(std::fabs(out.yaw_torque-.08f)<.0001f && out.pitch_torque==0);
    fb.position=.05f; fb.velocity=-.1f; step();
    assert(std::fabs(out.yaw_torque+.08f)<.0001f);
    fb.position=-.005f; fb.velocity=.01f; step();
    assert(std::fabs(out.yaw_torque-.04f)<.0001f); // continuous zero-crossing blend
    fb.position=fb.velocity=0; step();
    assert(out.yaw_torque==0 && controller.diagnostics().speed.feedforward==0);
    fb.enabled=false; step();
    assert(out.yaw_torque==0 && controller.diagnostics().speed.feedforward==0);
    fb.enabled=true; fb.position=-.1f; fb.velocity=0; step();
    assert(out.yaw_torque==config.torque_limit);
    cmd.emergency_stop=true; step();
    assert(!out.drive && out.yaw_torque==0 && controller.diagnostics().speed.feedforward==0);
  }
  {
    // Turning BOTH knobs to zero must preserve a working, finite pure PID path.
    SingleAxisConfig config=TestAxisConfig(); config.friction_torque=config.friction_transition_rate=0;
    SingleAxisTest controller(config);
    Command cmd; cmd.online=true; cmd.emergency_stop=false; cmd.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output out;
    for(uint32_t t=0;t<=4000;++t) out=controller.update(cmd,fb,true,true,t);
    assert(out.drive && out.reasons==0 && out.yaw_torque==0);
    fb.position=-.05f; fb.velocity=.02f;
    out=controller.update(cmd,fb,true,true,4001);
    assert(out.drive && std::fabs(out.yaw_torque-.08f)<.0001f);
    assert(controller.diagnostics().speed.feedforward==0);
    config.friction_torque=.08f; // active feedforward still requires a nonzero transition
    SingleAxisTest invalid(config);
    out=invalid.update(cmd,fb,true,true,0);
    assert(!out.drive && (out.reasons&Configuration));
  }
  // Verify actual firmware tuning can reach the new yaw effort ceiling while
  // keeping the inactive pitch axis at zero, and still stops on estop.
  {
    const auto config=single_axis_config::kControl;
    SingleAxisTest actual(config);
    Command c; c.online=true; c.emergency_stop=false; c.centered=true;
    damiao::JointFeedback fb; fb.online=fb.healthy=fb.zeroed=fb.enabled=true;
    Output o;
    for(uint32_t t=0;t<=4000;++t) o=actual.update(c,fb,true,true,t);
    c.centered=false;
    if(config.axis==Axis::Yaw) c.yaw=1; else c.pitch=1;
    float peak=0;
    for(uint32_t t=4001;t<=4700;++t) {
      o=actual.update(c,fb,true,true,t);
      const float effort=config.axis==Axis::Yaw ? o.yaw_torque : o.pitch_torque;
      peak=std::max(peak,std::fabs(effort));
      const auto pid=actual.diagnostics();
      assert(std::fabs(pid.target_rate)<=config.max_rate);
      assert(std::fabs(pid.torque_command-effort)<.00001f);
      assert(std::fabs(effort)<=config.torque_limit);
      assert((config.axis==Axis::Yaw ? o.pitch_torque : o.yaw_torque)==0);
    }
    if(config.axis==Axis::Yaw) assert(peak>=.99f*config.torque_limit);
    c.emergency_stop=true; o=actual.update(c,fb,true,true,4701);
    assert(!o.drive && o.yaw_torque==0 && o.pitch_torque==0);
    const auto stopped=actual.diagnostics();
    assert(stopped.angle.output==0 && stopped.speed.output==0 && stopped.target_rate==0);
  }
  Bench yaw; yaw.fb.position=3.13f; yaw.prepare();
  yaw.cmd.yaw=1; yaw.cmd.centered=false;
  for(unsigned i=0;i<200;++i) yaw.step();
  assert(yaw.out.drive && yaw.out.yaw_target>3.141593f && yaw.out.pitch_torque==0);
  assert(yaw.out.yaw_torque>0 && yaw.out.yaw_torque<=yaw.config.torque_limit);
  yaw.cmd.emergency_stop=true; yaw.step();
  assert(!yaw.out.drive && yaw.out.yaw_torque==0 && yaw.out.pitch_torque==0);
  Bench pitch(Axis::Pitch); pitch.prepare(); pitch.cmd.pitch=1; pitch.cmd.centered=false;
  pitch.fb.position=pitch.config.pitch_max;
  pitch.step(); assert(pitch.out.drive && pitch.out.pitch_torque<=0 && pitch.out.yaw_torque==0);
  pitch.fb.position=pitch.config.pitch_max+.01f;
  pitch.step(); assert(!pitch.out.drive && (pitch.out.reasons&PitchLimit));
  Bench missing; missing.prepare(); missing.fb.online=false; missing.step();
  assert(!missing.out.drive && (missing.out.reasons&MotorFeedback));
  Bench zero; zero.fb.zeroed=false; for(unsigned i=0;i<5000;++i) zero.step();
  assert(!zero.out.drive && zero.out.ready_count==0);
  Bench enable; enable.prepare(); enable.fb.enabled=false; enable.cmd.yaw=1;
  enable.step(); assert(enable.out.drive && enable.out.yaw_torque==0 && enable.out.reasons==EnablePending);
  Bench mode; mode.prepare(); mode.cmd.mode=Mode::Imu; mode.step();
  assert(!mode.out.drive && (mode.out.reasons&Configuration));
  Bench bus; bus.prepare(); bus.step(true,false); assert(!bus.out.drive && (bus.out.reasons&Transport));
  Bench gap; gap.prepare(); gap.now+=5; gap.step(); assert(!gap.out.drive && (gap.out.reasons&Timing));
  Bench overspeed; overspeed.prepare(); overspeed.fb.velocity=2; overspeed.step();
  assert(!overspeed.out.drive && (overspeed.out.reasons&Overspeed));
  overspeed.fb.velocity=0; overspeed.step(); assert(!overspeed.out.drive);
  Bench stall; stall.prepare(); stall.cmd.yaw=1; stall.cmd.centered=false;
  for(unsigned i=0;i<1600;++i) stall.step();
  assert(!stall.out.drive && (stall.out.reasons&Stall));
  std::puts("PASS: read-only motor parameters, single-axis readiness, yaw seam, pitch bounds, stop/fault guards");
}
