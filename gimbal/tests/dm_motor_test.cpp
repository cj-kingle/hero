#include "dm_protocol.hpp"
#include "dm_motor.hpp"
#include "dm_stop.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>
using namespace damiao;
namespace {
bool Near(float a, float b, float tolerance=.002f) { return std::fabs(a-b) < tolerance; }
Payload FeedbackFrame(uint8_t id, uint8_t state, float p, float v, float t, Limits l) {
  auto map = [](float x, float bound, unsigned bits) {
    return unsigned(std::lround((x+bound)/(2*bound)*float((1u<<bits)-1u)));
  };
  const auto pi=map(p,l.position,16), vi=map(v,l.velocity,12), ti=map(t,l.torque,12);
  return {uint8_t((state<<4)|id), uint8_t(pi>>8), uint8_t(pi), uint8_t(vi>>4),
          uint8_t((vi<<4)|(ti>>8)), uint8_t(ti), 25, 26};
}
struct FakeBus : StopTransport {
  bool abort_ok=true, busy=false, send_ok=true;
  int aborts=0, fail_on=-1, attempts=0;
  std::vector<Payload> sent;
  bool abortPending() override { ++aborts; return abort_ok; }
  bool pending() override { return busy; }
  bool send(const Payload& p, uint16_t id) override {
    assert(id==3 || id==9);
    if (!send_ok || attempts++ == fail_on) return false;
    sent.push_back(p); return true;
  }
};
void Protocol() {
  const Limits limits{12.5f,30,10}; // Synthetic test configuration, not hardware defaults.
  Payload out;
  assert(PackMit(limits,0,0,0,0,0,out));
  const Payload zero{0x80,0,0x80,0,0,0,8,0};
  assert(out==zero); // Fixed wire-vector validates bit positions/endian order.
  assert(PackMit(limits,100,100,1000,100,100,out));
  for (uint8_t byte:out) assert(byte==0xff);
  assert(PackMit(limits,-100,-100,-1,-1,-100,out));
  for (uint8_t byte:out) assert(byte==0);
  assert(!PackMit(limits,0,0,0,0,std::numeric_limits<float>::quiet_NaN(),out));
  assert(!PackMit({},0,0,0,0,0,out));
  assert(EnableFrame()[7]==0xfc && DisableFrame()[7]==0xfd);
  Feedback fb;
  Payload maximum{0x13,0xff,0xff,0xff,0xff,0xff,25,26};
  assert(DecodeFeedback(limits,maximum.data(),8,fb));
  assert(fb.id==3 && fb.state==1 && Near(fb.position,12.5f) && Near(fb.velocity,30) && Near(fb.torque,10));
  assert(!DecodeFeedback(limits,maximum.data(),7,fb));
  maximum[6]=255; assert(!DecodeFeedback(limits,maximum.data(),8,fb));
  auto pair=StopFrames(limits);
  assert(pair[0]==zero && pair[1]==DisableFrame());
  pair=StopFrames({}); assert(pair[0]==DisableFrame() && pair[1]==DisableFrame());
}
void StopTransactions() {
  const Limits l{12.5f,30,10};
  FakeBus bus; StopSequence sequence;
  bus.busy=true;
  assert(!sequence.step(bus,3,l,0) && bus.aborts==1 && bus.sent.empty());
  assert(!sequence.step(bus,3,l,1) && bus.aborts==1); // Wait for cancellation.
  bus.busy=false; bus.fail_on=1;
  assert(!sequence.step(bus,3,l,2) && bus.sent.size()==1); // Zero accepted, disable busy.
  bus.fail_on=-1;
  assert(sequence.step(bus,3,l,3));
  assert(bus.sent.size()==2 && bus.sent[0]==StopFrames(l)[0] && bus.sent[1]==DisableFrame());
  assert(sequence.step(bus,3,l,12) && bus.sent.size()==2);
  assert(sequence.step(bus,3,l,13) && bus.sent.size()==3 && bus.sent.back()==DisableFrame());
  assert(sequence.holding());
  FakeBus unconfirmed; StopSequence unknown;
  assert(unknown.step(unconfirmed,9,{},0));
  assert(unconfirmed.sent.size()==2 && unconfirmed.sent[0]==DisableFrame());
  FakeBus failure; StopSequence failed;
  failure.abort_ok=false;
  assert(!failed.step(failure,3,l,0) && failure.sent.empty() && !failed.holding());
  failure.abort_ok=true; failure.send_ok=false;
  assert(!failed.step(failure,3,l,1) && failure.sent.empty());
  failure.send_ok=true;
  assert(failed.step(failure,3,l,2));
}
void FeedbackAndWrap() {
  MotorConfig c; c.control_id=3; c.feedback_id=0x13; c.limits={12.5f,30,10};
  c.direction=1; c.confirmed=true;
  Motor motor(c);
  auto f=FeedbackFrame(3,0,3.13f,0,0,c.limits);
  assert(motor.onFeedback(0x13,f.data(),8,0) && motor.captureZero(0));
  f=FeedbackFrame(3,0,-3.13f,0,0,c.limits);
  assert(motor.onFeedback(0x13,f.data(),8,1));
  assert(motor.joint(1).position>0 && motor.joint(1).position<.03f);
  assert(!motor.joint(101).online);
  assert(motor.onFeedback(0x13,f.data(),8,102) && !motor.joint(102).zeroed);
  assert(motor.captureZero(102));
  assert(!motor.onFeedback(0x19,f.data(),8,103));
  f=FeedbackFrame(9,0,-3.13f,0,0,c.limits);
  assert(!motor.onFeedback(0x13,f.data(),8,103));
  c.control_id=9; c.feedback_id=0x19; c.direction=-1; c.motor_per_joint=2;
  c.limits={12.5f,10,28}; // Independent synthetic 4340 range for isolation test.
  Motor other(c);
  f=FeedbackFrame(9,0,0,0,0,c.limits);
  assert(other.onFeedback(0x19,f.data(),8,0) && other.captureZero(0));
  f=FeedbackFrame(9,1,.1f,1,2,c.limits);
  assert(other.onFeedback(0x19,f.data(),8,10));
  const auto j=other.joint(10);
  assert(j.enabled && Near(j.position,-.05f) && Near(j.velocity,-.5f,.004f) && Near(j.torque,-4,.03f));
  Payload torque;
  assert(other.torqueFrame(4,torque));
  const unsigned packed=(unsigned(torque[6]&15)<<8)|torque[7];
  const float decoded=float(packed)/4095*56-28;
  assert(Near(decoded,-2,.015f));
  assert(!motor.joint(102).enabled); // Independent state is not overwritten.
  c.confirmed=false; Motor invalid(c); assert(!invalid.torqueFrame(1,torque));
}
void NativeRangeRollover() {
  MotorConfig c; c.control_id=3; c.feedback_id=0x13; c.limits={12.5f,30,10};
  c.direction=1; c.confirmed=true; c.absolute_position=true; c.continuous_absolute=true;
  for(int direction : {-1,1}) {
    Motor motor(c);
    auto frame=FeedbackFrame(3,1,direction*12.49f,direction*2.f,0,c.limits);
    assert(motor.onFeedback(0x13,frame.data(),8,0));
    const float before=motor.joint(0).position;
    frame=FeedbackFrame(3,1,-direction*12.49f,direction*2.f,0,c.limits);
    assert(motor.onFeedback(0x13,frame.data(),8,10));
    assert(Near(motor.joint(10).position-before,direction*.02f,.001f));
    // A finite interior discontinuity must not invalidate continuous Yaw.
    frame=FeedbackFrame(3,1,0,0,0,c.limits);
    assert(motor.onFeedback(0x13,frame.data(),8,11));
    assert(motor.joint(11).online && motor.joint(11).healthy);
    auto bounded_config=c;bounded_config.continuous_absolute=false;
    Motor bounded(bounded_config);
    frame=FeedbackFrame(3,1,direction*12.49f,0,0,c.limits);
    assert(bounded.onFeedback(0x13,frame.data(),8,0));
    frame=FeedbackFrame(3,1,0,0,0,c.limits);
    assert(!bounded.onFeedback(0x13,frame.data(),8,1));
    assert(!bounded.joint(1).online); // the bounded-axis jump guard is unchanged
  }
}
void PositionRateObservation() {
  MotorConfig c; c.control_id=3; c.feedback_id=0x13; c.limits={12.5f,30,10};
  c.direction=-1; c.motor_per_joint=2; c.confirmed=true;
  Motor motor(c);
  for(uint32_t ms=0;ms<=60;++ms) {
    // Known position ramp, deliberately inconsistent velocity field. The new
    // observation must follow position, independent of the motor velocity data.
    auto f=FeedbackFrame(3,1,.4f*float(ms)*.001f,ms%2 ? 1.f : -1.f,0,c.limits);
    assert(motor.onFeedback(0x13,f.data(),8,ms));
    if(ms>=10) assert(Near(motor.positionRate(ms),-.2f,.025f));
  }
  assert(motor.positionRate(160)==0);
  auto f=FeedbackFrame(3,0,1,0,0,c.limits);
  assert(motor.onFeedback(0x13,f.data(),8,161) && motor.positionRate(161)==0);
  c.direction=1; c.motor_per_joint=1; Motor wrapped(c);
  for(uint32_t ms=0;ms<=20;++ms) {
    const float p=std::remainder(3.135f+.4f*float(ms)*.001f,c.feedback_wrap);
    f=FeedbackFrame(3,1,p,0,0,c.limits);
    assert(wrapped.onFeedback(0x13,f.data(),8,ms));
  }
  assert(Near(wrapped.positionRate(20),.4f,.05f));
  // Irregular arrival intervals use measured elapsed time, not a guessed 1 ms.
  Motor jitter(c);
  for(uint32_t ms : {100u,103u,107u,112u}) {
    f=FeedbackFrame(3,1,.4f*float(ms-100)*.001f,0,0,c.limits);
    assert(jitter.onFeedback(0x13,f.data(),8,ms));
  }
  assert(Near(jitter.positionRate(112),.4f,.04f));
}
}
int main() { Protocol(); NativeRangeRollover(); StopTransactions(); FeedbackAndWrap(); PositionRateObservation(); std::cout<<"DM protocol, stop retries and feedback tests passed\n"; }
