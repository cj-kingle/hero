#include "main_task.hpp"
#include "gyro_bias.hpp"
#include "m3508.hpp"
#include "c620_protocol.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <array>
using namespace chassis;
using namespace hello_world::devices::remote_control;
static std::array<uint8_t, 18> Frame(unsigned left = 3, unsigned right = 1,
                                     unsigned ch0 = 1024) {
  std::array<uint8_t, 18> d{};
  unsigned ch[4] = {ch0, 1024, 1024, 1024};
  for (unsigned c = 0; c < 4; ++c)
    for (unsigned b = 0; b < 11; ++b)
      d[(11*c+b)/8] |= ((ch[c] >> b) & 1u) << ((11*c+b)%8);
  d[5] |= (left << 6) | (right << 4); d[17] = 4;
  return d;
}
int main() {
  DT7 rc;
  assert(rc.rc_l_switch() == kSwitchStateDown && rc.rc_lv() == 0);
  auto frame = Frame(); assert(rc.decodeChecked(frame.data(), frame.size()));
  auto cmd = ProcessRemote(rc, true, 0xfffffff0u, 5);
  assert(cmd.online && cmd.centered && !cmd.emergency_stop && cmd.mode == Mode::Depart);
  assert(!ProcessRemote(rc, true, 0, 100).online);
  assert(!rc.decodeChecked(frame.data(), 17));
  auto bad = Frame(0); assert(!rc.decodeChecked(bad.data(), 18));
  bad = Frame(3, 1, 0); assert(!rc.decodeChecked(bad.data(), 18));
  frame = Frame(2, 2, 1684); assert(rc.decodeChecked(frame.data(), 18));
  auto stop = ProcessRemote(rc, true, 0, 0);
  assert(stop.emergency_stop && stop.yaw == 0 && stop.mode == Mode::Gyro);

  RobotStateMachine fsm;
  Readiness ready{true, true, true, true, false};
  uint32_t now = 0;
  assert(fsm.state() == State::Dead);
  fsm.update(cmd, ready, now++); assert(fsm.state() == State::Preparing);
  for (int i=0; i<1999; ++i) fsm.update(cmd, ready, now++);
  assert(fsm.state() == State::Preparing && fsm.readyCount() == 1999);
  // Current bench readiness policy requires wheels only.
  auto missing = ready; missing.wheels_ready = false;
  fsm.update(cmd, missing, now++); assert(fsm.readyCount() == 0);
  for (int i=0; i<2000; ++i) fsm.update(cmd, ready, now++);
  assert(fsm.state() == State::Working);
  cmd.mode = Mode::Gyro; fsm.update(cmd, ready, now++); assert(fsm.mode() == Mode::Gyro);
  fsm.update(cmd, missing, now++); assert(fsm.state() == State::Preparing);
  fsm.update(stop, ready, now++); assert(fsm.state() == State::Dead);
  cmd.centered = false;
  for (int i=0; i<2100; ++i) fsm.update(cmd, ready, now++);
  assert(fsm.state() == State::Preparing && fsm.readyCount() == 0);
  cmd.centered = true;
  for (int i=0; i<2000; ++i) fsm.update(cmd, ready, now++);
  assert(fsm.state() == State::Working);
  now += 10; fsm.update(cmd, ready, now++); assert(fsm.state() == State::Preparing);
  ready.fault = true; fsm.update(cmd, ready, now++); assert(fsm.state() == State::Fault);
  fsm.update(stop, ready, now++); ready.fault = false;
  fsm.update(cmd, ready, now++); assert(fsm.state() == State::Fault);
  fsm.update(stop, ready, now++); fsm.update(cmd, ready, now++);
  assert(fsm.state() == State::Dead);
  RemoteCommand offline; fsm.update(offline, ready, now++); assert(fsm.state() == State::Dead);

  Pid pid({1, 2, 0.1f, 1, 0.5f});
  for(int i=0;i<10000;++i) assert(pid.update(100, 0, .001f) <= 1);
  assert(std::fabs(pid.integral()) < .001f);
  assert(pid.update(0, std::numeric_limits<float>::quiet_NaN(), .001f) == 0);
  assert(pid.update(1, 0, 0) == 0);
  Pid disabled; assert(disabled.update(10,0,.001f)==0);
  Pid bad_pid({1,1,0,1,-1}); assert(bad_pid.update(10,0,.001f)==0);

  ChassisController controller;
  cmd.mode = Mode::Depart; cmd.forward = 1; cmd.left = 0;
  Velocity v;
  for(int i=0;i<1100;++i) v=controller.update(cmd,true,1.57079632679f,.001f);
  assert(std::fabs(v.forward)<1e-5 && std::fabs(v.left-.5f)<1e-4 && v.yaw_rate==0);
  v=controller.update(stop,true,0,.001f); assert(v.forward==0 && v.left==0 && v.yaw_rate==0);
  cmd.mode = Mode::Follow; v=controller.update(cmd,true,.5f,.001f); assert(v.yaw_rate>0);
  v=controller.update(cmd,false,0,.001f); assert(v.yaw_rate==0);
  cmd.mode=Mode::Gyro; v=controller.update(cmd,true,0,.001f); assert(v.yaw_rate>0);
  v=controller.update(cmd,true,std::numeric_limits<float>::infinity(),.001f);
  assert(v.forward==0 && v.yaw_rate==0);
  ChassisConfig bad_config; bad_config.acceleration = -1;
  ChassisController invalid_controller(bad_config);
  v = invalid_controller.update(cmd,true,0,.001f);
  assert(v.forward==0 && v.left==0 && v.yaw_rate==0);

  GyroBias bias; float acc[]={0,0,9.80665f}, gyro[]={.01f,-.02f,.03f};
  for(uint32_t i=0;i<1999;++i) assert(!bias.update(acc,gyro,i));
  assert(bias.update(acc,gyro,1999)); assert(std::fabs(bias.bias()[2]-.03f)<1e-5);
  bias.reset(); bias.update(acc,gyro,0); bias.update(acc,gyro,0);
  assert(bias.samples()==1); // Duplicate cannot count twice.
  gyro[0]=1; bias.update(acc,gyro,1); assert(bias.samples()==0);
  gyro[0]=.01f; bias.update(acc,gyro,2); bias.update(acc,gyro,4); assert(bias.samples()==1);
  acc[0]=std::numeric_limits<float>::quiet_NaN(); assert(!bias.update(acc,gyro,5));

  M3508 motor({1,0,0,100,0}); assert(motor.calculate(20,true,0,.001f)==0);
  motor.setFeedback(10,25,0); assert(motor.calculate(20,true,1,.001f)==10);
  assert(motor.calculate(20,true,100,.001f)==0);
  motor.setFeedback(10,80,101); assert(motor.calculate(20,true,101,.001f)==0);
  uint8_t reply[8] = {0x1f,0xff,0xff,0x9c,0xfe,0x00,25,0};
  C620Feedback decoded;
  assert(DecodeC620(0x201, reply, 8, decoded, 10));
  assert(decoded.encoder==8191 && decoded.rpm==-100 && decoded.current_raw==-512);
  assert(!DecodeC620(0x200, reply, 8, decoded, 11));
  assert(!DecodeC620(0x201, reply, 7, decoded, 11));
  auto packed = PackC620({800,-800,0,16384});
  assert(packed[0]==3 && packed[1]==32 && packed[2]==0xfc && packed[3]==0xe0);
  std::puts("PASS: remote, readiness, emergency stop, faults, modes, PID, gyro bias, motor guards");
}
