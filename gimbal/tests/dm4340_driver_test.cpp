#include "dm4340_drv.hpp"
#include "dm4340_motor.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

CAN_HandleTypeDef hcan1{0}, hcan2{1};
namespace {
damiao::Payload sent{};
uint32_t sent_id=0;
uint8_t sent_length=0;
CAN_HandleTypeDef* sent_bus=nullptr;
damiao::Payload Feedback(uint8_t state=0) {
  // Midpoint position, near-zero speed/torque, pitch control ID 9.
  return {uint8_t((state<<4)|9),0x80,0,0x80,0x08,0,25,26};
}
bool Near(float a,float b) { return std::fabs(a-b)<.00001f; }
}
void CAN_Send_Msg(CAN_HandleTypeDef* bus,uint8_t* data,uint32_t id,uint8_t length) {
  assert(length<=8);
  sent={}; std::copy_n(data,length,sent.begin());
  sent_id=id; sent_length=length; sent_bus=bus;
}
int main() {
  DM4340_Motor_t legacy{};
  dm4340_joint_motor_init(&legacy,9,DM4340_MIT_MODE);
  auto frame=Feedback();
  dm4340_fbdata(&legacy,frame.data(),8);
  assert(legacy.para.id==9 && legacy.para.state==0);
  assert(legacy.para.p_int==32768 && legacy.para.v_int==2048 && legacy.para.t_int==2048);
  assert(Near(legacy.para.pos,float(32768)/65535*(2*DM4340_P_MAX)-DM4340_P_MAX));

  dm4340_enable_motor_mode(&hcan2,9,DM4340_MIT_MODE);
  assert(sent_bus==&hcan2 && sent_id==9 && sent_length==8 && sent==damiao::EnableFrame());
  dm4340_disable_motor_mode(&hcan2,9,DM4340_MIT_MODE);
  assert(sent_id==9 && sent==damiao::DisableFrame());
  dm4340_mit_ctrl(&hcan2,9,0,0,0,0,0);
  const damiao::Payload legacy_zero{0x7f,0xff,0x7f,0xf0,0,0,7,0xff};
  assert(sent==legacy_zero); // Original driver truncates, rather than rounding.
  dm4340_pos_speed_ctrl(&hcan2,9,.1f,.2f);
  float position=0,velocity=0;
  std::memcpy(&position,sent.data(),4); std::memcpy(&velocity,sent.data()+4,4);
  assert(sent_id==0x109 && position==.1f && velocity==.2f);
  dm4340_speed_ctrl(&hcan2,9,-.1f);
  std::memcpy(&velocity,sent.data(),4);
  assert(sent_id==0x209 && sent_length==4 && velocity==-.1f);

  // The adapter must use actual read-back ranges, including a different PMAX.
  damiao::MotorConfig c;
  c.control_id=9; c.feedback_id=0x19; c.limits={12.5f,10,28};
  c.direction=1; c.confirmed=true;
  damiao::DM4340 motor(c);
  assert(motor.onFeedback(0x19,frame.data(),8,0) && motor.captureZero(0));
  frame={0x19,0x81,0,0x90,0x09,0,25,26};
  damiao::Feedback reference;
  assert(damiao::DecodeFeedback(c.limits,frame.data(),8,reference));
  assert(motor.onFeedback(0x19,frame.data(),8,10));
  assert(Near(motor.raw().position,reference.position));
  assert(Near(motor.raw().velocity,reference.velocity) && Near(motor.raw().torque,reference.torque));
  assert(motor.joint(10).enabled && motor.joint(10).healthy);
  auto bad=frame; bad[0]=0x13;
  assert(!motor.onFeedback(0x19,bad.data(),8,11));
  bad=frame; bad[0]=0x29;
  assert(!motor.onFeedback(0x19,bad.data(),8,11));
  bad=frame; bad[6]=255;
  assert(!motor.onFeedback(0x19,bad.data(),8,11));
  assert(!motor.onFeedback(0x19,nullptr,8,11));
  assert(!motor.onFeedback(0x19,frame.data(),7,11));
  assert(!motor.onFeedback(0x13,frame.data(),8,11));
  frame[0]=0x89;
  assert(motor.onFeedback(0x19,frame.data(),8,11) && !motor.joint(11).healthy);
  assert(!motor.joint(111).online);

  // Runtime torque frames retain checked, range-aware encoding and finite guards.
  damiao::Payload expected,actual;
  assert(damiao::PackMit(c.limits,0,0,0,0,.1f,expected));
  assert(motor.torqueFrame(.1f,actual) && actual==expected);
  c.direction=-1;
  damiao::DM4340 reversed(c);
  frame=Feedback();
  assert(reversed.onFeedback(0x19,frame.data(),8,0) && reversed.captureZero(0));
  frame={0x19,0x7f,0,0x70,0x07,0,25,26}; // Negative motor motion/effort is positive joint motion/effort.
  assert(reversed.onFeedback(0x19,frame.data(),8,10));
  const auto joint=reversed.joint(10);
  assert(joint.position>0 && joint.velocity>0 && joint.torque>0);
  assert(damiao::PackMit(c.limits,0,0,0,0,-.1f,expected));
  assert(reversed.torqueFrame(.1f,actual) && actual==expected);
  c.absolute_position=true;
  damiao::DM4340 absolute(c);
  frame={0x09,0x7f,0,0x80,0x08,0,25,26};
  assert(damiao::DecodeFeedback(c.limits,frame.data(),8,reference));
  assert(absolute.onFeedback(0x19,frame.data(),8,0));
  const float absolute_angle=absolute.joint(0).position;
  assert(absolute.joint(0).zeroed && Near(absolute_angle,-reference.position));
  assert(!absolute.captureZero(0) && absolute.joint(0).position==absolute_angle);
  assert(!absolute.joint(100).online && !absolute.joint(100).zeroed);
  assert(absolute.onFeedback(0x19,frame.data(),8,150));
  assert(absolute.joint(150).zeroed && absolute.joint(150).position==absolute_angle);
  damiao::DM4340 restarted(c);
  assert(restarted.onFeedback(0x19,frame.data(),8,200));
  assert(restarted.joint(200).zeroed && restarted.joint(200).position==absolute_angle);
  std::cout << "Unmodified DM4340 driver and range-aware pitch adapter passed\n";
}
