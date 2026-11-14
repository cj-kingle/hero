#include "lower_runtime.hpp"
#include "chassis_motor_task.hpp"
#include "board_link.hpp"
#include "iwdg.h"
#include "yaw_calibration.hpp"
#include "imu_task.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
CAN_HandleTypeDef hcan1,hcan2;
UART_HandleTypeDef huart3;
IWDG_HandleTypeDef hiwdg;
volatile uint32_t dbg_state=0,dbg_ready_count=0,dbg_mode=0,dbg_rc_online=0,dbg_estop=0,dbg_centered=0;
volatile uint32_t dbg_left_switch=0,dbg_right_switch=0,dbg_fault=0,dbg_rc_frames=0,dbg_rc_invalid=0;
volatile uint32_t dbg_motor_online_mask=0,dbg_motor_online=0,dbg_can_error=0;
volatile uint32_t dbg_can_rx=0,dbg_encoder=0;
volatile float dbg_motor_target_rpm[4]{},dbg_motor_actual_rpm[4]{},dbg_motor_current[4]{};
volatile float dbg_motor_feedback_current_a[4]{},dbg_motor_mechanical_angle_deg[4]{};
volatile float dbg_target_rpm=0,dbg_actual_rpm=0,dbg_current=0,dbg_temperature=0;
volatile float dbg_mechanical_angle_deg=0,dbg_feedback_current=0,dbg_feedback_current_a=0,dbg_speed_kp=0;
volatile uint32_t dbg_chassis_imu_mounting_confirmed=1;
namespace calibration_test {
std::array<uint32_t,64> flash=[] {std::array<uint32_t,64> a{};a.fill(0xffffffff);return a;}();
uint32_t Read(size_t i){return flash.at(i);}
bool Write(size_t i,uint32_t v){if(flash.at(i)!=0xffffffff)return false;flash[i]=v;return true;}
}
namespace chassis {
bool LoadYawCalibration(float& v){return CalibrationJournal{64,calibration_test::Read,calibration_test::Write}.load(v);}
CalibrationSave SaveYawCalibration(float v){return CalibrationJournal{64,calibration_test::Read,calibration_test::Write}.save(v);}
}
namespace {
uint32_t now=0,feeds=0,abort_wheels=0;
uint8_t* buffer=nullptr;
bool send_fail=false,imu_ready=false,status_on=true;
bool hard_tx_error=false;
bool upper_imu_mode_confirmed=true;
unsigned busy_upper_remaining=0,busy_wheels_remaining=0;
uint32_t can_error=0;
bool chassis_imu_ready=true;
float chassis_yaw_rate=0;
uint8_t status_sequence=0;
unsigned left_switch=2,right_switch=1;
int forward=0,left=0,yaw=0,pitch=0;
float motor_yaw=.7f;
gimbal::Command sent_command;
std::array<int16_t,4> current{};
void Remote() {
  std::fill_n(buffer,18,0);
  const unsigned channels[]{unsigned(1024-yaw),unsigned(1024+pitch),unsigned(1024-left),unsigned(1024+forward)};
  for(unsigned c=0;c<4;++c) for(unsigned b=0;b<11;++b)
    buffer[(11*c+b)/8]|=((channels[c]>>b)&1u)<<((11*c+b)%8);
  buffer[5]|=(left_switch<<6)|(right_switch<<4);buffer[17]=4;
  assert(LowerOnUart(&huart3,18));
}
void Feedback(bool drive_ready=true) {
  CAN_RxHeaderTypeDef h;
  for(unsigned i=0;i<4;++i) {
    h.StdId=0x201+i;
    const uint8_t f[8]{0,0,0,0,0,0,30,0};LowerOnCan(&hcan2,&h,f);
  }
  if(status_on && now%10==0) {
    gimbal::BoardStatus s;
    s.yaw_valid=true;s.drive=drive_ready;s.imu_ready=imu_ready;
    s.imu_mode=upper_imu_mode_confirmed && sent_command.mode==gimbal::Mode::Imu;
    s.yaw_angle=motor_yaw;s.sequence=++status_sequence;
    std::array<uint8_t,8> data;assert(gimbal::EncodeStatus(s,data));
    h.StdId=gimbal::kBoardStatusId;LowerOnCan(&hcan1,&h,data.data());
  }
}
void Advance(unsigned duration,bool poll=true,bool rc_on=true) {
  for(unsigned i=0;i<duration;++i) {
    ++now;Feedback();if(rc_on && now%7==0)Remote();
    LowerSafetyTick();if(poll)LowerPoll();
  }
}
void Prepare() {
  now=0;feeds=abort_wheels=0;send_fail=false;imu_ready=false;status_on=true;
  hard_tx_error=false;busy_upper_remaining=busy_wheels_remaining=0;can_error=0;
  upper_imu_mode_confirmed=true;
  left_switch=2;right_switch=1;forward=left=yaw=pitch=0;motor_yaw=.7f;
  chassis_imu_ready=true;chassis_yaw_rate=0;dbg_chassis_imu_mounting_confirmed=1;
  LowerInit();Remote();Feedback(false);LowerPoll();
  Advance(350);
  dbg_capture_yaw_forward=1;Advance(1);
  assert(dbg_yaw_aligned && std::fabs(dbg_yaw_forward_offset_rad-.7f)<.001f);
  left_switch=1;Advance(2100);assert(dbg_state==uint32_t(chassis::State::Working));
}
void AllZero() {for(auto c:current)assert(c==0);}
void Protocol() {
  gimbal::BoardStatus s;s.yaw_valid=s.drive=s.imu_ready=s.imu_mode=true;
  s.yaw_angle=13;s.reasons=4096;s.sequence=255;
  std::array<uint8_t,8> data;assert(gimbal::EncodeStatus(s,data));
  gimbal::StatusReceiver r;assert(r.receive(data.data(),8,0xfffffff0u));
  assert(!r.receive(data.data(),8,0xfffffff1u));
  gimbal::BoardStatus d;assert(r.snapshot(20,d) && d.reasons==4096);
  assert(!r.snapshot(84,d)); // wrap-safe timeout, repeated packet cannot refresh it
  s.sequence=0;assert(gimbal::EncodeStatus(s,data));assert(r.receive(data.data(),8,85));
  data[4]^=1;assert(!r.receive(data.data(),8,86));assert(!r.snapshot(86,d));
}
}
uint32_t HAL_GetTick(){return now;}
bool ImuReady(uint32_t){return chassis_imu_ready;}
bool ImuGetFeedback(uint32_t t,chassis::ChassisImuFeedback& f){f={0,chassis_yaw_rate,t,chassis_imu_ready};return f.ready;}
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef*){return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*){return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef*){return can_error;}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef* b,uint32_t){if(b==&hcan2)++abort_wheels;return HAL_OK;}
void CanFilter_Init(CAN_HandleTypeDef*){}
HAL_StatusTypeDef CAN_TrySend(CAN_HandleTypeDef* b,const uint8_t* data,uint32_t id,uint8_t n) {
  assert(n==8);if(hard_tx_error)return HAL_ERROR;if(send_fail)return HAL_BUSY;
  auto& busy=b==&hcan1 ? busy_upper_remaining : busy_wheels_remaining;
  if(busy) {--busy;return HAL_BUSY;}
  if(b==&hcan1){assert(id==gimbal::kBoardCommandId);assert(gimbal::DecodeCommand(data,8,sent_command));}
  else {assert(b==&hcan2 && id==0x200);for(unsigned i=0;i<4;++i) current[i]=int16_t((uint16_t(data[2*i])<<8)|data[2*i+1]);}
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*,uint8_t* p,uint16_t n){assert(n==18);buffer=p;return HAL_OK;}
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*){return 0;}
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef*){++feeds;return HAL_OK;}
int main() {
  // Completed arming is not lost when runtime feedback pauses. Both automatic
  // modes must resume ramped output as soon as all feedback is valid again,
  // including when the operator continues holding a translation/yaw command.
  for(unsigned mode_switch : {3u,2u}) {
    Prepare();imu_ready=true;right_switch=mode_switch;Advance(2200);
    forward=300;yaw=200;Advance(100);
    assert(dbg_state==uint32_t(chassis::State::Working));
    chassis_imu_ready=false;Advance(1);
    assert(!sent_command.emergency_stop && dbg_state==uint32_t(chassis::State::Working));
    chassis_imu_ready=true;Advance(1);
    assert(dbg_state==uint32_t(chassis::State::Working) && dbg_ready_count==2000);
    Advance(20);
    bool moving=false;for(auto c:current)moving|=c!=0;assert(moving);
    chassis_imu_ready=false;imu_ready=false;upper_imu_mode_confirmed=false;
    Advance(200);
    assert(dbg_state==uint32_t(chassis::State::Working) && !sent_command.emergency_stop);
    moving=false;for(auto c:current)moving|=c!=0;assert(moving);
    chassis_imu_ready=true;imu_ready=true;upper_imu_mode_confirmed=true;
    // Bounded 11..19 ms foreground jitter must neither revoke arming nor
    // turn an ordinary scheduling delay into a latched stop of the upper.
    for(unsigned gap : {11u,19u}) {
      Advance(gap-1,false);Advance(1);
      assert(dbg_state==uint32_t(chassis::State::Working) && !(dbg_lower_reasons&64u));
      assert(!sent_command.emergency_stop);
    }
    // An actual user estop still revokes arming and requires neutral preparation.
    left_switch=2;Remote();Advance(1);AllZero();left_switch=1;Advance(2100);
    assert(dbg_state!=uint32_t(chassis::State::Working));
  }
  // A delayed upper-mode acknowledgement inhibits output, not completed
  // preparation. Once acknowledged, deflected sticks must work immediately.
  for(unsigned mode_switch : {3u,2u}) {
    Prepare();right_switch=mode_switch;imu_ready=false;upper_imu_mode_confirmed=false;
    Advance(30);
    forward=300;yaw=200;Advance(30);
    assert(dbg_state==uint32_t(chassis::State::Working) && !sent_command.emergency_stop);
    imu_ready=true;Advance(20);assert(dbg_state==uint32_t(chassis::State::Working));
    upper_imu_mode_confirmed=true;Advance(20);
    assert(dbg_state==uint32_t(chassis::State::Working));
    bool moving=false;for(auto c:current)moving|=c!=0;assert(moving);
  }
  calibration_test::flash.fill(0xffffffff); // initial-boot cases below require an empty calibration journal
  right_switch=1;imu_ready=false; // restore the original cold-boot fixture after the new automatic-mode cases
  // A valid neutral RC at boot must not need a down/up switch acknowledgement
  // just because the lower initially had no command to forward.
  left_switch=1;forward=left=yaw=pitch=0;LowerInit();Remote();Advance(2200);
  assert(dbg_cfg_gyro_yaw_rate==chassis::config::kMotion.gyro_rate && dbg_cfg_follow_yaw_rate==4.f);
  assert(dbg_cfg_gyro_yaw_acceleration==chassis::config::kMotion.gyro_yaw_acceleration);
  assert(dbg_state==uint32_t(chassis::State::Working) && !sent_command.emergency_stop);
  assert(!(dbg_lower_reasons&8192));
  left_switch=2;
  // No calibration record is required to enter Working or use the configured zero.
  LowerInit();Remote();Feedback(false);LowerPoll();left_switch=1;motor_yaw=0;
  Advance(2100);assert(dbg_state==uint32_t(chassis::State::Working) && !dbg_yaw_calibrated);
  assert(dbg_yaw_forward_offset_rad==chassis::config::kYawForwardOffsetRad);
  Protocol();Prepare();forward=400;Advance(300);
  assert(current[0]>0 && current[1]>0 && current[2]<0 && current[3]<0);
  for(auto c:current)assert(std::abs(int(c))<=chassis::config::kNormalCurrentLimitRaw);
  left_switch=2;Remote();AllZero();assert(sent_command.emergency_stop && abort_wheels>0);
  LowerInit();assert(dbg_yaw_calibrated && std::fabs(dbg_yaw_forward_offset_rad-.7f)<.001f); // power-cycle reload
  Prepare();motor_yaw=.9f;Advance(350);
  dbg_capture_yaw_forward=1;Advance(1);
  assert(dbg_yaw_calibration_result==1 && std::fabs(dbg_yaw_forward_offset_rad-.9f)<.001f);
  assert(!sent_command.emergency_stop); // calibration works while upper remains enabled
  assert(dbg_state==uint32_t(chassis::State::Preparing));AllZero();
  Advance(2100);assert(dbg_state==uint32_t(chassis::State::Working));
  forward=300;Advance(50);dbg_capture_yaw_forward=1;Advance(1);
  assert(dbg_yaw_calibration_result==2); // moving request rejected
  Prepare();motor_yaw=.7f+1.570796327f;forward=400;Advance(500);
  assert(current[0]<0 && current[1]>0 && current[2]>0 && current[3]<0); // Depart follows gimbal heading, no chassis spin
  // A single short or malformed UART event must not send an artificial
  // whole-vehicle estop while the last VALID remote frame is still fresh.
  Prepare();forward=400;Advance(300);Remote();
  const auto before_bad_frame_feeds=feeds;
  assert(LowerOnUart(&huart3,9));
  assert(feeds==before_bad_frame_feeds);Advance(1,true,false);
  assert(!sent_command.emergency_stop && dbg_state==uint32_t(chassis::State::Working));
  std::fill_n(buffer,18,0);assert(LowerOnUart(&huart3,18));Advance(10,true,false);
  assert(!sent_command.emergency_stop && dbg_state==uint32_t(chassis::State::Working));
  // Repeated bad data cannot extend the original freshness deadline.
  for(unsigned i=0;i<100;++i){assert(LowerOnUart(&huart3,18));Advance(1,true,false);}
  AllZero();assert(!sent_command.online && sent_command.emergency_stop);
  Prepare();forward=400;Advance(300);left_switch=2;Remote();AllZero();
  assert(sent_command.emergency_stop); // a valid user estop still acts in the RX callback
  Prepare();forward=400;Advance(300);Advance(110,true,false);AllZero();assert(!sent_command.online);
  Prepare();forward=400;Advance(300);const auto old_feeds=feeds;Advance(40,false);AllZero();assert(sent_command.emergency_stop);
  const auto frozen_feeds=feeds;Advance(40,false);assert(feeds==frozen_feeds && frozen_feeds>=old_feeds);
  Prepare();forward=400;Advance(300);status_on=false;Advance(110);AllZero();assert(!dbg_upper_online);
  Prepare();right_switch=3;Advance(2200);assert(dbg_state==uint32_t(chassis::State::Working));
  assert(sent_command.mode==gimbal::Mode::Imu && (dbg_lower_reasons&32));
  imu_ready=true;Advance(2200);assert(dbg_state==uint32_t(chassis::State::Working));
  motor_yaw=1.f;Advance(300);assert(current[0]<0 && current[1]<0 && current[2]<0 && current[3]<0);
  motor_yaw=dbg_yaw_forward_offset_rad;chassis_yaw_rate=.003f;Advance(1500);
  assert(std::fabs(dbg_relative_yaw_deg)<.1f && dbg_chassis_target_yaw_rate==0 && dbg_chassis_applied_yaw_rate==0);
  AllZero(); // lifted chassis, head aligned and small gyro bias: no persistent wheel effort
  imu_ready=false;Advance(20);AllZero();assert(dbg_state==uint32_t(chassis::State::Working));
  Prepare();right_switch=2;imu_ready=true;Advance(2200);assert(dbg_state==uint32_t(chassis::State::Working));
  Advance(500);assert(current[0]<0 && current[1]<0 && current[2]<0 && current[3]<0); // autonomous gyro, centered sticks
  Advance(400);
  assert(std::fabs(dbg_chassis_target_yaw_rate-chassis::config::kMotion.gyro_rate)<.0001f);
  assert(dbg_chassis_target_yaw_rate==chassis::config::kMotion.gyro_rate &&
      dbg_chassis_applied_yaw_rate<=chassis::config::kGyroYawRateOutputLimit+.0001f);
  assert(std::fabs(dbg_motor_target_rpm[0])>chassis::wheel_config::kMaxMotorRpm);
  for(unsigned i=0;i<4;++i)assert(std::fabs(dbg_motor_target_rpm[i])<=chassis::config::kGyroMaxMotorRpm+.01f);
  assert(dbg_mode==2 && dbg_chassis_target_yaw_rate>0 && dbg_chassis_actual_yaw_rate==0);
  assert(dbg_lower_reasons==0); // lifted chassis: zero IMU rotation still produces nonzero wheel effort
  right_switch=1;Advance(2200);AllZero();assert(dbg_state==uint32_t(chassis::State::Working));
  forward=400;Advance(300);
  assert(dbg_chassis_forward_input>0 && current[0]>0 && current[1]>0 && current[2]<0 && current[3]<0);
  // Return from autonomous gyro with a deflected stick: translation must remain available.
  Prepare();right_switch=2;imu_ready=true;Advance(2200);assert(dbg_state==uint32_t(chassis::State::Working));
  right_switch=3;forward=400;Advance(40);
  assert(dbg_state==uint32_t(chassis::State::Working) && dbg_mode==1 && dbg_chassis_forward_input>0);
  // Working is checked above after 40 ms. Allow the unchanged wheel ramp
  // enough time to reverse from the faster gyro speed before checking signs.
  Advance(1800);
  assert(current[0]>0 && current[1]>0 && current[2]<0 && current[3]<0);
  right_switch=1;Advance(300);
  assert(dbg_state==uint32_t(chassis::State::Working) && dbg_mode==0);
  assert(current[0]>0 && current[1]>0 && current[2]<0 && current[3]<0);
  Prepare();imu_ready=true;Advance(20);right_switch=2;forward=300;Advance(40);
  assert(dbg_state==uint32_t(chassis::State::Working)); // normal switch needs no neutral dwell
  left_switch=2;Remote();Advance(10);left_switch=1;Advance(2200);AllZero();
  assert(dbg_lower_reasons&4096); // a real estop still requires neutral preparation
  forward=0;Advance(2200);assert(dbg_state==uint32_t(chassis::State::Working));
  Prepare();chassis_yaw_rate=.1f;Advance(100);
  assert(dbg_chassis_target_yaw_rate==0 && dbg_chassis_applied_yaw_rate<0); // Depart actively rejects measured spin
  assert(current[0]>0 && current[1]>0 && current[2]>0 && current[3]>0);
  chassis_imu_ready=false;Advance(100);AllZero();assert(dbg_state==uint32_t(chassis::State::Working) && (dbg_lower_reasons&1024));
  Advance(20);assert(!sent_command.emergency_stop); // chassis sensor loss must not repeatedly disable upper
  Prepare();dbg_chassis_imu_mounting_confirmed=0;Advance(1);AllZero();assert(dbg_state==uint32_t(chassis::State::Working) && (dbg_lower_reasons&1024));
  dbg_chassis_imu_mounting_confirmed=1;Advance(2100);assert(dbg_state==uint32_t(chassis::State::Working));
  Prepare();Advance(30,false);AllZero();Advance(2200);
  assert(sent_command.emergency_stop && (dbg_lower_reasons&8192)); // foreground freeze cannot auto-release upper
  left_switch=2;Remote();Advance(20);left_switch=1;Advance(2200);
  assert(!sent_command.emergency_stop && dbg_state==uint32_t(chassis::State::Working));
  // Normal yaw stick input must remain an enabled upper command in every
  // chassis mode, for both left-switch operating positions and both directions.
  for(unsigned left_position : {1u,3u}) for(unsigned right_position : {1u,3u,2u}) {
    Prepare();imu_ready=true;left_switch=left_position;right_switch=right_position;
    Advance(2200);assert(dbg_state==uint32_t(chassis::State::Working));
    for(int yaw_input : {-660,-330,330,660}) {
      yaw=yaw_input;Advance(150);
      assert(sent_command.online && !sent_command.emergency_stop && !sent_command.centered);
      assert((sent_command.yaw>0)==(yaw_input>0));
      assert(sent_command.mode==((right_position!=1 || left_position==3) ? gimbal::Mode::Imu : gimbal::Mode::Joint));
      assert(dbg_state==uint32_t(chassis::State::Working));
    }
  }
  Prepare();imu_ready=true;Advance(20);forward=300;Advance(100);
  for(unsigned mode : {3u,2u,1u,2u,3u}) {
    right_switch=mode;Advance(40);
    assert(dbg_state==uint32_t(chassis::State::Working) && dbg_ready_count==2000);
    assert(!sent_command.emergency_stop);
  }
  for(unsigned mode : {1u,3u}) {
    Prepare();imu_ready=true;right_switch=mode;Advance(2200);
    forward=660;Advance(1600);
    for(unsigned i=0;i<4;++i) {
      assert(std::fabs(std::fabs(dbg_wheel_target_rpm[i])-chassis::config::kTranslationWheelRpm)<.01f);
      assert(std::fabs(dbg_motor_target_rpm[i])<=chassis::config::kNormalMaxMotorRpm+.01f);
      assert(dbg_wheel_actual_rpm[i]==0);
    }
  }
  Prepare();forward=400;Advance(300);
  const auto before_speed_event=current;
  CAN_RxHeaderTypeDef speed_header;speed_header.StdId=0x201;
  const int high_rpm=int(chassis::config::kOverspeedMotorRpm+1);
  const uint8_t speed_data[8]{0,0,uint8_t(high_rpm>>8),uint8_t(high_rpm),0,0,30,0};
  LowerOnCan(&hcan2,&speed_header,speed_data);LowerSafetyTick();
  assert(current==before_speed_event); // timer does not apply the removed speed latch
  Advance(1);assert(dbg_state==uint32_t(chassis::State::Working));
  Prepare();forward=400;Advance(200);busy_wheels_remaining=3;Advance(6);
  assert(!dbg_fault && dbg_state==uint32_t(chassis::State::Working) && !sent_command.emergency_stop);
  assert(dbg_last_stop_reasons==0 && busy_wheels_remaining==0);
  busy_upper_remaining=3;Advance(20);
  assert(!dbg_fault && dbg_state==uint32_t(chassis::State::Working) && !sent_command.emergency_stop);
  assert(busy_upper_remaining==0);
  can_error=HAL_CAN_ERROR_EWG|HAL_CAN_ERROR_ACK;Advance(10);
  assert(!dbg_fault && dbg_state==uint32_t(chassis::State::Working) && dbg_can_error!=0);
  can_error=HAL_CAN_ERROR_BOF;Advance(1);
  assert(dbg_fault && (dbg_last_stop_reasons&128u));AllZero();
  Prepare();forward=400;Advance(200);busy_wheels_remaining=1;Advance(1);
  assert(!dbg_fault);Advance(chassis::config::kCanBusyGraceMs,false);LowerPoll();
  assert(dbg_fault);AllZero(); // success after the deadline must not revive a delayed nonzero command
  Prepare();forward=400;Advance(200);busy_upper_remaining=100;Advance(30);
  assert(dbg_fault && (dbg_last_stop_reasons&128u));AllZero();
  Prepare();forward=400;Advance(200);hard_tx_error=true;Advance(1);assert(dbg_fault);
  for(auto c:chassis::four_wheel_status.commands)assert(c==0);
  Prepare();forward=400;Advance(200);busy_wheels_remaining=2;left_switch=2;Remote();Advance(1);
  assert(sent_command.emergency_stop && !dbg_fault);AllZero(); // stop stays zero; brief mailbox busy no longer permanently faults
  Prepare();forward=400;Advance(200);send_fail=true;Advance(chassis::config::kCanBusyGraceMs+2);assert(dbg_fault);
  for(auto c:chassis::four_wheel_status.commands)assert(c==0);
  std::cout<<"Three-mode runtime: alignment, frame CRC/freshness, follow/gyro, IMU gate, estop, task/link loss, TX failure passed\n";
}
