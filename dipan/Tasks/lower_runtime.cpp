#include "lower_runtime.hpp"
#include "fault_log.h"
#include "chassis_motor_task.hpp"
#include "board_link.hpp"
#include "DT7.hpp"
#include "iwdg.h"
#include "usart.h"
#include "yaw_calibration.hpp"
#include "imu_task.hpp"
#include <algorithm>
#include <cmath>

volatile uint32_t dbg_upper_online=0,dbg_upper_ready=0,dbg_upper_reasons=0,dbg_imu_ready=0;
volatile uint32_t dbg_yaw_aligned=0,dbg_capture_yaw_forward=0,dbg_lower_reasons=0;
volatile uint32_t dbg_yaw_calibrated=0,dbg_yaw_calibration_result=0;
volatile uint32_t dbg_chassis_imu_ready=0;
volatile uint32_t dbg_last_stop_reasons=0;
volatile float dbg_chassis_target_yaw_rate=0,dbg_chassis_actual_yaw_rate=0,dbg_chassis_applied_yaw_rate=0;
volatile float dbg_chassis_forward_input=0,dbg_chassis_left_input=0;
volatile float dbg_cfg_gyro_yaw_rate=0,dbg_cfg_follow_yaw_rate=0,dbg_cfg_gyro_yaw_acceleration=0;
volatile float dbg_wheel_target_rpm[4]{},dbg_wheel_actual_rpm[4]{};
volatile float dbg_relative_yaw_deg=0,dbg_yaw_forward_offset_rad=chassis::config::kYawForwardOffsetRad;
namespace {
namespace rc=hello_world::devices::remote_control;
rc::DT7 remote;
uint8_t buffer[rc::kRcRxDataLen]{};
std::array<chassis::C620Feedback,4> wheels{};
gimbal::StatusReceiver upper;
chassis::FourWheelTest drive;
bool active=false,received=false,heartbeat=false,transport_fault=false,stopped=true;
bool upper_stopped=true;
bool upper_stop_latched=false;
bool operation_started=false; // startup no-command is not a post-operation fault
uint32_t received_ms=0,last_poll=0,last_send=0,frames=0,invalid=0;
uint8_t sequence=0;
struct TxRetry {bool busy=false;uint32_t since=0;};
TxRetry upper_retry,wheel_retry;
bool RetryExpired(const TxRetry& retry,uint32_t now) {
  return retry.busy && uint32_t(now-retry.since)>=chassis::config::kCanBusyGraceMs;
}
bool AcceptTx(HAL_StatusTypeDef status,TxRetry& retry,uint32_t now) {
  if(status==HAL_OK) {retry={};return true;}
  if(status!=HAL_BUSY) {transport_fault=true;return false;}
  if(!retry.busy)retry={true,now};
  if(uint32_t(now-retry.since)>=chassis::config::kCanBusyGraceMs)transport_fault=true;
  return false;
}
float forward_offset=chassis::config::kYawForwardOffsetRad;
uint32_t yaw_stable_since=0;
float yaw_stable_anchor=0;
bool yaw_stability_started=false;
uint32_t Lock(){auto p=__get_PRIMASK();__disable_irq();return p;}
void Unlock(uint32_t p){__set_PRIMASK(p);}
bool Start(CAN_HandleTypeDef* bus,uint32_t fifo) {
  bus->Init.AutoBusOff=ENABLE;bus->Init.AutoRetransmission=DISABLE;
  if(HAL_CAN_Init(bus)!=HAL_OK) return false;
  CanFilter_Init(bus);
  return HAL_CAN_Start(bus)==HAL_OK && HAL_CAN_ActivateNotification(bus,fifo)==HAL_OK;
}
gimbal::Command ToUpper(const chassis::RemoteCommand& c,bool force_stop=false) {
  gimbal::Command g;
  g.online=c.online;g.emergency_stop=c.emergency_stop||force_stop||upper_stop_latched;
  // Right UP: independent mode, left UP joint / MID IMU. Automatic chassis
  // modes require inertial yaw stabilization, never silently use joint control.
  g.mode=c.mode!=chassis::Mode::Depart || remote.rc_l_switch()==rc::kSwitchStateMid ?
      gimbal::Mode::Imu : gimbal::Mode::Joint;
  g.yaw=g.emergency_stop?0:c.yaw;g.pitch=g.emergency_stop?0:c.pitch;
  g.centered=c.centered;g.sequence=sequence;return g;
}
bool SendUpper(const gimbal::Command& c) {
  if(!c.emergency_stop && c.online && RetryExpired(upper_retry,HAL_GetTick())) {
    transport_fault=true;return false; // no late nonzero command after the retry deadline
  }
  std::array<uint8_t,8> data;
  if(c.emergency_stop && !upper_stopped)
    HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
  if(!gimbal::EncodeCommand(c,data)) {transport_fault=true;return false;}
  const bool ok=AcceptTx(CAN_TrySend(&hcan1,data.data(),gimbal::kBoardCommandId,8),
      upper_retry,HAL_GetTick());
  if(ok) {
    upper_stopped=c.emergency_stop;
    if(c.online && !c.emergency_stop)operation_started=true;
  }
  return ok;
}
void StopWheels() {
  if(!stopped) {
    HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
  }
  stopped=true;
  const std::array<uint8_t,8> zero{};
  (void)AcceptTx(CAN_TrySend(&hcan2,zero.data(),0x200,8),wheel_retry,HAL_GetTick());
}
void Stop(const chassis::RemoteCommand& c) {
  if(operation_started && (!c.online || !c.emergency_stop)) upper_stop_latched=true;
  StopWheels();
  if(!SendUpper(ToUpper(c,true))) transport_fault=true;
}
}
void LowerInit() {
  const auto p=Lock();
  remote=rc::DT7{};wheels={};upper={};drive=chassis::FourWheelTest{};
  received=heartbeat=false;transport_fault=false;stopped=false;frames=invalid=0;sequence=0;
  upper_stopped=false;
  upper_stop_latched=false;operation_started=false;
  upper_retry=wheel_retry={};dbg_last_stop_reasons=0;dbg_state=uint32_t(chassis::State::Dead);
  dbg_cfg_gyro_yaw_rate=chassis::config::kMotion.gyro_rate;
  dbg_cfg_follow_yaw_rate=chassis::config::kMotion.max_yaw_rate;
  dbg_cfg_gyro_yaw_acceleration=chassis::config::kMotion.gyro_yaw_acceleration;
  forward_offset=chassis::config::kYawForwardOffsetRad;dbg_capture_yaw_forward=0;
  yaw_stability_started=false;yaw_stable_since=0;yaw_stable_anchor=0;
  dbg_yaw_calibrated=chassis::LoadYawCalibration(forward_offset);
  dbg_yaw_aligned=1;dbg_yaw_calibration_result=0;
  last_poll=last_send=HAL_GetTick();active=true;
  Unlock(p);
  const bool a=Start(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  const bool b=Start(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
  if(!a || !b || HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buffer,sizeof(buffer))!=HAL_OK) transport_fault=true;
  Stop({});
}
void LowerOnCan(CAN_HandleTypeDef* bus,const CAN_RxHeaderTypeDef* h,const uint8_t* data) {
  if(!active || !h || h->IDE!=CAN_ID_STD || h->RTR!=CAN_RTR_DATA) return;
  const auto p=Lock();const auto now=HAL_GetTick();
  if(bus==&hcan1 && h->StdId==gimbal::kBoardStatusId && upper.receive(data,h->DLC,now)) dbg_can_rx=dbg_can_rx+1;
  if(bus==&hcan2 && h->StdId>=0x201 && h->StdId<=0x204) {
    chassis::C620Feedback f;
    if(chassis::DecodeC620(h->StdId,data,h->DLC,f,now)) { wheels[h->StdId-0x201]=f;dbg_can_rx=dbg_can_rx+1; }
  }
  Unlock(p);
}
bool LowerOnUart(UART_HandleTypeDef* uart,uint16_t size) {
  if(!active || uart!=&huart3) return false;
  if(HAL_UARTEx_GetRxEventType(uart)==HAL_UART_RXEVENT_HT) return true;
  const auto p=Lock();const auto now=HAL_GetTick();
  if(remote.decodeChecked(buffer,size)) {
    received=true;received_ms=now;++sequence;++frames;
    const auto c=chassis::ProcessRemote(remote,true,received_ms,now);
    if(c.emergency_stop) {upper_stop_latched=false;Stop(c);}
    if(heartbeat && uint32_t(now-last_poll)<20u) HAL_IWDG_Refresh(&hiwdg);
  } else {
    // A partial/invalid callback is not a new remote command. Retain only the
    // LAST VALID frame until its original 100 ms deadline; invalid bytes must
    // never refresh that timestamp, the command sequence, or the watchdog.
    ++invalid;
    const auto c=chassis::ProcessRemote(remote,received,received_ms,now);
    if(!c.online) {received=false;Stop(c);}
  }
  Unlock(p);
  if(uart->hdmarx && uart->hdmarx->Init.Mode!=DMA_CIRCULAR &&
      HAL_UARTEx_ReceiveToIdle_DMA(uart,buffer,sizeof(buffer))!=HAL_OK) transport_fault=true;
  return true;
}
void LowerSafetyTick() {
  if(!active) return;
  const auto p=Lock();const auto now=HAL_GetTick();
  const auto c=chassis::ProcessRemote(remote,received,received_ms,now);
  gimbal::BoardStatus s;const bool online=upper.snapshot(now,s);
  bool wheels_ok=true;
  for(const auto& f:wheels) wheels_ok &= f.received && uint32_t(now-f.updated_ms)<100u &&
      f.temperature<70 && (!chassis::config::kWheelOverspeedTripEnabled ||
      std::fabs(float(f.rpm))<=chassis::config::kOverspeedMotorRpm);
  // Stop is independent of foreground execution. During preparation upper may
  // still enable/hold; do not repeatedly estop it just because wheels are not ready.
  if(!c.online || c.emergency_stop || transport_fault || (heartbeat && uint32_t(now-last_poll)>=20u)) Stop(c);
  else if(!stopped && (!online || !s.drive || !wheels_ok ||
      (chassis::config::kYawRateControlEnabled && chassis::config::kImuLossTripEnabled &&
       (!ImuReady(now) || !dbg_chassis_imu_mounting_confirmed)))) StopWheels();
  Unlock(p);
}
void LowerPoll() {
  if(!active) return;
  const auto p=Lock();const uint32_t now=HAL_GetTick();
  if(heartbeat && now==last_poll){Unlock(p);return;}
  const bool was_working=dbg_state==uint32_t(chassis::State::Working);
  const uint32_t log_gap=heartbeat ? uint32_t(now-last_poll) : 0u;
  const bool timely=!heartbeat || uint32_t(now-last_poll)<=chassis::config::kMaxControlGapMs;
  last_poll=now;heartbeat=true;
  const auto c=chassis::ProcessRemote(remote,received,received_ms,now);
  gimbal::BoardStatus s;const bool online=upper.snapshot(now,s);
  if(online && s.drive)operation_started=true;
  bool stationary=true;for(const auto& f:wheels) stationary &= f.received &&
      uint32_t(now-f.updated_ms)<100u && std::abs(int(f.rpm))<10;
  if(!c.online || !c.centered || !stationary || !online || !s.yaw_valid) yaw_stability_started=false;
  else if(!yaw_stability_started || std::fabs(std::remainder(s.yaw_angle-yaw_stable_anchor,6.283185307f))>.003f) {
    yaw_stability_started=true;yaw_stable_since=now;yaw_stable_anchor=s.yaw_angle;
  }
  bool calibration_hold=false;
  if(dbg_capture_yaw_forward) {
    dbg_capture_yaw_forward=0;
    dbg_yaw_calibration_result=2; // not centered/stationary, wrong mode, or stale angle
    if(!transport_fault && c.online && c.centered && c.mode==chassis::Mode::Depart &&
        stationary && online && s.yaw_valid && yaw_stability_started && uint32_t(now-yaw_stable_since)>=300u) {
      // Keep upper enabled so it can hold yaw/pitch and supply fresh angle data.
      // Cancel wheel effort BEFORE Flash access and reset chassis preparation below.
      calibration_hold=true;
      if(!stopped && HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2)!=HAL_OK)
        transport_fault=true;
      const std::array<uint8_t,8> zero{};
      if(CAN_TrySend(&hcan2,zero.data(),0x200,8)!=HAL_OK) transport_fault=true;
      stopped=true;
      if(!transport_fault) {
        const auto result=chassis::SaveYawCalibration(s.yaw_angle);
        dbg_yaw_calibration_result=uint32_t(result);
        if(result==chassis::CalibrationSave::Saved) {forward_offset=s.yaw_angle;dbg_yaw_calibrated=1;}
      } else dbg_yaw_calibration_result=3;
    }
  }
  chassis::BoardFeedback b;
  if(chassis::config::kYawRateControlEnabled) {
    ImuGetFeedback(now,b.chassis_imu);
    b.chassis_imu.ready=b.chassis_imu.ready && dbg_chassis_imu_mounting_confirmed!=0;
  }
  b.updated_ms=now;b.relative_yaw=std::remainder(s.yaw_angle-forward_offset,6.283185307f);
  b.readiness.yaw_ready=online && s.yaw_valid;
  b.readiness.upper_board_ready=online && s.drive && !upper_stop_latched;
  b.readiness.imu_ready=online && s.imu_ready;
  b.upper_mode_ready=c.mode==chassis::Mode::Depart ||
      (online && (!chassis::config::kImuLossTripEnabled || s.imu_mode));
  constexpr uint32_t fatal_can_errors=HAL_CAN_ERROR_BOF|HAL_CAN_ERROR_TIMEOUT|
      HAL_CAN_ERROR_NOT_INITIALIZED|HAL_CAN_ERROR_NOT_READY|HAL_CAN_ERROR_NOT_STARTED|
      HAL_CAN_ERROR_PARAM|HAL_CAN_ERROR_INTERNAL;
  // Warning/ACK/arbitration flags are sticky history, not proof of a current
  // bus outage. Freshness, bounded TX retry and the watchdog remain enforced.
  if((HAL_CAN_GetError(&hcan1)|HAL_CAN_GetError(&hcan2))&fatal_can_errors)
    transport_fault=true;
  // A missed foreground tick resets readiness and the motion planner immediately.
  if(!timely || calibration_hold) {
    b.readiness.upper_board_ready=false;b.readiness.yaw_ready=false;
  }
  auto result=drive.updateIntegrated(c,wheels,b,now,transport_fault);
  if(upper_retry.busy || uint32_t(now-last_send)>=10u || c.emergency_stop || !c.online) {
    last_send=now;
    (void)SendUpper(ToUpper(c,transport_fault));
  }
  if(transport_fault) {
    result=drive.updateIntegrated(c,wheels,b,now,true);Stop(c);
  } else {
    const auto data=chassis::PackC620(result.commands);
    const bool moving=result.state==chassis::State::Working;
    if(!moving && !stopped)
      HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    bool sent=false;
    if(RetryExpired(wheel_retry,now))transport_fault=true;
    else sent=AcceptTx(CAN_TrySend(&hcan2,data.data(),0x200,8),wheel_retry,now);
    if(transport_fault) {
      Stop(c);result=drive.updateIntegrated(c,wheels,b,now,true);
    } else if(sent) stopped=!moving;
  }
  chassis::four_wheel_status=result;
  dbg_state=uint32_t(result.state);dbg_ready_count=result.ready_count;dbg_mode=uint32_t(c.mode);
  dbg_rc_online=c.online;dbg_estop=c.emergency_stop;dbg_centered=c.centered;
  dbg_chassis_forward_input=c.forward;dbg_chassis_left_input=c.left;
  dbg_left_switch=remote.rc_l_switch();dbg_right_switch=remote.rc_r_switch();
  dbg_fault=transport_fault || result.state==chassis::State::Fault;
  dbg_rc_frames=frames;dbg_rc_invalid=invalid;dbg_motor_online_mask=result.online_mask;
  dbg_motor_online=result.online_mask==15;dbg_can_error=HAL_CAN_GetError(&hcan1)|HAL_CAN_GetError(&hcan2);
  dbg_upper_online=online;dbg_upper_ready=online&&s.drive;dbg_upper_reasons=s.reasons;
  dbg_imu_ready=b.readiness.imu_ready;dbg_yaw_aligned=1;
  dbg_chassis_imu_ready=b.chassis_imu.ready;
  dbg_chassis_target_yaw_rate=result.target_yaw_rate;
  dbg_chassis_actual_yaw_rate=b.chassis_imu.yaw_rate;
  dbg_chassis_applied_yaw_rate=result.velocity.yaw_rate;
  dbg_yaw_forward_offset_rad=forward_offset;dbg_relative_yaw_deg=b.relative_yaw*57.2957795f;
  dbg_lower_reasons=(!c.online?1u:0u)|(c.emergency_stop?2u:0u)|(!online?4u:0u)|
      (!b.readiness.upper_board_ready?16u:0u)|
      (c.mode!=chassis::Mode::Depart&&!b.readiness.imu_ready?32u:0u)|
      (!timely?64u:0u)|(transport_fault?128u:0u)|(result.online_mask!=15?256u:0u)|
      (result.state==chassis::State::Fault?512u:0u)|
      (chassis::config::kYawRateControlEnabled && !b.chassis_imu.ready?1024u:0u)|
      (online && !s.yaw_valid?2048u:0u)|
      (result.state==chassis::State::Preparing && !c.centered?4096u:0u)|
      (upper_stop_latched?8192u:0u)|(!b.upper_mode_ready?16384u:0u);
  if(was_working && result.state!=chassis::State::Working)dbg_last_stop_reasons=dbg_lower_reasons;
  FaultLogSnapshot log{};
  log.tick_ms=now;log.reasons=dbg_lower_reasons;log.mode=uint32_t(c.mode);
  log.state=uint32_t(result.state);log.gap_ms=log_gap;
  log.flags=(c.online?1u:0u)|(c.emergency_stop?2u:0u)|(online?4u:0u)|
      (s.drive?8u:0u)|(b.chassis_imu.ready?16u:0u)|(s.imu_ready?32u:0u)|
      (upper_stop_latched?64u:0u)|(transport_fault?128u:0u)|(!stopped?256u:0u);
  log.peer_reasons=s.reasons;log.can1_error=HAL_CAN_GetError(&hcan1);
  log.can2_error=HAL_CAN_GetError(&hcan2);log.online_mask=result.online_mask;log.rx_frames=frames;
  log.yaw_rate=b.chassis_imu.yaw_rate;log.yaw_target_rate=result.target_yaw_rate;
  log.yaw_position=b.relative_yaw;
  for(unsigned i=0;i<4;++i)log.wheel_rpm[i]=float(wheels[i].rpm);
  FaultLogObserve(&log,result.state==chassis::State::Working);
  for(unsigned i=0;i<4;++i) {
    dbg_motor_target_rpm[i]=result.targets.motor_rpm[i];dbg_motor_actual_rpm[i]=wheels[i].rpm;
    dbg_wheel_target_rpm[i]=result.targets.wheel_rpm[i];
    dbg_wheel_actual_rpm[i]=float(wheels[i].rpm)*chassis::config::kWheels.motor_direction[i]/
        chassis::config::kWheels.total_reduction;
    dbg_motor_current[i]=result.commands[i];
    dbg_motor_feedback_current_a[i]=wheels[i].current_raw*(20.f/16384.f);
    dbg_motor_mechanical_angle_deg[i]=wheels[i].encoder*(360.f/8192.f);
  }
  constexpr unsigned selected=chassis::wheel_config::kObservedMotorId-1;
  dbg_target_rpm=dbg_motor_target_rpm[selected];dbg_actual_rpm=wheels[selected].rpm;
  dbg_current=result.commands[selected];dbg_temperature=wheels[selected].temperature;
  dbg_encoder=wheels[selected].encoder;dbg_mechanical_angle_deg=dbg_motor_mechanical_angle_deg[selected];
  dbg_feedback_current=wheels[selected].current_raw;dbg_feedback_current_a=dbg_motor_feedback_current_a[selected];
  dbg_speed_kp=chassis::wheel_config::kWheelPids[selected].kp;
  Unlock(p);
}
