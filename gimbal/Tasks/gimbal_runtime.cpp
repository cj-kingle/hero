#include "gimbal_runtime.hpp"
#include "main_task.hpp"
#include "gimbal_config.hpp"
#include "dm4310_drv.hpp"
#include "dm4340_motor.hpp"
#include "dm_stop.hpp"
#include "imu_task.hpp"
#include "iwdg.h"
#include "single_axis_config.hpp"
#include "dm_parameters.hpp"
#include <cmath>

// Plain C-linkage mirrors avoid Ozone's C++ namespace/expression limitations.
volatile uint32_t g_dbg_state=0, g_dbg_ready_count=0, g_dbg_reasons=gimbal::NoCommand;
volatile uint32_t g_dbg_rc_online=0, g_dbg_estop=1, g_dbg_yaw_online=0, g_dbg_pitch_online=0;
volatile uint32_t g_dbg_yaw_state=0, g_dbg_pitch_state=0, g_dbg_zeroed=0;
volatile uint32_t g_dbg_rx_frames=0, g_dbg_watchdog_feeds=0, g_dbg_tx_failed=0, g_dbg_stop_complete=0;
volatile float g_dbg_yaw_deg=0, g_dbg_pitch_deg=0, g_dbg_yaw_torque=0, g_dbg_pitch_torque=0;
volatile float g_dbg_yaw_feedback_torque=0, g_dbg_pitch_feedback_torque=0;
volatile uint32_t g_dbg_selected_axis=0, g_dbg_configured=0, g_dbg_param_mask=0, g_dbg_control_mode=0;
volatile uint32_t g_dbg_feedback_id=0xffff, g_dbg_capture_zero_request=0, g_dbg_zero_result=0;
volatile float g_dbg_pmax=0, g_dbg_vmax=0, g_dbg_tmax=0;
volatile float g_dbg_target_deg=0, g_dbg_actual_deg=0, g_dbg_actual_rate=0;
volatile float g_dbg_torque_limit=0;
volatile float g_dbg_angle_p=0, g_dbg_angle_i=0, g_dbg_angle_d=0, g_dbg_angle_pid_out=0;
volatile float g_dbg_speed_p=0, g_dbg_speed_i=0, g_dbg_speed_d=0, g_dbg_speed_pid_out=0;
volatile float g_dbg_target_rate=0;
volatile float g_dbg_speed_ff=0;
volatile float g_dbg_filtered_rate=0; // PID measurement only; raw remains the overspeed source
// First overspeed sample plus PREVIOUS control step values. Preserve after stop
// so Ozone can inspect the event instead of only seeing reset/zero PID terms.
volatile float g_dbg_trip_raw_rate=0, g_dbg_trip_filtered_rate=0;
volatile float g_dbg_trip_target_rate=0, g_dbg_trip_torque=0;
volatile float g_dbg_position_rate=0; // rad/s from position increments; observer, not PID input.
volatile uint32_t g_dbg_command_mode=0, g_dbg_command_centered=0;
volatile uint32_t g_dbg_active_mode=0,g_dbg_imu_switch_pending=0;
volatile uint32_t g_dbg_can1_error=0, g_dbg_can2_error=0;
volatile uint32_t g_dbg_can_tx_busy=0;
volatile uint32_t dbg_link_online=0, dbg_link_rc_online=0, dbg_link_estop=1;
volatile uint32_t dbg_link_init_error=0, dbg_link_tx_count=0, dbg_link_rx_count=0;
volatile float dbg_link_yaw=0, dbg_link_pitch=0;
volatile uint32_t g_dbg_dual_axis=0,g_dbg_yaw_param_mask=0,g_dbg_pitch_param_mask=0;
volatile uint32_t g_dbg_imu_mounting_confirmed=gimbal::config::kImuMountingConfirmed;
volatile uint32_t g_dbg_imu_ready=0;
volatile uint32_t g_dbg_fault_latched=0,g_dbg_last_fault_reason=0;
volatile float g_dbg_yaw_target_deg=0,g_dbg_pitch_target_deg=0;
volatile float g_dbg_yaw_target_rate=0,g_dbg_pitch_target_rate=0;
volatile float g_dbg_yaw_position_kp=0,g_dbg_pitch_position_kp=0; // effective scheduled position-loop gains
volatile float g_dbg_yaw_load_torque=0;
// Last manual turn, excluding its first 1200 ms. Release freezes these until
// the next turn; only explicit software initialization clears the last run.
volatile uint32_t g_dbg_yaw_run_samples=0,g_dbg_yaw_run_mode_changes=0,g_dbg_yaw_run_i_limit_samples=0;
volatile float g_dbg_yaw_run_input_span=0,g_dbg_yaw_run_target_span=0,g_dbg_yaw_run_rate_span=0,g_dbg_yaw_run_i_peak=0;
volatile float g_dbg_yaw_run_slow_motor_deg=0,g_dbg_yaw_run_slow_torque=0;
volatile float g_dbg_yaw_run_slow_rate=0,g_dbg_yaw_run_slow_position_rate=0;

namespace {
struct YawRunStats {
  bool active=false;uint32_t started=0;float sign=1;
  gimbal::Mode mode=gimbal::Mode::Joint;
  float input_min=0,input_max=0,target_min=0,target_max=0,rate_min=0,rate_max=0;
} yaw_run;
void ClearYawRunStats() {
  g_dbg_yaw_run_samples=0;g_dbg_yaw_run_mode_changes=0;g_dbg_yaw_run_i_limit_samples=0;
  g_dbg_yaw_run_input_span=0;g_dbg_yaw_run_target_span=0;
  g_dbg_yaw_run_rate_span=0;g_dbg_yaw_run_i_peak=0;
  g_dbg_yaw_run_slow_motor_deg=0;g_dbg_yaw_run_slow_torque=0;
  g_dbg_yaw_run_slow_rate=0;g_dbg_yaw_run_slow_position_rate=0;
}
void ObserveYawRun(const gimbal::Input& in,const gimbal::Output& out,
    const gimbal::ControlDiagnostics& d,float position_rate,uint32_t now) {
  if(!out.drive || !in.command.online || in.command.emergency_stop || in.command.yaw==0) {
    yaw_run.active=false;return;
  }
  if(!yaw_run.active || yaw_run.sign*in.command.yaw<0) {
    yaw_run={};yaw_run.active=true;yaw_run.started=now;
    yaw_run.sign=std::copysign(1.f,in.command.yaw);yaw_run.mode=out.active_mode;
    ClearYawRunStats();
  }
  if(out.active_mode!=yaw_run.mode) {
    g_dbg_yaw_run_mode_changes=g_dbg_yaw_run_mode_changes+1;yaw_run.mode=out.active_mode;
  }
  if(uint32_t(now-yaw_run.started)<1200u)return;
  const float input=std::fabs(in.command.yaw),target=yaw_run.sign*d.yaw_rate;
  const float rate=yaw_run.sign*d.yaw_speed.measurement;
  if(!std::isfinite(target) || !std::isfinite(rate))return;
  const bool first=g_dbg_yaw_run_samples==0;
  if(first) {
    yaw_run.input_min=yaw_run.input_max=input;yaw_run.target_min=yaw_run.target_max=target;
    yaw_run.rate_min=yaw_run.rate_max=rate;
  }
  if(first || rate<yaw_run.rate_min) {
    g_dbg_yaw_run_slow_motor_deg=std::remainder(in.yaw.position,6.283185307f)*57.2957795f;
    g_dbg_yaw_run_slow_torque=out.yaw_torque;
    g_dbg_yaw_run_slow_rate=rate;
    g_dbg_yaw_run_slow_position_rate=yaw_run.sign*position_rate;
  }
  yaw_run.input_min=std::min(yaw_run.input_min,input);yaw_run.input_max=std::max(yaw_run.input_max,input);
  yaw_run.target_min=std::min(yaw_run.target_min,target);yaw_run.target_max=std::max(yaw_run.target_max,target);
  yaw_run.rate_min=std::min(yaw_run.rate_min,rate);yaw_run.rate_max=std::max(yaw_run.rate_max,rate);
  g_dbg_yaw_run_input_span=yaw_run.input_max-yaw_run.input_min;
  g_dbg_yaw_run_target_span=yaw_run.target_max-yaw_run.target_min;
  g_dbg_yaw_run_rate_span=yaw_run.rate_max-yaw_run.rate_min;
  g_dbg_yaw_run_i_peak=std::max(float(g_dbg_yaw_run_i_peak),std::fabs(d.yaw_speed.i));
  if(gimbal::config::kControl.yaw_manual_integral_limit>0 &&
      std::fabs(d.yaw_speed.i)>=gimbal::config::kControl.yaw_manual_integral_limit-.0001f)
    g_dbg_yaw_run_i_limit_samples=g_dbg_yaw_run_i_limit_samples+1;
  g_dbg_yaw_run_samples=g_dbg_yaw_run_samples+1;
}
gimbal::DualAxisGimbal dual_controller(gimbal::config::kDual);
gimbal::SingleAxisTest single_controller(gimbal::single_axis_config::kControl);
damiao::ParameterReader yaw_parameters(3), pitch_parameters(9);
uint32_t last_query_ms=0, query_slot=0;
uint32_t last_status_ms=0;
uint8_t status_sequence=0;
constexpr bool single_axis=gimbal::single_axis_config::kEnabled;
constexpr bool selected_yaw=gimbal::single_axis_config::kSelectedAxis==gimbal::Axis::Yaw;
damiao::DM4310 yaw(gimbal::config::kYaw);
damiao::DM4340 pitch(gimbal::config::kPitch);
const damiao::Motor& YawMotor() { return single_axis ? static_cast<const damiao::Motor&>(yaw) : dual_controller.motor(gimbal::AxisBus::Can1); }
const damiao::Motor& PitchMotor() { return single_axis ? static_cast<const damiao::Motor&>(pitch) : dual_controller.motor(gimbal::AxisBus::Can2); }
gimbal::CommandReceiver command_receiver;
gimbal::WatchdogGate watchdog;
gimbal::RuntimeStatus status;
volatile bool initialized = false;
volatile uint32_t stop_epoch = 0;
uint32_t handled_stop_epoch = 0, last_step_ms = 0;
bool can_started = false, have_step = false, previous_drive = false;
uint32_t last_link_ms=0;
bool seen_link=false;
volatile bool tx_fault = false;
bool safety_latched = true;
constexpr uint32_t kAllMailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
constexpr uint32_t kFatalCanErrors=HAL_CAN_ERROR_BOF|HAL_CAN_ERROR_TIMEOUT|
    HAL_CAN_ERROR_NOT_INITIALIZED|HAL_CAN_ERROR_NOT_READY|HAL_CAN_ERROR_NOT_STARTED|
    HAL_CAN_ERROR_PARAM|HAL_CAN_ERROR_INTERNAL;
struct TxState {
  damiao::StopSequence stop;
  uint32_t last_enable = 0;
  bool enable_sent = false;
  bool effort_blocked = false;
  uint32_t effort_blocked_since = 0;
};
TxState yaw_tx, pitch_tx;
uint32_t Lock() { const uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
void Unlock(uint32_t p) { __set_PRIMASK(p); }
// Called with IRQs masked. Repeated stop requests MUST NOT abort the zero/FD
// packets already in flight. Only the transition out of drive starts a new stop.
void LatchStop(bool notify_controller=true) {
  if (!safety_latched) {
    safety_latched = true;
    yaw_tx = {}; pitch_tx = {};
  }
  if(notify_controller) stop_epoch = stop_epoch + 1;
}
bool RelevantBus(CAN_HandleTypeDef* bus) { return !single_axis || !selected_yaw || bus==&hcan1; }
bool Send(CAN_HandleTypeDef* bus, const damiao::Payload& data, uint16_t id) {
  const auto result=CAN_TrySend(bus,data.data(),id,8);
  if(result==HAL_OK) { ++status.tx_ok; return true; }
  // Busy is temporary backpressure, not a hardware CAN error. StopSequence
  // retries its own packets; enabled motor effort has a separate 5 ms bound.
  if(result==HAL_BUSY) { g_dbg_can_tx_busy=g_dbg_can_tx_busy+1; return false; }
  ++status.tx_failed; if (RelevantBus(bus)) tx_fault = true; return false;
}
bool Pending(CAN_HandleTypeDef* bus) { return HAL_CAN_IsTxMessagePending(bus, kAllMailboxes) != 0; }
class StopBus final : public damiao::StopTransport {
 public:
  explicit StopBus(CAN_HandleTypeDef* bus) : bus_(bus) {}
  bool abortPending() override {
    if (HAL_CAN_AbortTxRequest(bus_, kAllMailboxes) == HAL_OK) return true;
    ++status.tx_failed; if (RelevantBus(bus_)) tx_fault = true; return false;
  }
  bool pending() override { return Pending(bus_); }
  bool send(const damiao::Payload& data, uint16_t id) override { return Send(bus_, data, id); }
 private:
  CAN_HandleTypeDef* bus_;
};
bool StopAxis(CAN_HandleTypeDef* bus, const damiao::Motor& motor, TxState& tx, uint32_t now) {
  StopBus transport(bus);
  tx.enable_sent = false;
  return tx.stop.step(transport, motor.config().control_id, motor.config().limits, now);
}
bool DriveAxis(CAN_HandleTypeDef* bus, const damiao::Motor& motor,
               const damiao::JointFeedback& fb, TxState& tx, float torque, uint32_t now) {
  // The stop transaction must finish on the bus BEFORE any subsequent enable.
  if (!tx.stop.holding()) { StopAxis(bus, motor, tx, now); return false; }
  if (!motor.configured() || !fb.healthy || !fb.zeroed) return false;
  if (!fb.enabled) {
    // The first MIT packet after enable is zero; torque is allowed only after
    // enabled feedback is actually received, never by guessing a fixed delay.
    if (!tx.enable_sent || uint32_t(now-tx.last_enable) >= 20u) {
      if (Pending(bus)) return false;
      damiao::Payload zero;
      if (!motor.torqueFrame(0, zero) || !Send(bus, zero, motor.config().control_id) ||
          !Send(bus, damiao::EnableFrame(), motor.config().control_id)) return false;
      tx.enable_sent = true; tx.last_enable = now;
    }
    return false;
  }
  damiao::Payload frame;
  if(!motor.torqueFrame(torque,frame)) { tx_fault=true;return false; }
  if(Send(bus,frame,motor.config().control_id)) { tx.effort_blocked=false;return true; }
  if(!tx.effort_blocked) { tx.effort_blocked=true;tx.effort_blocked_since=now; }
  else if(uint32_t(now-tx.effort_blocked_since)>=gimbal::config::kControl.max_control_gap_ms)
    tx_fault=true;
  return false;
}
bool MotorRangesSupportOutput() {
  const auto& c = gimbal::config::kControl;
  return YawMotor().configured() && PitchMotor().configured() &&
      c.yaw_torque_limit <= YawMotor().config().limits.torque*YawMotor().config().motor_per_joint &&
      c.pitch_torque_limit <= PitchMotor().config().limits.torque*PitchMotor().config().motor_per_joint;
}
}

void GimbalSoftwareInit() {
  const auto lock = Lock();
  initialized = false;
  dual_controller = gimbal::DualAxisGimbal(gimbal::config::kDual);
  single_controller = gimbal::SingleAxisTest(gimbal::single_axis_config::kControl);
  yaw_parameters=damiao::ParameterReader(3); pitch_parameters=damiao::ParameterReader(9);
  last_query_ms=query_slot=0;
  last_status_ms=0;status_sequence=0;
  last_link_ms=0; seen_link=false; dbg_link_init_error=0; dbg_link_rx_count=0;
  yaw = damiao::DM4310(gimbal::config::kYaw); pitch = damiao::DM4340(gimbal::config::kPitch);
  command_receiver = {}; watchdog = {}; status = {};
  yaw_run={};ClearYawRunStats();
  g_dbg_trip_raw_rate=0; g_dbg_trip_filtered_rate=0;
  g_dbg_trip_target_rate=0; g_dbg_trip_torque=0;
  stop_epoch = handled_stop_epoch = 0;
  g_dbg_fault_latched=0;g_dbg_last_fault_reason=0;g_dbg_can_tx_busy=0;
  yaw_tx = {}; pitch_tx = {}; have_step = previous_drive = false; tx_fault = false;
  safety_latched = true;
  can_started = false; initialized = true;
  Unlock(lock);
}
bool GimbalStartCan() {
  if (!initialized || can_started) return can_started;
  CanFilter_Init(&hcan1); CanFilter_Init(&hcan2);
  const bool start1=HAL_CAN_Start(&hcan1)==HAL_OK;
  const bool start2=HAL_CAN_Start(&hcan2)==HAL_OK;
  if(!start1 || !start2) { dbg_link_init_error=(!start1 ? 1u : 0u)|(!start2 ? 2u : 0u); return false; }
  const uint32_t errors = CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING | CAN_IT_ERROR;
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | errors) != HAL_OK ||
      HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | errors) != HAL_OK) {
    dbg_link_init_error=4; return false;
  }
  can_started = true; GimbalRequestStop(); return true;
}
void GimbalRequestStop() {
  const auto lock = Lock(); LatchStop(); Unlock(lock);
}
void GimbalSafetyTick() {
  const auto lock = Lock();
  if (!initialized || !can_started) { Unlock(lock); return; }
  const auto now = HAL_GetTick();
  const auto command = command_receiver.snapshot(now);
  if (!command.online || command.emergency_stop ||
      (have_step && uint32_t(now-last_step_ms)>=20u)) LatchStop();
  if (safety_latched) {
    const bool y = StopAxis(&hcan1,YawMotor(),yaw_tx,now);
    const bool p = StopAxis(&hcan2,PitchMotor(),pitch_tx,now);
    status.stop_frames_completed = single_axis ? (selected_yaw ? y : p) : y && p;
    g_dbg_stop_complete = status.stop_frames_completed;
    g_dbg_estop = 1;
    g_dbg_yaw_torque = 0; g_dbg_pitch_torque = 0;
  }
  Unlock(lock);
}
void GimbalOnCanRx(CAN_HandleTypeDef* bus, const CAN_RxHeaderTypeDef& header, const uint8_t* data) {
  if (!initialized || header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA) return;
  const uint32_t now = HAL_GetTick();
  if(!single_axis) dual_controller.onCan(bus==&hcan1 ? gimbal::AxisBus::Can1 : gimbal::AxisBus::Can2,
      uint16_t(header.StdId),data,header.DLC,now);
  if (bus == &hcan1 && header.StdId == gimbal::kBoardCommandId) {
    const bool accepted = command_receiver.receive(data, header.DLC, now);
    const auto command = command_receiver.snapshot(now);
    if (accepted) ++status.command_frames; else ++status.rejected_frames;
    // Even a repeated valid stop frame has priority over the previous command.
    gimbal::Command decoded;
    const bool valid = gimbal::DecodeCommand(data, header.DLC, decoded);
    if(valid) { last_link_ms=now; seen_link=true; dbg_link_rx_count=dbg_link_rx_count+1; }
    if (!valid || !command.online || (valid && decoded.emergency_stop)) {
      if (valid && decoded.emergency_stop && !accepted) command_receiver.invalidate();
      GimbalRequestStop();
      GimbalSafetyTick(); // urgent zero/disable does not wait for foreground
    }
    if (watchdog.mayFeed(accepted, command.online, now)) {
      // Sole refresh point: NEW lower-board RC command, NOT motor feedback.
      if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) ++status.watchdog_feeds;
    }
  } else if (single_axis && header.DLC==8 && data[2]==0x33 && data[1]==0 &&
      ((bus==&hcan1 && data[0]==3) || (bus==&hcan2 && data[0]==9))) {
    if (bus==&hcan1) yaw_parameters.receive(uint16_t(header.StdId),data,header.DLC);
    else pitch_parameters.receive(uint16_t(header.StdId),data,header.DLC);
  } else if (single_axis && bus == &hcan1 && header.StdId == YawMotor().config().feedback_id) {
    if (!yaw.onFeedback(uint16_t(header.StdId), data, header.DLC, now)) GimbalRequestStop();
  } else if (single_axis && bus == &hcan2 && header.StdId == PitchMotor().config().feedback_id) {
    if (!pitch.onFeedback(uint16_t(header.StdId), data, header.DLC, now)) GimbalRequestStop();
  }
}
void GimbalOnCanError(CAN_HandleTypeDef* bus,bool receive_failed) {
  if (initialized && RelevantBus(bus) &&
      (receive_failed || (HAL_CAN_GetError(bus)&kFatalCanErrors))) {
    tx_fault = true; GimbalRequestStop();
  }
}
bool GimbalCaptureMechanicalZero() {
  if (!initialized) return false;
  const auto lock = Lock();
  const uint32_t now = HAL_GetTick();
  if(!single_axis) {
    const bool ok=dual_controller.captureZeros(now);
    Unlock(lock); return ok;
  }
  const auto command = command_receiver.snapshot(now);
  const bool stopped = !command.online || command.emergency_stop;
  bool ok = false;
  if (stopped) {
    // Software zero only: NEVER send the persistent motor zero command 0xFE.
    ok = selected_yaw ? yaw.captureZero(now) : pitch.captureZero(now);
  }
  Unlock(lock); return ok;
}
void GimbalStep() {
  if (!initialized) return;
  const uint32_t now = HAL_GetTick();
  if (have_step && now == last_step_ms) return;
  have_step = true; last_step_ms = now;
  if (g_dbg_capture_zero_request==1) {
    g_dbg_capture_zero_request=0;
    g_dbg_zero_result=GimbalCaptureMechanicalZero() ? 1u : 2u;
  }
  gimbal::Input input;
  damiao::Payload parameter_query{};
  CAN_HandleTypeDef* query_bus=selected_yaw ? &hcan1 : &hcan2;
  bool query_pending=false;
  auto lock = Lock();
  if (single_axis) {
    auto& parameters=selected_yaw ? yaw_parameters : pitch_parameters;
    auto& motor=selected_yaw ? static_cast<damiao::Motor&>(yaw) : static_cast<damiao::Motor&>(pitch);
    auto c=parameters.motorConfig(gimbal::single_axis_config::kDirection,
        gimbal::single_axis_config::kFeedbackWrap);
    c.absolute_position=!selected_yaw; // pitch uses the motor's absolute horizontal zero
    if (!motor.configured() && parameters.ready() &&
        (!selected_yaw || c.feedback_id!=gimbal::kBoardCommandId)) {
      if (selected_yaw) yaw=damiao::DM4310(c); else pitch=damiao::DM4340(c);
    }
    if (!parameters.complete() && uint32_t(now-last_query_ms)>=20u) {
      last_query_ms=now;
      parameter_query=parameters.request(query_slot++);
      query_pending=true;
    }
    g_dbg_selected_axis=selected_yaw ? 0u : 1u;
    g_dbg_param_mask=parameters.mask(); g_dbg_control_mode=parameters.mode();
    g_dbg_feedback_id=c.feedback_id;
    g_dbg_pmax=c.limits.position; g_dbg_vmax=c.limits.velocity; g_dbg_tmax=c.limits.torque;
    g_dbg_configured=motor.configured();
  } else {
    dual_controller.confirmImuMounting(g_dbg_imu_mounting_confirmed!=0);
    gimbal::ParameterQuery query;
    if(dual_controller.nextParameterQuery(now,query)) {
      query_pending=true; parameter_query=query.data;
      query_bus=query.bus==gimbal::AxisBus::Can1 ? &hcan1 : &hcan2;
    }
    const auto focus=selected_yaw ? gimbal::AxisBus::Can1 : gimbal::AxisBus::Can2;
    const auto& c=dual_controller.motor(focus).config();
    g_dbg_param_mask=dual_controller.parameterMask(focus);
    g_dbg_control_mode=dual_controller.parameterMode(focus);
    g_dbg_feedback_id=c.feedback_id;
    g_dbg_pmax=c.limits.position; g_dbg_vmax=c.limits.velocity; g_dbg_tmax=c.limits.torque;
    g_dbg_configured=dual_controller.parametersReady();
  }
  g_dbg_selected_axis=selected_yaw ? 0u : 1u; // legacy curve focus, not active-axis count
  g_dbg_dual_axis=single_axis ? 0u : 1u;
  g_dbg_yaw_param_mask=single_axis ? yaw_parameters.mask() : dual_controller.parameterMask(gimbal::AxisBus::Can1);
  g_dbg_pitch_param_mask=single_axis ? pitch_parameters.mask() : dual_controller.parameterMask(gimbal::AxisBus::Can2);
  const uint32_t epoch = stop_epoch;
  input.command = command_receiver.snapshot(now);
  input.yaw = YawMotor().joint(now); input.pitch = PitchMotor().joint(now);
  input.configuration_ready = single_axis ?
      ((selected_yaw ? YawMotor().configured() : PitchMotor().configured()) &&
       (selected_yaw ? YawMotor().config().limits.torque : PitchMotor().config().limits.torque)>=
          gimbal::single_axis_config::kControl.torque_limit) : MotorRangesSupportOutput();
  status.yaw_raw = YawMotor().raw(); status.pitch_raw = PitchMotor().raw();
  Unlock(lock);
  if (!single_axis) ImuGetFeedback(now, input.imu);
  input.imu.ready = input.imu.ready && g_dbg_imu_mounting_confirmed!=0;
  g_dbg_imu_ready=input.imu.ready;
  input.transport_ok = can_started && !tx_fault &&
      !(HAL_CAN_GetError(&hcan1)&kFatalCanErrors) &&
      ((single_axis && selected_yaw) || !(HAL_CAN_GetError(&hcan2)&kFatalCanErrors));
  if (epoch != handled_stop_epoch) {
    dual_controller.requestStop(); single_controller.stop();
    handled_stop_epoch = epoch; ++status.stop_requests;
    // Force at least this pass to stop even if a newer release frame arrived.
    input.command.emergency_stop = true;
  }
  const auto previous_pid=single_controller.diagnostics();
  const auto previous_dual=dual_controller.diagnostics();
  lock=Lock();
  auto output = single_axis ? single_controller.update(input.command,
      selected_yaw ? input.yaw : input.pitch,input.configuration_ready,input.transport_ok,now) :
      dual_controller.update(now,input.imu,input.transport_ok);
  Unlock(lock);
  if((output.reasons & gimbal::Overspeed) &&
      !(status.output.reasons & gimbal::Overspeed)) {
    const bool trip_yaw=single_axis ? selected_yaw : std::fabs(input.yaw.velocity)>gimbal::config::kDual.yaw_overspeed;
    g_dbg_trip_raw_rate=trip_yaw ? input.yaw.velocity : input.pitch.velocity;
    g_dbg_trip_filtered_rate=single_axis ? previous_pid.speed.measurement :
        (trip_yaw ? previous_dual.yaw_speed.measurement : previous_dual.pitch_speed.measurement);
    g_dbg_trip_target_rate=single_axis ? previous_pid.target_rate :
        (trip_yaw ? previous_dual.yaw_rate : previous_dual.pitch_rate);
    g_dbg_trip_torque=trip_yaw ? status.output.yaw_torque : status.output.pitch_torque;
  }
  tx_fault = false;
  if (can_started) {
    lock = Lock();
    // Prevent an estop IRQ between the final permission check and CAN enqueue.
    const bool permitted = output.drive && stop_epoch == epoch;
    if (!permitted) {
      // A controller fault already stopped its own state. Do not inject a
      // synthetic estop into it: that would clear a latched overspeed/stall.
      if (previous_drive) LatchStop(false);
      const bool y = StopAxis(&hcan1, YawMotor(), yaw_tx, now);
      const bool p = StopAxis(&hcan2, PitchMotor(), pitch_tx, now);
      status.stop_frames_completed = single_axis ? (selected_yaw ? y : p) : y && p;
      output.drive = false; output.yaw_torque = output.pitch_torque = 0;
    } else {
      safety_latched = false;
      status.stop_frames_completed = false;
      const bool y = single_axis && !selected_yaw ?
          (StopAxis(&hcan1,YawMotor(),yaw_tx,now),false) :
          DriveAxis(&hcan1, YawMotor(), input.yaw, yaw_tx, output.yaw_torque, now);
      const bool p = single_axis && selected_yaw ?
          (StopAxis(&hcan2,PitchMotor(),pitch_tx,now),false) :
          DriveAxis(&hcan2, PitchMotor(), input.pitch, pitch_tx, output.pitch_torque, now);
      if (!y) output.yaw_torque = 0;
      if (!p) output.pitch_torque = 0;
    }
    previous_drive = permitted;
    if (tx_fault) {
      // Immediately abort any queued drive and start stopping BOTH axes.
      dual_controller.requestStop(); single_controller.stop(); LatchStop(false);
      StopAxis(&hcan1, YawMotor(), yaw_tx, now); StopAxis(&hcan2, PitchMotor(), pitch_tx, now);
      output = {}; output.reasons = gimbal::Transport; previous_drive = false;
    }
    Unlock(lock);
  }
  // StopSequence may abort mailboxes on a fresh stop request. Queue the read
  // AFTER that transaction so estop-held parameter discovery is not cancelled.
  if (query_pending && can_started) {
    lock=Lock();
    if (CAN_TrySend(query_bus,parameter_query.data(),0x7ff,8)!=HAL_OK)
      ++status.tx_failed;
    Unlock(lock);
  }
  lock = Lock(); status.input = input; status.output = output;
  g_dbg_state = uint32_t(output.state); g_dbg_ready_count = output.ready_count;
  g_dbg_reasons = output.reasons; g_dbg_rc_online = input.command.online;
  g_dbg_fault_latched=single_axis ? 0u : dual_controller.faultReason();
  if(g_dbg_fault_latched)g_dbg_last_fault_reason=g_dbg_fault_latched;
  g_dbg_estop = input.command.emergency_stop;
  g_dbg_command_mode=uint32_t(input.command.mode);
  g_dbg_active_mode=uint32_t(output.active_mode);
  g_dbg_imu_switch_pending=(output.reasons&gimbal::ImuModePending)!=0;
  g_dbg_command_centered=input.command.centered;
  g_dbg_can1_error=HAL_CAN_GetError(&hcan1); g_dbg_can2_error=HAL_CAN_GetError(&hcan2);
  dbg_link_online=seen_link && uint32_t(now-last_link_ms)<100u;
  dbg_link_rc_online=input.command.online; dbg_link_estop=input.command.emergency_stop;
  dbg_link_yaw=input.command.yaw; dbg_link_pitch=input.command.pitch;
  dbg_link_tx_count=status.tx_ok; // total queued upper motor packets, no echo test remains
  g_dbg_yaw_online = input.yaw.online; g_dbg_pitch_online = input.pitch.online;
  g_dbg_yaw_state = status.yaw_raw.state; g_dbg_pitch_state = status.pitch_raw.state;
  g_dbg_zeroed = single_axis ? (selected_yaw ? input.yaw.zeroed : input.pitch.zeroed) :
      input.yaw.zeroed && input.pitch.zeroed;
  g_dbg_rx_frames = status.command_frames; g_dbg_watchdog_feeds = status.watchdog_feeds;
  g_dbg_tx_failed = status.tx_failed; g_dbg_stop_complete = status.stop_frames_completed;
  g_dbg_yaw_deg = input.yaw.position * 57.2957795f;
  g_dbg_pitch_deg = input.pitch.position * 57.2957795f;
  g_dbg_yaw_torque = output.yaw_torque; g_dbg_pitch_torque = output.pitch_torque;
  g_dbg_yaw_feedback_torque = input.yaw.torque; g_dbg_pitch_feedback_torque = input.pitch.torque;
  {
    const auto& fb=selected_yaw ? input.yaw : input.pitch;
    g_dbg_torque_limit=gimbal::single_axis_config::kControl.torque_limit;
    auto pid=single_controller.diagnostics();
    if(!single_axis) {
      const auto d=dual_controller.diagnostics();
      ObserveYawRun(input,output,d,YawMotor().positionRate(now),now);
      g_dbg_yaw_position_kp=d.yaw_angle.effective_kp;
      g_dbg_pitch_position_kp=d.pitch_angle.effective_kp;
      g_dbg_yaw_load_torque=d.yaw_load_torque;
      pid=selected_yaw ? gimbal::SingleAxisDiagnostics{d.yaw_angle,d.yaw_speed,d.yaw_rate,output.yaw_torque} :
          gimbal::SingleAxisDiagnostics{d.pitch_angle,d.pitch_speed,d.pitch_rate,output.pitch_torque};
      g_dbg_yaw_target_rate=d.yaw_rate; g_dbg_pitch_target_rate=d.pitch_rate;
    }
    g_dbg_angle_p=pid.angle.p; g_dbg_angle_i=pid.angle.i; g_dbg_angle_d=pid.angle.d;
    if(single_axis) {
      g_dbg_yaw_position_kp=selected_yaw ? pid.angle.effective_kp : 0;
      g_dbg_pitch_position_kp=selected_yaw ? 0 : pid.angle.effective_kp;
    }
    g_dbg_angle_pid_out=pid.angle.output;
    g_dbg_speed_p=pid.speed.p; g_dbg_speed_i=pid.speed.i; g_dbg_speed_d=pid.speed.d;
    g_dbg_speed_pid_out=pid.speed.output; g_dbg_target_rate=pid.target_rate;
    g_dbg_speed_ff=pid.speed.feedforward;
    g_dbg_filtered_rate=pid.speed.measurement;
    g_dbg_actual_deg=fb.position*57.2957795f; g_dbg_actual_rate=fb.velocity;
    g_dbg_position_rate=selected_yaw ? YawMotor().positionRate(now) : PitchMotor().positionRate(now);
    g_dbg_target_deg=(selected_yaw ? output.yaw_target : output.pitch_target)*57.2957795f;
    g_dbg_yaw_target_deg=output.yaw_target*57.2957795f;
    g_dbg_pitch_target_deg=output.pitch_target*57.2957795f;
  }
  if(!single_axis && can_started && uint32_t(now-last_status_ms)>=10u) {
    last_status_ms=now;
    gimbal::BoardStatus feedback;
    feedback.yaw_valid=input.yaw.online && input.yaw.healthy && input.yaw.zeroed;
    feedback.drive=output.drive && input.yaw.enabled && input.pitch.enabled &&
        input.yaw.healthy && input.pitch.healthy && !input.command.emergency_stop;
    feedback.imu_ready=input.imu.ready;
    feedback.imu_mode=output.drive && output.active_mode==gimbal::Mode::Imu;
    feedback.yaw_angle=input.yaw.position;feedback.reasons=uint16_t(output.reasons);
    feedback.sequence=++status_sequence;
    std::array<uint8_t,8> data;
    // Telemetry has lower priority than motor/stop packets. Congestion is handled
    // by LOWER's 100-ms freshness check, never by blocking motor control here.
    if(gimbal::EncodeStatus(feedback,data))
      (void)CAN_TrySend(&hcan1,data.data(),gimbal::kBoardStatusId,8);
  }
  watchdog.completedControl(now); Unlock(lock);
}
namespace gimbal {
RuntimeStatus GetStatus() { const auto lock = Lock(); const auto result = status; Unlock(lock); return result; }
}
