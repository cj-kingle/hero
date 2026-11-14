#include "gimbal_runtime.hpp"
#include "iwdg.h"
#include "gimbal_config.hpp"
#include <array>
#include <vector>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
CAN_HandleTypeDef hcan1{0},hcan2{1};
IWDG_HandleTypeDef hiwdg;
extern volatile uint32_t g_dbg_zeroed,g_dbg_dual_axis,g_dbg_yaw_param_mask,g_dbg_pitch_param_mask;
extern volatile uint32_t g_dbg_imu_mounting_confirmed;
extern volatile float g_dbg_yaw_target_deg,g_dbg_pitch_target_deg,g_dbg_trip_raw_rate,g_dbg_trip_torque;
extern volatile float g_dbg_yaw_torque,g_dbg_pitch_torque;
extern volatile uint32_t g_dbg_yaw_run_samples,g_dbg_yaw_run_mode_changes,g_dbg_yaw_run_i_limit_samples;
extern volatile float g_dbg_yaw_run_input_span,g_dbg_yaw_run_target_span,g_dbg_yaw_run_rate_span,g_dbg_yaw_run_i_peak;
namespace {
uint32_t now=0,feeds=0,aborts=0;
uint8_t sequence=0;
bool imu_available=false;
uint32_t yaw_busy_until=0;
bool yaw_send_error=false;
std::array<uint32_t,2> can_errors{};
gimbal::ImuFeedback imu{};
struct MotorModel {
  CAN_HandleTypeDef* bus;
  uint16_t id,feedback;
  damiao::Limits limits;
  int direction;
  bool enabled=false;
  float position=0,velocity=0;
};
std::array<MotorModel,2> motors{{
  {&hcan1,3,0x13,{12.5f,30,10},gimbal::single_axis_config::kYawDirection},
  {&hcan2,9,0x19,{12.5f,10,28},gimbal::single_axis_config::kPitchDirection}
}};
struct Packet { CAN_HandleTypeDef* bus; uint32_t id,due; damiao::Payload data; };
std::vector<Packet> queued,delivered;
void Rx(CAN_HandleTypeDef* bus,uint16_t id,const damiao::Payload& p) {
  GimbalOnCanRx(bus,CAN_RxHeaderTypeDef{id,0,0,8},p.data());
}
void Feedback(MotorModel& m) {
  auto map=[](float x,float bound,unsigned bits) {
    return unsigned(std::lround((x+bound)/(2*bound)*float((1u<<bits)-1)));
  };
  const auto p=map(std::remainder(m.position*m.direction,6.283185307f),m.limits.position,16),
      v=map(m.velocity*m.direction,m.limits.velocity,12),t=map(0,m.limits.torque,12);
  Rx(m.bus,m.feedback,{uint8_t((m.enabled ? 0x10 : 0)|m.id),uint8_t(p>>8),uint8_t(p),
      uint8_t(v>>4),uint8_t((v<<4)|(t>>8)),uint8_t(t),25,25});
}
void Flush() {
  std::vector<Packet> completed;
  for(auto it=queued.begin();it!=queued.end();) {
    if(it->due<=now) { completed.push_back(*it); it=queued.erase(it); }
    else ++it;
  }
  for(const auto& p:completed) {
    delivered.push_back(p);
    for(auto& m:motors) {
      if(p.bus!=m.bus) continue;
      if(p.id==0x7ff && p.data[0]==m.id) {
        auto reply=p.data; assert(reply[2]==0x33);
        uint32_t bits=0;
        switch(reply[3]) {
          case 7: bits=m.feedback; break;
          case 8: bits=m.id; break;
          case 10: bits=1; break;
          case 21: std::memcpy(&bits,&m.limits.position,4); break;
          case 22: std::memcpy(&bits,&m.limits.velocity,4); break;
          case 23: std::memcpy(&bits,&m.limits.torque,4); break;
          default: assert(false);
        }
        for(unsigned i=0;i<4;++i) reply[4+i]=uint8_t(bits>>(8*i));
        Rx(m.bus,m.feedback,reply);
      } else if(p.id==m.id) {
        if(p.data==damiao::EnableFrame()) m.enabled=true;
        if(p.data==damiao::DisableFrame()) m.enabled=false;
        Feedback(m);
      }
    }
  }
}
void Command(bool stop,float axis=0,bool repeat=false,gimbal::Mode mode=gimbal::Mode::Joint) {
  gimbal::Command c;
  c.online=true; c.emergency_stop=stop; c.centered=axis==0;
  c.yaw=axis; c.pitch=axis; c.mode=mode;
  c.sequence=repeat ? sequence : ++sequence;
  damiao::Payload p; assert(gimbal::EncodeCommand(c,p)); Rx(&hcan1,gimbal::kBoardCommandId,p);
}
void Advance(unsigned n,bool foreground=true,bool commands=true,bool stop=false,float axis=0) {
  for(unsigned i=0;i<n;++i) {
    ++now; Flush();
    if(commands && now%10==0) Command(stop,axis);
    GimbalSafetyTick(); if(foreground) GimbalStep();
  }
}
void Ready() {
  now=feeds=aborts=0; sequence=0; yaw_busy_until=0; yaw_send_error=false;can_errors={}; queued.clear(); delivered.clear();
  imu_available=false; imu={}; g_dbg_imu_mounting_confirmed=0;
  for(auto& m:motors) { m.enabled=false; m.velocity=0; }
  motors[0].position=.6f;
  motors[1].position=gimbal::config::kControl.pitch_start_angle; // fixture stores joint angle; Feedback() applies motor direction
  GimbalSoftwareInit(); assert(GimbalStartCan());
  Advance(350,true,true,true);
  assert(g_dbg_dual_axis==1 && g_dbg_yaw_param_mask==63 && g_dbg_pitch_param_mask==63);
  assert(g_dbg_zeroed==1); // Both motors use absolute feedback, no zero request.
  for(const auto& m:motors) assert(!m.enabled);
  Advance(4300);
  if(!gimbal::GetStatus().output.drive)std::cerr<<"Ready failed: reasons="<<gimbal::GetStatus().output.reasons<<" pitch="<<motors[1].position<<" now="<<now<<"\n";
  assert(gimbal::GetStatus().output.drive);
  assert(motors[0].enabled && motors[1].enabled);
  assert(std::fabs(g_dbg_yaw_target_deg-.6f*57.2957795f)<.02f); // hold yaw in place
  assert(std::fabs(g_dbg_pitch_target_deg-17.5f)<.001f); // raw motor -18 deg maps to joint +18 deg
}
bool IsNonzeroTorque(const Packet& p,const MotorModel& m) {
  if(p.bus!=m.bus || p.id!=m.id || p.data==damiao::EnableFrame() || p.data==damiao::DisableFrame()) return false;
  damiao::Payload zero; assert(damiao::PackMit(m.limits,0,0,0,0,0,zero)); return p.data!=zero;
}
void Drive() {
  Advance(200,true,true,false,.7f);
  for(const auto& m:motors) {
    bool found=false;
    for(auto it=delivered.rbegin();it!=delivered.rend() && it->due>now-15;++it) found|=IsNonzeroTorque(*it,m);
    assert(found && m.enabled);
  }
}
void VerifyStopped(size_t from) {
  for(const auto& m:motors) {
    bool zero=false,disable=false;
    damiao::Payload z; assert(damiao::PackMit(m.limits,0,0,0,0,0,z));
    for(size_t i=from;i<delivered.size();++i) {
      const auto& p=delivered[i]; if(p.bus!=m.bus || p.id!=m.id) continue;
      assert(!IsNonzeroTorque(p,m) && p.data!=damiao::EnableFrame());
      zero|=p.data==z;
      if(p.data==damiao::DisableFrame()) { assert(zero); disable=true; }
    }
    assert(zero && disable && !m.enabled);
  }
}
}
uint32_t HAL_GetTick() { return now; }
void CanFilter_Init(CAN_HandleTypeDef*) {}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef* bus,uint32_t) {
  ++aborts;
  for(auto it=queued.begin();it!=queued.end();) {
    if(it->bus==bus) it=queued.erase(it); else ++it;
  }
  return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef* bus,uint32_t) {
  for(const auto& p:queued) if(p.bus==bus) return 1;
  return 0;
}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef* bus) { return can_errors[bus->index]; }
HAL_StatusTypeDef CAN_TrySend(CAN_HandleTypeDef* bus,const uint8_t* p,uint32_t id,uint8_t n) {
  if(bus==&hcan1 && id==3 && yaw_send_error)return HAL_ERROR;
  if(bus==&hcan1 && id==3 && now<yaw_busy_until)return HAL_BUSY;
  assert(n==8); unsigned count=0; for(const auto& v:queued) count+=v.bus==bus;
  if(count==3) return HAL_BUSY;
  Packet item{bus,id,now+2,{}}; std::copy(p,p+8,item.data.begin()); queued.push_back(item); return HAL_OK;
}
void CAN_Send_Msg(CAN_HandleTypeDef*,uint8_t*,uint32_t,uint8_t) {
  assert(false && "Runtime must retain checked CAN transmission");
}
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef*) { ++feeds; return HAL_OK; }
bool ImuGetFeedback(uint32_t,gimbal::ImuFeedback& result) { result=imu; result.ready=imu_available; return result.ready; }
int main() {
  static_assert(!gimbal::single_axis_config::kEnabled,"This fixture tests integrated dual-axis runtime");
  // A held command has a fixed reference despite periodically varying measured
  // speed; the completed record survives release for inspection while stopped.
  Ready();
  for(unsigned n=0;n<2400;++n) {
    motors[0].velocity=n%200<100 ? .8f : 1.2f;
    Advance(1,true,true,false,.5f);
  }
  assert(g_dbg_yaw_run_samples>1000 && g_dbg_yaw_run_mode_changes==0);
  assert(g_dbg_yaw_run_input_span<.001f && g_dbg_yaw_run_target_span<.001f);
  assert(g_dbg_yaw_run_rate_span>.35f && g_dbg_yaw_run_i_peak<=gimbal::config::kControl.yaw_manual_integral_limit+.0001f);
  const auto samples=g_dbg_yaw_run_samples;const float span=g_dbg_yaw_run_rate_span;
  Command(false,0);Advance(200,true,true,false,0);
  assert(g_dbg_yaw_run_samples==samples && g_dbg_yaw_run_rate_span==span);
  // A short mailbox backlog while moving yaw must not disable either axis.
  Ready(); Drive();
  yaw_busy_until=now+4; Advance(3,true,true,false,.7f);
  assert(gimbal::GetStatus().output.drive && motors[0].enabled && motors[1].enabled);
  Advance(3,true,true,false,.7f);
  assert(gimbal::GetStatus().output.drive && motors[0].enabled && motors[1].enabled);
  // A sustained failure or a real HAL error still stops both motors.
  yaw_busy_until=now+50; Advance(gimbal::config::kControl.max_control_gap_ms+2,true,true,false,.7f);
  assert(!gimbal::GetStatus().output.drive && (gimbal::GetStatus().output.reasons&gimbal::Transport));
  yaw_busy_until=0; Advance(10,true,true,false,.7f);
  assert(!gimbal::GetStatus().output.drive);
  Ready(); Drive(); yaw_send_error=true; Advance(1,true,true,false,.7f);
  assert(!gimbal::GetStatus().output.drive && (gimbal::GetStatus().output.reasons&gimbal::Transport));
  for(unsigned axis=0;axis<2;++axis) {
    auto* bus=axis==0 ? &hcan1 : &hcan2;
    Ready();Drive();
    can_errors[axis]=HAL_CAN_ERROR_EWG|HAL_CAN_ERROR_EPV|HAL_CAN_ERROR_ACK|HAL_CAN_ERROR_TX_ALST0;
    GimbalOnCanError(bus);Advance(30);
    assert(gimbal::GetStatus().output.drive && g_dbg_fault_latched==0);
    can_errors[axis]=HAL_CAN_ERROR_BOF;GimbalOnCanError(bus);Advance(1);
    assert(!gimbal::GetStatus().output.drive && (g_dbg_fault_latched&gimbal::Transport));
    Ready();Drive();GimbalOnCanError(bus,true);Advance(1);
    assert(!gimbal::GetStatus().output.drive && (g_dbg_fault_latched&gimbal::Transport));
  }

  Ready(); Drive();
  bool found_status=false;
  for(const auto& frame:delivered) if(frame.id==gimbal::kBoardStatusId) {
    gimbal::BoardStatus s;assert(gimbal::DecodeStatus(frame.data.data(),8,s));
    if(s.drive) {
      assert(s.yaw_valid && std::fabs(s.yaw_angle-.6f)<.002f);
      found_status=true;
    }
  }
  assert(found_status);
  auto begin=delivered.size(); const auto abort_count=aborts;
  Command(true);
  for(unsigned i=0;i<30;++i) { Command(true,0,true); Advance(1,false,false); }
  VerifyStopped(begin); assert(aborts==abort_count+2);
  const auto before=feeds; Advance(40,false,true,true); assert(feeds==before);

  Ready(); Drive(); Advance(100,true,false); begin=delivered.size();
  Advance(20,true,false); VerifyStopped(begin);
  const auto lost_feeds=feeds; Advance(100,true,false); assert(feeds==lost_feeds);

  Ready(); Drive(); Advance(19,false,false);
  begin=delivered.size(); Advance(25,false,false); VerifyStopped(begin);

  for(unsigned axis=0;axis<2;++axis) {
    Ready(); Drive();
    const float threshold=axis==0 ? gimbal::config::kDual.yaw_overspeed : gimbal::config::kDual.pitch_overspeed;
    motors[axis].velocity=threshold+.2f; Feedback(motors[axis]);
    if(axis==0) {
      Advance(gimbal::config::kDual.yaw_overspeed_confirm_ms,true,false);
      assert(gimbal::GetStatus().output.drive);
    }
    // Integrated software overspeed trips are disabled; feedback remains visible.
    Advance(1,true,false);
    begin=delivered.size(); Advance(7,true,false);
    assert(gimbal::GetStatus().output.drive && !(gimbal::GetStatus().output.reasons&gimbal::Overspeed));
    motors[axis].velocity=0; Feedback(motors[axis]); Advance(50,true,false);
    assert(gimbal::GetStatus().output.drive);
  }

  // Initial IMU request while mounting/calibration is unavailable must hold
  // enabled Joint control and tell the lower board that IMU mode is NOT active.
  Ready();imu_available=true;
  Command(false,0,false,gimbal::Mode::Imu);
  const auto pending_begin=delivered.size();
  for(unsigned n=0;n<150;++n) {
    if(now%10==0)Command(false,0,false,gimbal::Mode::Imu);
    Advance(1,true,false);
    const auto held=gimbal::GetStatus().output;
    assert(held.drive && held.state==gimbal::State::Working);
    assert(held.active_mode==gimbal::Mode::Joint && (held.reasons&gimbal::ImuModePending));
    assert(g_dbg_fault_latched==0 && motors[0].enabled && motors[1].enabled);
  }
  bool pending_status=false;
  for(size_t i=pending_begin;i<delivered.size();++i) {
    const auto& frame=delivered[i];if(frame.id!=gimbal::kBoardStatusId)continue;
    gimbal::BoardStatus status;assert(gimbal::DecodeStatus(frame.data.data(),8,status));
    if(status.reasons&gimbal::ImuModePending) {
      assert(status.drive && !status.imu_mode && !status.imu_ready);pending_status=true;
    }
  }
  assert(pending_status);
  g_dbg_imu_mounting_confirmed=1;imu.yaw=.6f;
  Advance(1,true,false);
  assert(gimbal::GetStatus().output.drive && gimbal::GetStatus().output.active_mode==gimbal::Mode::Imu);
  assert(!(gimbal::GetStatus().output.reasons&gimbal::ImuModePending));
  assert(gimbal::GetStatus().output.ready_count==2000 && g_dbg_fault_latched==0);
  // Runtime IMU loss falls back to the encoder loop without dropping either motor.
  imu_available=false;Advance(1,true,false);
  assert(gimbal::GetStatus().output.drive && g_dbg_fault_latched==0);
  assert(gimbal::GetStatus().output.active_mode==gimbal::Mode::Joint);

  Ready(); imu_available=true; g_dbg_imu_mounting_confirmed=1;
  imu.yaw=1.2f; imu.pitch=.1f;
  Command(false,0,false,gimbal::Mode::Imu); Advance(1,true,false);
  auto out=gimbal::GetStatus().output;
  assert(out.drive && out.yaw_torque==0 && out.pitch_torque==0); // mode handover frame
  imu_available=false; Advance(1,true,false);
  assert(gimbal::GetStatus().output.drive && !(gimbal::GetStatus().output.reasons&gimbal::ImuUnavailable));
  Advance(30,true,false);assert(motors[0].enabled && motors[1].enabled);
  imu_available=true;Advance(1,true,false);
  assert(gimbal::GetStatus().output.active_mode==gimbal::Mode::Imu && gimbal::GetStatus().output.drive);
  std::cout<<"Integrated dual-axis discovery, absolute startup, estop, link/task loss, software-trip bypass and IMU fallback passed\n";
}
