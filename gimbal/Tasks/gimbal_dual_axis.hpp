#pragma once
#include "gimbal_task.hpp"
#include "dm_parameters.hpp"
#include "dm4340_motor.hpp"

namespace gimbal {
enum class AxisBus : uint8_t { Can1, Can2 };
struct AxisMounting {
  int direction=0; // Explicit hardware confirmation required: +1 or -1.
  float motor_per_joint=1, feedback_wrap=6.283185307f;
  bool absolute_position=false,continuous_absolute=false;
};
struct DualAxisConfig {
  Config control{}; // Separate from the active single-axis tuning configuration.
  AxisMounting yaw{},pitch{};
  float yaw_overspeed=.75f,pitch_overspeed=.4f;
  uint32_t yaw_overspeed_confirm_ms=0; // default bench mode retains immediate protection
  bool overspeed_trip_enabled=true;
  bool imu_mounting_confirmed=false;
};
struct ParameterQuery {
  AxisBus bus=AxisBus::Can1;
  uint16_t id=0x7ff;
  damiao::Payload data{};
};
// Dormant full-gimbal module. No global instance, HAL calls, CAN transmission,
// watchdog refresh or main-task hooks. Caller serializes access and supplies
// ImuGetFeedback(now) plus transport status to update() once per millisecond.
class DualAxisGimbal {
 public:
  explicit DualAxisGimbal(DualAxisConfig config={});
  bool nextParameterQuery(uint32_t now,ParameterQuery& query);
  bool onCan(AxisBus bus,uint16_t standard_id,const uint8_t* data,size_t length,uint32_t now);
  bool captureZeros(uint32_t now); // stopped, fresh disabled and stationary on BOTH axes
  void requestStop() { stop_pending_=true; }
  Output update(uint32_t now,const ImuFeedback& imu,bool transport_ok=true);
  bool parametersReady() const;
  uint32_t parameterMask(AxisBus bus) const;
  uint32_t parameterMode(AxisBus bus) const {
    return bus==AxisBus::Can1 ? yaw_parameters_.mode() : pitch_parameters_.mode();
  }
  const damiao::Motor& motor(AxisBus bus) const;
  ControlDiagnostics diagnostics() const { return controller_.diagnostics(); }
  void confirmImuMounting(bool confirmed) { config_.imu_mounting_confirmed=confirmed; }
  uint32_t faultReason() const { return latched_reason_; }
 private:
  bool configurationReady() const;
  void applyParameters(AxisBus bus);
  DualAxisConfig config_;
  Controller controller_;
  damiao::ParameterReader yaw_parameters_{3},pitch_parameters_{9};
  damiao::Motor yaw_{};
  damiao::DM4340 pitch_{damiao::MotorConfig{}};
  CommandReceiver commands_;
  Output last_{};
  uint32_t last_query_=0,query_slots_[2]{},latched_reason_=0;
  bool queried_=false,stop_pending_=false,next_pitch_=false,mode_seen_=false;
  bool yaw_enabled_seen_=false,pitch_enabled_seen_=false;
  uint32_t yaw_overspeed_since_=0;
  bool yaw_overspeed_seen_=false;
  Mode last_mode_=Mode::Joint;
};
}
