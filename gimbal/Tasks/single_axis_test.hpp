#pragma once
#include "gimbal_task.hpp"
namespace gimbal {
enum class Axis : uint8_t { Yaw, Pitch };
enum class SingleAxisMode : uint8_t { Position, Velocity };
struct SingleAxisConfig {
  Axis axis=Axis::Yaw;
  SingleAxisMode mode=SingleAxisMode::Position;
  float max_rate=.15f, torque_limit=.15f;
  float max_rate_acceleration=0; // rad/s^2, reference slew; 0 bypasses it
  float pitch_min=-.087266463f, pitch_max=.087266463f, limit_margin=.017453293f;
  float overspeed=.75f;
  float friction_torque=0; // joint N*m, magnitude of velocity-directed friction feedforward
  float friction_torque_reverse=-1; // negative-rate magnitude; -1 follows friction_torque
  float friction_transition_rate=.03f; // rad/s; blend to zero near zero target velocity
  float pitch_gravity_torque=0; // signed joint N*m; fixed gravity bias for Pitch only
  // Parameter storage only. Tune both loops in single_axis_config.hpp.
  // Zero initialization prevents an unconfigured controller from driving.
  PidConfig angle_pid{}, speed_pid{};
};
struct SingleAxisDiagnostics {
  PidTerms angle{},speed{};
  float target_rate=0,torque_command=0;
};
class SingleAxisTest {
 public:
  explicit SingleAxisTest(SingleAxisConfig config={}) : config_(config),
      angle_(config.angle_pid), speed_(config.speed_pid) {}
  Output update(const Command&,const damiao::JointFeedback&,bool configured,bool transport_ok,uint32_t now);
  void stop();
  SingleAxisDiagnostics diagnostics() const {
    return {angle_.terms(),speed_.terms(),rate_command_,torque_command_};
  }
 private:
  SingleAxisConfig config_;
  Pid angle_,speed_;
  State state_=State::Dead;
  uint32_t boot_=0,last_=0,ready_=0,stall_=0;
  float target_=0;
  float rate_command_=0,torque_command_=0;
  float ramped_rate_=0;
  bool started_=false;
  uint32_t latched_reason_=0;
};
}
