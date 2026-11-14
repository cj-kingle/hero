#include "main_task.hpp"
#include <algorithm>
namespace chassis {
void RobotStateMachine::update(const RemoteCommand& command, const Readiness& ready,
                               uint32_t now_ms,uint32_t max_control_gap_ms,bool actuation_ready,
                               bool preserve_preparation) {
  const auto elapsed=uint32_t(now_ms-previous_ms_);
  const bool one_tick = clock_started_ && elapsed>=1u && elapsed<=max_control_gap_ms && max_control_gap_ms<20u;
  previous_ms_ = now_ms; clock_started_ = true;
  if (ready.fault) fault_latched_ = true;
  if (fault_latched_) {
    state_ = State::Fault; ready_count_ = 0; prepared_ = false;
    // A deliberate emergency-stop -> release sequence acknowledges a cleared fault.
    // Acknowledgements made while the fault persists are not remembered.
    if (ready.fault) { reset_armed_ = false; return; }
    if (command.online && command.emergency_stop) reset_armed_ = true;
    if (command.online && !command.emergency_stop && reset_armed_) {
      fault_latched_ = reset_armed_ = false; state_ = State::Dead;
    }
    return;
  }
  if (!command.online || command.emergency_stop) {
    state_ = State::Dead; ready_count_ = 0; prepared_ = false; return;
  }
  const bool ready_all = ready.wheels_ready
                         // && ready.yaw_ready
                         // && ready.upper_board_ready
                         // && ready.imu_ready
                         ;
  // Evaluate from the old state: never jump Dead -> Working in one update.
  switch (state_) {
    case State::Dead:
      state_ = State::Preparing; ready_count_ = 0; break;
    case State::Preparing:
      if(preserve_preparation && prepared_) {
        // A runtime feedback pause is not another initial arming sequence.
        // Stay inhibited until fresh feedback, timing and actuation are valid;
        // the motor/planner layer resets its ramps while paused.
        ready_count_=2000u;
        if(ready_all && actuation_ready && one_tick) {mode_=command.mode;state_=State::Working;}
        break;
      }
      if (!ready_all || !command.centered || !one_tick) ready_count_ = 0;
      else {
        // Count real milliseconds, including bounded 1..5 ms scheduling jitter.
        // Local preparation may run while the upper board is preparing. Motor
        // effort remains forbidden until the upper confirms it is enabled.
        ready_count_=std::min<uint32_t>(2000u,ready_count_+elapsed);
        if(ready_count_>=2000u && actuation_ready) {
          mode_=command.mode;state_=State::Working;prepared_=preserve_preparation;
        }
      }
      break;
    case State::Working:
      if (!ready_all || !actuation_ready || !one_tick) {
        state_ = State::Preparing;
        ready_count_ = preserve_preparation && prepared_ ? 2000u : 0u;
      }
      else mode_ = command.mode;
      break;
    default:
      fault_latched_ = true; state_ = State::Fault; ready_count_ = 0; prepared_ = false; break;
  }
}
}
