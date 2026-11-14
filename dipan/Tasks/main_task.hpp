/**
*******************************************************************************
* @file      ：main_task.hpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#include <stdint.h>
// Flat read-only debugger observations: simple C symbols for J-Scope/Watch.
extern volatile uint32_t dbg_state, dbg_ready_count, dbg_rc_online, dbg_motor_online;
extern volatile uint32_t dbg_fault, dbg_centered, dbg_estop;
extern volatile uint32_t dbg_left_switch, dbg_right_switch;
extern volatile uint32_t dbg_mode;
extern volatile uint32_t dbg_rc_frames, dbg_rc_invalid, dbg_can_rx, dbg_can_error;
extern volatile float dbg_target_rpm, dbg_actual_rpm, dbg_current, dbg_temperature;
extern volatile float dbg_speed_kp, dbg_feedback_current;
extern volatile float dbg_feedback_current_a;
// Selected motor rotor position: raw 0..8191 and single-turn mechanical degrees.
// Check dbg_motor_online before interpreting these last-received values.
extern volatile uint32_t dbg_encoder;
extern volatile float dbg_mechanical_angle_deg;
// Array index 0..3 is motor CAN ID 1..4; rpm/current retain raw C620 signs.
extern volatile uint32_t dbg_motor_online_mask;
extern volatile float dbg_motor_target_rpm[4], dbg_motor_actual_rpm[4];
extern volatile float dbg_motor_current[4], dbg_motor_feedback_current_a[4];
extern volatile float dbg_motor_mechanical_angle_deg[4];

void MainInit(void);

void MainTask(void);
void MainPoll(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#ifdef __cplusplus
#include "remote_command.hpp"
#include "chassis_task.hpp"
#include "chassis_imu_feedback.hpp"
#include <cstdint>
namespace chassis {
enum class State : uint8_t { Dead, Preparing, Working, Fault };
struct Readiness {
  // Transport adapters must derive these flags from fresh, validated feedback.
  bool wheels_ready = false;
  bool yaw_ready = false;
  bool upper_board_ready = false;
  bool imu_ready = false;
  bool fault = false;
};
struct BoardFeedback {
  Readiness readiness;
  float relative_yaw = 0; // rad, positive counterclockwise from chassis to gimbal
  uint32_t updated_ms = 0;
  ChassisImuFeedback chassis_imu{};
  bool upper_mode_ready=true; // short command/status handshake; not another preparation cycle
};
class RobotStateMachine {
 public:
  void update(const RemoteCommand& command, const Readiness& ready,
              uint32_t now_ms,uint32_t max_control_gap_ms=1,bool actuation_ready=true,
              bool preserve_preparation=false);
  State state() const { return state_; }
  uint32_t readyCount() const { return ready_count_; }
  Mode mode() const { return mode_; }
 private:
  State state_ = State::Dead;
  Mode mode_ = Mode::Depart;
  uint32_t ready_count_ = 0, previous_ms_ = 0;
  bool clock_started_ = false, fault_latched_ = false, reset_armed_ = false;
  bool prepared_ = false; // completed initial arming; runtime pauses still inhibit all output
};
}
#endif
#endif /* __MAIN_TASK_HPP__ */
