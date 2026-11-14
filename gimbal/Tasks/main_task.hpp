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

extern volatile uint32_t g_dbg_state, g_dbg_ready_count, g_dbg_reasons;
extern volatile uint32_t g_dbg_rc_online, g_dbg_estop, g_dbg_yaw_online, g_dbg_pitch_online;
extern volatile uint32_t g_dbg_yaw_state, g_dbg_pitch_state, g_dbg_zeroed;
extern volatile uint32_t g_dbg_rx_frames, g_dbg_watchdog_feeds, g_dbg_tx_failed, g_dbg_stop_complete;
extern volatile float g_dbg_yaw_deg, g_dbg_pitch_deg, g_dbg_yaw_torque, g_dbg_pitch_torque;
extern volatile float g_dbg_yaw_feedback_torque, g_dbg_pitch_feedback_torque;
extern volatile uint32_t g_dbg_selected_axis, g_dbg_configured, g_dbg_param_mask, g_dbg_control_mode;
extern volatile uint32_t g_dbg_feedback_id, g_dbg_capture_zero_request, g_dbg_zero_result;
extern volatile float g_dbg_pmax, g_dbg_vmax, g_dbg_tmax;
extern volatile float g_dbg_target_deg, g_dbg_actual_deg, g_dbg_actual_rate;
extern volatile float g_dbg_torque_limit;
// Selected-axis curves: angle PID outputs rad/s; speed PID outputs joint N*m.
extern volatile float g_dbg_angle_p, g_dbg_angle_i, g_dbg_angle_d, g_dbg_angle_pid_out;
extern volatile float g_dbg_speed_p, g_dbg_speed_i, g_dbg_speed_d, g_dbg_speed_pid_out;
extern volatile float g_dbg_target_rate;
extern volatile uint32_t g_dbg_command_mode, g_dbg_command_centered;
extern volatile uint32_t g_dbg_can1_error, g_dbg_can2_error;
// Stable compatibility names from the former communication bench.
extern volatile uint32_t dbg_link_online, dbg_link_rc_online, dbg_link_estop;
extern volatile uint32_t dbg_link_init_error, dbg_link_tx_count, dbg_link_rx_count;
extern volatile float dbg_link_yaw, dbg_link_pitch;

void MainInit(void);

void MainTask(void);
// Poll in foreground after MainInit; timer IRQ only posts a tick request.
void MainPoll(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __MAIN_TASK_HPP__ */
