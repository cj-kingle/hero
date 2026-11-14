/**
*******************************************************************************
* @file      :main_task.cpp
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

// Integrated upper-board entry; single-axis bench remains selectable in its config.
#include "main_task.hpp"
#include "gimbal_runtime.hpp"
#include "imu_task.hpp"
#include "single_axis_config.hpp"
#include "system_user.hpp"

uint32_t tick = 0;
namespace { volatile bool tick_pending = false; }

void RobotInit(void) { GimbalSoftwareInit(); tick_pending = false; }

void MainInit(void) {
  RobotInit();
  const bool can_ok=GimbalStartCan();
  if (!can_ok) GimbalRequestStop();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) GimbalRequestStop();
  // Stop traffic is already active during sensor initialization.
  if constexpr (!gimbal::single_axis_config::kEnabled) { if(can_ok) ImuInit(); }
  MX_IWDG_Init();
  // No USART3 DT7 reception: the receiver is physically connected to LOWER board.
}

void MainTask(void) {
  if constexpr (!gimbal::single_axis_config::kEnabled) ImuUpdate();
  GimbalStep();
}
void MainPoll(void) {
  const uint32_t primask = __get_PRIMASK(); __disable_irq();
  const bool pending = tick_pending; tick_pending = false;
  __set_PRIMASK(primask);
  if (pending) MainTask();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) { ++tick; GimbalSafetyTick(); tick_pending = true; }
}
