/**
*******************************************************************************
* @file      :imu_task.hpp
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
#ifndef HOMEWORK_TASKS_IMU_TASK_HPP_
#define HOMEWORK_TASKS_IMU_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gimbal_task.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern float euler_angles[3];
/* Exported function prototypes ----------------------------------------------*/

void ImuInit();
void ImuUpdate();
// Pure processing entry for fresh samples; may be used before hardware integration.
// accel: m/s^2, gyro: rad/s. Call once per NEW 1 ms sample, outside IRQ handlers.
bool ImuProcessSample(const float accel[3], const float gyro[3], uint32_t sample_ms);
void ImuResetProcessing();
bool ImuReady(uint32_t now_ms);
bool ImuGetFeedback(uint32_t now_ms, gimbal::ImuFeedback& result);
#endif /* HOMEWORK_TASKS_IMU_TASK_HPP_ */
