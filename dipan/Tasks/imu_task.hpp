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
#include <cstdint>
#include "chassis_imu_feedback.hpp"
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
bool ImuGetFeedback(uint32_t now_ms,chassis::ChassisImuFeedback& feedback);
extern "C" {
extern volatile uint32_t dbg_chassis_imu_init_error,dbg_chassis_imu_samples,dbg_chassis_imu_calibrated;
extern volatile uint32_t dbg_chassis_imu_mounting_confirmed;
extern volatile float dbg_chassis_imu_yaw_deg,dbg_chassis_imu_pitch_deg,dbg_chassis_imu_roll_deg;
extern volatile float dbg_chassis_imu_raw_yaw_rate;
extern volatile uint32_t dbg_chassis_imu_update_count,dbg_chassis_imu_gap_ms,dbg_chassis_imu_reset_count;
extern volatile uint32_t dbg_chassis_imu_stationary;
extern volatile float dbg_chassis_imu_acc_norm,dbg_chassis_imu_max_gyro;
}
#endif /* HOMEWORK_TASKS_IMU_TASK_HPP_ */
