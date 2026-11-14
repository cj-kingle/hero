#pragma once
#include "HW_can.hpp"
void LowerInit();
void LowerPoll();
void LowerSafetyTick();
bool LowerOnUart(UART_HandleTypeDef*,uint16_t);
void LowerOnCan(CAN_HandleTypeDef*,const CAN_RxHeaderTypeDef*,const uint8_t*);
extern "C" {
extern volatile uint32_t dbg_upper_online,dbg_upper_ready,dbg_upper_reasons,dbg_imu_ready;
extern volatile uint32_t dbg_yaw_aligned,dbg_capture_yaw_forward,dbg_lower_reasons;
extern volatile float dbg_relative_yaw_deg,dbg_yaw_forward_offset_rad;
extern volatile uint32_t dbg_yaw_calibrated,dbg_yaw_calibration_result;
extern volatile uint32_t dbg_chassis_imu_ready;
extern volatile float dbg_chassis_target_yaw_rate,dbg_chassis_actual_yaw_rate,dbg_chassis_applied_yaw_rate;
extern volatile float dbg_chassis_forward_input,dbg_chassis_left_input;
extern volatile float dbg_cfg_gyro_yaw_rate,dbg_cfg_follow_yaw_rate,dbg_cfg_gyro_yaw_acceleration;
extern volatile uint32_t dbg_last_stop_reasons;
extern volatile float dbg_wheel_target_rpm[4],dbg_wheel_actual_rpm[4];
}
