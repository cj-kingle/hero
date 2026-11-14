#include "main_task.hpp"
#include "system_user.hpp"
#include "chassis_config.hpp"
#include "lower_runtime.hpp"
#include "imu_task.hpp"

// Keep existing Ozone observations; LowerRuntime owns their values.
volatile uint32_t dbg_state=0,dbg_ready_count=0,dbg_rc_online=0,dbg_motor_online=0;
volatile uint32_t dbg_fault=0,dbg_centered=0,dbg_estop=1;
volatile uint32_t dbg_left_switch=2,dbg_right_switch=2,dbg_mode=0;
volatile uint32_t dbg_rc_frames=0,dbg_rc_invalid=0,dbg_can_rx=0,dbg_can_error=0;
volatile float dbg_target_rpm=0,dbg_actual_rpm=0,dbg_current=0,dbg_temperature=0;
volatile float dbg_speed_kp=0,dbg_feedback_current=0,dbg_feedback_current_a=0;
volatile uint32_t dbg_encoder=0,dbg_motor_online_mask=0;
volatile float dbg_mechanical_angle_deg=0;
volatile float dbg_motor_target_rpm[4]{},dbg_motor_actual_rpm[4]{};
volatile float dbg_motor_current[4]{},dbg_motor_feedback_current_a[4]{};
volatile float dbg_motor_mechanical_angle_deg[4]{};

namespace {
volatile bool control_pending=false;
}
void MainInit(void) {
  control_pending=false;
  LowerInit();
  if(HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK) Error_Handler();
  if constexpr(chassis::config::kYawRateControlEnabled) ImuInit();
}
void MainTask(void) {
  if constexpr(chassis::config::kYawRateControlEnabled) ImuUpdate();
  LowerPoll();
}
void MainPoll(void) {
  const uint32_t primask=__get_PRIMASK();__disable_irq();
  const bool pending=control_pending;control_pending=false;
  __set_PRIMASK(primask);
  // Coalesce timer notifications; the independent safety tick detects task loss.
  if(pending)MainTask();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if(htim==&htim6) {
    LowerSafetyTick();
    control_pending=true;
  }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,uint16_t size) {
  (void)LowerOnUart(huart,size);
}
