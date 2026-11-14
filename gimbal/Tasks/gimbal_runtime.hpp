#pragma once
#include "HW_can.hpp"
#include "gimbal_task.hpp"

namespace gimbal {
struct RuntimeStatus {
  Output output{};
  Input input{};
  damiao::Feedback yaw_raw{}, pitch_raw{};
  uint32_t command_frames = 0, rejected_frames = 0, watchdog_feeds = 0;
  uint32_t tx_ok = 0, tx_failed = 0, stop_requests = 0;
  bool stop_frames_completed = false;
};
RuntimeStatus GetStatus(); // Interrupt-safe snapshot for debugger/unit integration.
}

// Runtime called from the foreground task; single_axis_config.hpp selects bench/full control.
void GimbalSoftwareInit();
bool GimbalStartCan();
void GimbalStep(); // Call once per 1 ms in FOREGROUND, never inside a CAN/timer IRQ.
bool GimbalCaptureMechanicalZero(); // Explicit manual midpoint, disabled motors only.
void GimbalRequestStop();
void GimbalSafetyTick(); // bounded, nonblocking; TIM6 IRQ also checks link/task timeout
void GimbalOnCanRx(CAN_HandleTypeDef*, const CAN_RxHeaderTypeDef&, const uint8_t*);
void GimbalOnCanError(CAN_HandleTypeDef*,bool receive_failed=false);
extern "C" {
extern volatile uint32_t g_dbg_fault_latched,g_dbg_last_fault_reason;
extern volatile uint32_t g_dbg_active_mode,g_dbg_imu_switch_pending;
}
