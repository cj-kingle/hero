#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Shared binary format, version 1. Board: 1 lower, 2 upper.
// Event: 1 stopped after Working, 2 reset while Working, 3 CPU exception.
typedef struct {
  // flags bit 8: outputs still active (Flash flush must wait).
  uint32_t board, event, tick_ms, reasons, mode, state, flags, gap_ms;
  uint32_t peer_reasons, can1_error, can2_error, online_mask, rx_frames;
  uint32_t reset_flags, cfsr, hfsr, bfar, mmfar;
  float yaw_rate, pitch_rate, yaw_target_rate, pitch_target_rate;
  float yaw_torque, pitch_torque, yaw_position, pitch_position;
  float wheel_rpm[4];
} FaultLogSnapshot;
// Debugger-readable latest saved event. valid=1 only after full CRC validation.
extern volatile FaultLogSnapshot dbg_fault_log;
extern volatile uint32_t dbg_fault_log_valid, dbg_fault_log_sequence;
// storage: 0 idle, 1 saved, 2 full, 3 write failed. Never erases automatically.
extern volatile uint32_t dbg_fault_log_storage, dbg_fault_log_reset_flags;
// Simple scalar names for Ozone expressions; build marker proves matching ELF.
extern volatile uint32_t dbg_diag_version,dbg_stop_reason,dbg_stop_event,dbg_stop_tick_ms;
void FaultLogObserve(const FaultLogSnapshot* snapshot, int working);
void FaultLogCpuFault(uint32_t cfsr,uint32_t hfsr,uint32_t bfar,uint32_t mmfar);
void FaultLogBoot(uint32_t board);
void FaultLogPoll(void);
#ifdef __cplusplus
}
#endif
