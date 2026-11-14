#include "imu_task.hpp"
#include "chassis_imu_config.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
uint32_t HAL_GetTick() {return 0;}
int main() {
  uint32_t now=0;
  const float acc[]{0,0,9.80665f},still[]{0,0,0};
  ImuResetProcessing();
  for(unsigned i=0;i<100;++i)ImuProcessSample(acc,still,++now);
  now+=11;assert(!ImuProcessSample(acc,still,now));
  assert(dbg_chassis_imu_samples==1); // late samples cannot complete initial calibration
  for(unsigned i=0;i<1999;++i)ImuProcessSample(acc,still,++now);
  assert(ImuReady(now) && dbg_chassis_imu_calibrated);
  const auto resets=dbg_chassis_imu_reset_count;
  const float spin[]{0,0,3.5f};float expected=0;
  // Exercise real bias/AHRS/continuous-yaw processing through multiple turns,
  // including the 11..19 ms gaps that previously discarded calibration.
  for(unsigned i=0;i<12000;++i) {
    const uint32_t gap=i%127==0 ? (i%3==0 ? 11u : 19u) : 1u;
    now+=gap;expected+=3.5f*float(gap)*.001f;
    assert(ImuProcessSample(acc,spin,now));
    chassis::ChassisImuFeedback f;assert(ImuGetFeedback(now,f));
    assert(f.ready && std::fabs(dbg_chassis_imu_raw_yaw_rate-3.5f)<.01f);
    assert(f.yaw_rate>=0 && f.yaw_rate<=3.51f);
    if(i>150)assert(std::fabs(f.yaw_rate-3.5f)<.01f); // retain the configured startup of the rate filter
    assert(std::fabs(f.yaw-expected)<.15f);
    assert(dbg_chassis_imu_reset_count==resets);
  }
  // Finite acceleration shocks at speed must not clear established gyro bias
  // or poison attitude/yaw-rate feedback. This failed before the repair.
  const float shock[]{50,0,9.80665f};
  for(unsigned i=0;i<100;++i) {
    ++now;assert(ImuProcessSample(i%10==0 ? shock : acc,spin,now));
    chassis::ChassisImuFeedback f;assert(ImuGetFeedback(now,f));
    assert(std::fabs(f.yaw_rate-3.5f)<.02f && dbg_chassis_imu_reset_count==resets);
  }
  assert(!ImuReady(now+20)); // genuine stale feedback remains unavailable
  chassis::ChassisImuFeedback before,after;assert(ImuGetFeedback(now,before));
  now+=80;assert(ImuProcessSample(acc,spin,now));
  assert(dbg_chassis_imu_calibrated && dbg_chassis_imu_reset_count==resets);
  assert(ImuGetFeedback(now,after) && std::fabs(after.yaw-before.yaw)<.01f);
  const float invalid[]{0,0,std::numeric_limits<float>::quiet_NaN()};
  assert(!ImuProcessSample(acc,invalid,++now) && !ImuReady(now));
  assert(dbg_chassis_imu_calibrated && dbg_chassis_imu_reset_count==resets);
  assert(ImuProcessSample(acc,spin,++now) && ImuGetFeedback(now,after));
  assert(std::fabs(after.yaw_rate-3.5f)<.02f && dbg_chassis_imu_samples==2000);
  ImuResetProcessing();assert(!ImuProcessSample(acc,spin,++now));
  assert(!dbg_chassis_imu_calibrated); // manual reset/power-up still needs a stationary first calibration
  std::cout<<"Real chassis IMU processing: calibration, multiple turns, bounded gaps and genuine loss passed\n";
}
