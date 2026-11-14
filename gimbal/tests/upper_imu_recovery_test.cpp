#include "imu_task.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
extern volatile uint32_t g_dbg_imu_samples,g_dbg_imu_calibrated,g_dbg_imu_reset_count;
uint32_t HAL_GetTick(){return 0;}
int main() {
  const float acc[]{0,0,9.80665f},bias[]{.01f,-.02f,.03f},spin[]{.01f,-.02f,3.53f};
  uint32_t now=0;ImuResetProcessing();
  for(int i=0;i<100;++i)assert(!ImuProcessSample(acc,spin,++now));
  assert(!g_dbg_imu_calibrated && g_dbg_imu_samples==0); // moving at initial boot cannot establish a zero-rate bias
  for(int i=0;i<100;++i)ImuProcessSample(acc,bias,++now);
  now+=11;assert(!ImuProcessSample(acc,bias,now));assert(g_dbg_imu_samples==1);
  for(int i=0;i<1999;++i)ImuProcessSample(acc,bias,++now);
  assert(ImuReady(now) && g_dbg_imu_calibrated);
  const auto resets=g_dbg_imu_reset_count;
  const float shock[]{50,0,9.80665f};float expected=0;
  for(int i=0;i<12000;++i) {
    const unsigned gap=i%127==0 ? (i%2 ? 11u:19u) : 1u;
    now+=gap;expected+=3.5f*float(gap)*.001f;
    assert(ImuProcessSample(i%50==0?shock:acc,spin,now));
    gimbal::ImuFeedback f;assert(ImuGetFeedback(now,f));
    assert(std::fabs(f.yaw_rate-3.5f)<.01f && std::fabs(f.yaw-expected)<.15f);
    assert(g_dbg_imu_calibrated && g_dbg_imu_reset_count==resets);
  }
  gimbal::ImuFeedback before,after;assert(ImuGetFeedback(now,before));
  assert(!ImuReady(now+20));now+=80;
  assert(ImuProcessSample(acc,spin,now) && ImuGetFeedback(now,after));
  assert(std::fabs(after.yaw-before.yaw)<.01f); // no invented integration over the missing 80 ms
  const float invalid[]{0,0,std::numeric_limits<float>::quiet_NaN()};
  assert(!ImuProcessSample(acc,invalid,++now) && !ImuReady(now));
  assert(g_dbg_imu_calibrated && g_dbg_imu_reset_count==resets);
  assert(ImuProcessSample(acc,spin,++now) && ImuGetFeedback(now,after));
  assert(std::fabs(after.yaw_rate-3.5f)<.01f && g_dbg_imu_samples==2000);
  ImuResetProcessing();assert(!ImuProcessSample(acc,spin,++now)); // explicit reset still needs initial stationary calibration
  std::cout<<"Upper IMU preserves known bias through moving recovery, rejects invalid samples and retains cold-start calibration\n";
}
