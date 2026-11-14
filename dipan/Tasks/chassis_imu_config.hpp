#pragma once
#include <array>
namespace chassis::imu_config {
// Chassis axes: +X forward, +Y left, +Z up. Row-major sensor-to-chassis rotation.
inline constexpr std::array<float,9> kSensorToBody{1,0,0,0,1,0,0,0,1};
// Confirm by observing counterclockwise chassis rotation -> positive yaw rate.
inline constexpr bool kMountingConfirmed=true; // bench confirmed: counterclockwise chassis rotation produces positive yaw rate
inline constexpr float kYawSign=1;
inline constexpr float kYawRateFilterTau=.01f; // seconds; 0 disables filtering
inline constexpr unsigned kMaxSampleGapMs=10; // initial stationary calibration budget
inline constexpr unsigned kMaxCalibratedSampleGapMs=19; // bias remains valid within the existing 20 ms freshness window
constexpr float Abs(float x){return x<0?-x:x;}
constexpr bool Valid(){
  const auto& r=kSensorToBody;
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j){
    float v=0;for(unsigned k=0;k<3;++k)v+=r[3*i+k]*r[3*j+k];
    if(!(Abs(v-(i==j?1.f:0.f))<.001f))return false;
  }
  const float det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  return Abs(det-1)<.001f && (kYawSign==1 || kYawSign==-1) &&
      kYawRateFilterTau>=0 && kYawRateFilterTau<=1 && kMaxSampleGapMs>=1 &&
      kMaxSampleGapMs<=kMaxCalibratedSampleGapMs && kMaxCalibratedSampleGapMs<20;
}
static_assert(Valid(),"Invalid chassis IMU mounting rotation, yaw sign or filter");
}
