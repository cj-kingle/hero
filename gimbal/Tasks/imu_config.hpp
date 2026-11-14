#pragma once
#include <array>
namespace gimbal::imu_config {
// Body axes: +X forward, +Y left, +Z up. Row-major sensor-to-body rotation.
// Change with motor power off; verify all three axes physically before confirming.
inline constexpr std::array<float,9> kSensorToBody{1,0,0,0,1,0,0,0,1};
inline constexpr bool kMountingConfirmed=true; // yaw direction checked: left turn increases IMU yaw
inline constexpr bool kPitchImuEnabled=false; // sensor does not track pitch linkage motion
inline constexpr unsigned kMaxSampleGapMs=10; // tolerate short scheduling jitter; longer gaps require recalibration
inline constexpr float kYawSign=1;
// Local linkage calibration: internal motor-angle change / IMU pitch change.
// Its sign includes the linkage direction; identity is an UNCALIBRATED placeholder.
inline constexpr float kPitchMotorPerImu=1;
inline constexpr float kPitchHorizontalImuRad=-0.1f*0.01745329252f; // approximate horizontal reading [deg] supplied during bench check
constexpr float Abs(float x){return x<0?-x:x;}
constexpr bool Valid() {
  const auto& r=kSensorToBody;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
    float v=0;for(unsigned k=0;k<3;++k)v+=r[3*i+k]*r[3*j+k];
    if(!(Abs(v-(i==j?1.f:0.f))<.001f))return false;
  }
  const float det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  return Abs(det-1)<.001f && (kYawSign==1 || kYawSign==-1) &&
      Abs(kPitchMotorPerImu)>.001f && Abs(kPitchMotorPerImu)<10000 &&
      Abs(kPitchHorizontalImuRad)<6.284f && kMaxSampleGapMs>=1 && kMaxSampleGapMs<20;
}
static_assert(Valid(),"IMU rotation must be orthonormal and right-handed; linkage scale must be finite/nonzero");
}
