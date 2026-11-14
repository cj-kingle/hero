#pragma once
#include <cstdint>
#include <cmath>
namespace chassis {
struct ChassisImuFeedback {
  float yaw=0,yaw_rate=0;
  uint32_t updated_ms=0;
  bool ready=false;
};
// Euler yaw rate from BODY angular velocity, including roll/pitch tilt.
inline bool EulerYawRate(float roll,float pitch,const float gyro[3],float& rate){
  if(!gyro || !std::isfinite(roll) || !std::isfinite(pitch))return false;
  for(unsigned i=0;i<3;++i)if(!std::isfinite(gyro[i]))return false;
  const float cp=std::cos(pitch);
  if(std::fabs(cp)<.1f)return false;
  rate=(gyro[1]*std::sin(roll)+gyro[2]*std::cos(roll))/cp;
  return std::isfinite(rate);
}
}
