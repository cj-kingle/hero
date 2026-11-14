#pragma once
#include <cstdint>
namespace hello_world::imu {
struct BMI088HWConfig {
  int* hspi;
  void* acc_cs_port;
  uint16_t acc_cs_pin;
  void* gyro_cs_port;
  uint16_t gyro_cs_pin;
};
inline constexpr int kBMI088ErrStateNoErr=0;
class BMI088 {
 public:
  BMI088(BMI088HWConfig,const float*) {}
  int imuInit(bool) {return 0;}
  void getData(float* acc,float* gyro,float* temp) {
    acc[0]=acc[1]=0;acc[2]=9.80665f;
    gyro[0]=gyro[1]=gyro[2]=0;*temp=25;
  }
};
}
