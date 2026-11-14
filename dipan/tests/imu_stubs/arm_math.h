#pragma once
#include <cmath>
#include <cstdint>
// Host equivalents of the CMSIS arithmetic primitives. The production AHRS
// and IMU processing code itself is compiled unchanged into the test.
inline int arm_atan2_f32(float y,float x,float* out) { *out=std::atan2(y,x);return 0; }
inline void arm_scale_f32(const float* a,float scale,float* out,uint32_t n) {
  for(uint32_t i=0;i<n;++i)out[i]=a[i]*scale;
}
inline void arm_add_f32(const float* a,const float* b,float* out,uint32_t n) {
  for(uint32_t i=0;i<n;++i)out[i]=a[i]+b[i];
}
inline void arm_quaternion_product_single_f32(const float* a,const float* b,float* out) {
  out[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
  out[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
  out[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
  out[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
