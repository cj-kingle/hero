#include "chassis_task.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
using namespace chassis;
static bool Near(float a, float b) { return std::fabs(a-b) < 0.002f; }
int main() {
  MecanumConfig c;
  c.max_motor_rpm = 0;
  constexpr float pi = 3.14159265358979323846f;
  const float rpm_per_mps = 60 / (2*pi*c.wheel_radius_m);
  auto out = SolveMecanumX({1,0,0},c);
  assert(out.valid && out.motor_targets_ready);
  for(unsigned i=0;i<4;++i) {
    const int direction = i < 2 ? 1 : -1;
    assert(Near(out.wheel_rpm[i],rpm_per_mps));
    assert(Near(out.motor_rpm[i],rpm_per_mps*14*direction));
  }
  c.motor_direction = {{0,0,0,0}};
  out = SolveMecanumX({1,0,0},c);
  assert(out.valid && !out.motor_targets_ready);
  for(unsigned i=0;i<4;++i) { assert(Near(out.wheel_rpm[i],rpm_per_mps)); assert(out.motor_rpm[i]==0); }
  c.motor_direction = kWheelMotorDirections;
  out = SolveMecanumX({0,0,0},c);
  assert(out.valid && out.motor_targets_ready);
  for(float rpm:out.motor_rpm) assert(rpm==0);
  out = SolveMecanumX({0,1,0},c);
  const int left_sign[] = {-1,1,-1,1};
  for(unsigned i=0;i<4;++i) assert(Near(out.wheel_rpm[i],left_sign[i]*rpm_per_mps));
  out = SolveMecanumX({0,0,1},c);
  const int turn_sign[] = {-1,-1,1,1};
  for(unsigned i=0;i<4;++i) assert(Near(out.wheel_rpm[i],turn_sign[i]*.5f*rpm_per_mps));
  out = SolveMecanumX({1,1,0},c);
  assert(Near(out.wheel_rpm[0],0) && Near(out.wheel_rpm[2],0));
  assert(Near(out.wheel_rpm[1],2*rpm_per_mps) && Near(out.wheel_rpm[3],2*rpm_per_mps));
  // Reconstruct an arbitrary twist from wheel speeds as an independent inverse check.
  const Velocity v{.3f,-.2f,.7f};
  auto mixed = SolveMecanumX(v,c);
  const auto& n = mixed.wheel_rpm;
  assert(Near((n[0]+n[1]+n[2]+n[3])/(4*rpm_per_mps),v.forward));
  assert(Near((-n[0]+n[1]-n[2]+n[3])/(4*rpm_per_mps),v.left));
  assert(Near((-n[0]-n[1]+n[2]+n[3])/(4*.5f*rpm_per_mps),v.yaw_rate));
  for(unsigned i=0;i<4;++i)
    assert(Near(mixed.motor_rpm[i],n[i]*14*c.motor_direction[i]));
  c.max_motor_rpm=300;
  auto limited = SolveMecanumX(v,c);
  assert(limited.scale>0 && limited.scale<1);
  for(unsigned i=0;i<4;++i) {
    assert(std::fabs(limited.motor_rpm[i])<=300.001f);
    assert(Near(limited.motor_rpm[i],mixed.motor_rpm[i]*limited.scale));
  }
  auto reversed=SolveMecanumX({-v.forward,-v.left,-v.yaw_rate},c);
  for(unsigned i=0;i<4;++i) assert(Near(reversed.motor_rpm[i],-limited.motor_rpm[i]));
  c.wheel_radius_m=0; assert(!SolveMecanumX(v,c).valid);
  c=MecanumConfig{}; c.motor_direction[0]=2; assert(!SolveMecanumX(v,c).valid);
  c=MecanumConfig{}; c.total_reduction=-1; assert(!SolveMecanumX(v,c).valid);
  c=MecanumConfig{}; c.max_motor_rpm=-1; assert(!SolveMecanumX(v,c).valid);
  c=MecanumConfig{};
  out=SolveMecanumX({std::numeric_limits<float>::quiet_NaN(),0,0},c);
  assert(!out.valid); for(float rpm:out.motor_rpm) assert(rpm==0);
  out=SolveMecanumX({std::numeric_limits<float>::max(),0,std::numeric_limits<float>::max()},c);
  assert(!out.valid);
  std::puts("PASS: X mecanum translation, rotation, units, motor signs, common limiting, invalid inputs");
}
