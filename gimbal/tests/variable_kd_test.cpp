#include "pid.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
int main() {
  using namespace gimbal;
  PidConfig c{70,0,16,100,0};c.derivative_from_rate=true;
  c.variable_kd=true;c.small_error_kd=0;
  c.small_d_error_range=.3f*.017453293f;c.full_d_error=1.2f*.017453293f;
  assert(ValidPidErrorShaping(c));
  for(float sign : {-1.f,1.f}) {
    float previous=0;
    for(unsigned n=0;n<=1500;++n) {
      const float error=sign*float(n)*.001f*.017453293f;
      Pid pid(c);pid.update(error,0,.001f,0,.1f);
      const float gain=-pid.terms().d/.1f;
      assert(std::fabs(pid.terms().p-70*error)<1e-6f); // no change to P
      if(n<=300)assert(std::fabs(gain)<1e-5f);
      if(n>=1200)assert(std::fabs(gain-16)<1e-4f);
      assert(gain>=previous-1e-5f && gain-previous<.03f);previous=gain;
    }
  }
  c.small_error_kd=4;Pid near(c);near.update(0,0,.001f,7.1f,.1f);
  assert(std::fabs(near.terms().d+.4f)<1e-6f && near.terms().feedforward==7.1f);
  c.variable_kd=false;Pid legacy(c);legacy.update(0,0,.001f,0,.1f);
  assert(std::fabs(legacy.terms().d+1.6f)<1e-6f); // default behavior preserved
  c.variable_kd=true;c.full_d_error=c.small_d_error_range;assert(!ValidPidErrorShaping(c));
  c.full_d_error=1;c.small_error_kd=-1;assert(!ValidPidErrorShaping(c));
  c.small_error_kd=std::numeric_limits<float>::quiet_NaN();assert(!ValidPidErrorShaping(c));
  std::cout<<"Independent small-error D: zero/configurable gain, smooth bands, P/feedforward isolation and legacy behavior passed\n";
}
