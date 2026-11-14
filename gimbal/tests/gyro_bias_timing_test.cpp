#include "gyro_bias.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  const float a[]{0,0,9.80665f},g[]{.01f,-.02f,.03f};
  gimbal::GyroBias b(5);uint32_t now=0;
  for(unsigned n=0;n<2000;++n){
    now+=n%17==0?2u:1u;
    const bool ready=b.update(a,g,now);assert(ready==(n==1999));
    assert(b.samples()==n+1);
    b.update(a,g,now);assert(b.samples()==n+1); // duplicate cannot inflate progress
  }
  assert(b.ready() && std::fabs(b.bias()[2]-.03f)<.00001f);
  b.reset();assert(!b.ready());
  assert(!b.update(a,g,10));assert(!b.update(a,g,15));assert(b.samples()==2); // reset retains max gap
  assert(!b.update(a,g,21) && b.samples()==1); // 6-ms gap breaks the stationary sequence
  const float moving[]{.1f,0,0};assert(!b.update(a,moving,22) && b.samples()==0 && !b.stationary());
  assert(!b.update(a,g,23) && b.samples()==1 && b.stationary());
  const float wrong_units[]{0,0,1};assert(!b.update(wrong_units,g,24) && b.samples()==0);
  gimbal::GyroBias strict;
  strict.update(a,g,0);strict.update(a,g,2);assert(strict.samples()==1); // legacy strict default retained
  gimbal::GyroBias tolerant(10);uint32_t jitter_time=0;
  for(unsigned n=0;n<2000;++n) {
    jitter_time+=n%17==0?8u:1u;
    assert(tolerant.update(a,g,jitter_time)==(n==1999));
  }
  tolerant.reset();tolerant.update(a,g,0);tolerant.update(a,g,10);
  assert(tolerant.samples()==2);tolerant.update(a,g,21);assert(tolerant.samples()==1);
  std::cout<<"Gyro bias: bounded jitter, duplicate rejection, gap reset and stationary checks passed\n";
}

