#include "yaw_calibration.hpp"
#include <array>
#include <cassert>
#include <iostream>
#include <limits>
namespace {
std::array<uint32_t,16> flash;
int remaining=-1;
uint32_t Read(size_t i){return flash.at(i);}
bool Write(size_t i,uint32_t v){
  if(remaining==0)return false;
  if(remaining>0)--remaining;
  if(flash.at(i)!=0xffffffff)return false;
  flash[i]=v;return true;
}
void Erase(){flash.fill(0xffffffff);remaining=-1;}
}
int main(){
  using namespace chassis;CalibrationJournal j{flash.size(),Read,Write};float v=0;
  Erase();assert(!j.load(v));assert(j.save(.7f)==CalibrationSave::Saved);
  assert(j.load(v) && v==.7f);
  auto snapshot=flash;assert(j.save(.7f)==CalibrationSave::Saved && flash==snapshot);
  for(int n=0;n<4;++n){
    flash=snapshot;remaining=n;assert(j.save(1.2f)==CalibrationSave::FlashError);
    assert(j.load(v) && v==.7f); // a reset during any word retains the old committed calibration
    remaining=-1;assert(j.save(-.3f)==CalibrationSave::Saved && j.load(v) && v==-.3f);
  }
  Erase();assert(j.save(.1f)==CalibrationSave::Saved);assert(j.save(.2f)==CalibrationSave::Saved);
  flash[5]^=1;assert(j.load(v) && v==.1f); // checksum rejects corruption
  assert(j.save(.3f)==CalibrationSave::Saved);assert(j.save(.4f)==CalibrationSave::Saved);
  snapshot=flash;assert(j.save(.5f)==CalibrationSave::Full && snapshot==flash && j.load(v) && v==.4f);
  assert(j.save(std::numeric_limits<float>::infinity())==CalibrationSave::FlashError);
  std::cout<<"Calibration journal: persistence, interrupted writes, CRC, full storage and invalid values passed\n";
}
