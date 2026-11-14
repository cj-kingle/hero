#include "yaw_calibration.hpp"
#include <cmath>
#include <cstring>
#include <initializer_list>
namespace chassis {
namespace {
constexpr uint32_t magic=0x59415701,commit=0x43414c31,erased=0xffffffff;
uint32_t Crc(uint32_t bits) {
  uint32_t crc=0xffffffff;
  for(uint32_t value:{magic,bits}) for(unsigned i=0;i<32;++i) {
    const bool bit=(crc^value)&1u;crc>>=1;value>>=1;if(bit)crc^=0xedb88320u;
  }
  return ~crc;
}
bool ReadRecord(const CalibrationJournal& j,size_t i,float& value) {
  if(j.read(i)!=magic || j.read(i+3)!=commit) return false;
  const uint32_t bits=j.read(i+1);
  if(j.read(i+2)!=Crc(bits)) return false;
  std::memcpy(&value,&bits,sizeof(value));
  return std::isfinite(value) && std::fabs(value)<=3.142f;
}
}
bool CalibrationJournal::load(float& value) const {
  if(!read) return false;
  bool found=false;
  for(size_t i=0;i+3<words;i+=4) {float v;if(ReadRecord(*this,i,v)){value=v;found=true;}}
  return found;
}
CalibrationSave CalibrationJournal::save(float value) const {
  if(!read || !program || !std::isfinite(value) || std::fabs(value)>3.142f) return CalibrationSave::FlashError;
  float old;
  if(load(old) && old==value) return CalibrationSave::Saved;
  size_t next=0;
  for(size_t i=0;i+3<words;i+=4)
    if(read(i)!=erased || read(i+1)!=erased || read(i+2)!=erased || read(i+3)!=erased)next=i+4;
  if(next+3>=words)return CalibrationSave::Full;
  uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));
  const uint32_t record[]{magic,bits,Crc(bits),commit};
  for(unsigned n=0;n<4;++n) if(!program(next+n,record[n]))return CalibrationSave::FlashError;
  float check;
  return ReadRecord(*this,next,check) && check==value ? CalibrationSave::Saved : CalibrationSave::FlashError;
}
}
