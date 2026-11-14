#include "dm_parameters.hpp"
#include <cstring>
#include <cmath>
namespace damiao {
namespace { constexpr uint8_t registers[]{7,8,10,21,22,23}; }
Payload ParameterReader::request(unsigned slot) {
  pending_slot_=slot%6; pending_=true;
  return {uint8_t(id_),uint8_t(id_>>8),0x33,registers[slot%6],0,0,0,0};
}
bool ParameterReader::receive(uint16_t can_id,const uint8_t* d,size_t n) {
  if (!d || n!=8 || can_id>0x7ff || d[2]!=0x33 ||
      (unsigned(d[0]) | (unsigned(d[1])<<8))!=id_) return false;
  unsigned slot=0;
  while(slot<6 && registers[slot]!=d[3]) ++slot;
  if(slot==6 || !pending_ || slot!=pending_slot_ ||
      (reply_id_!=0xffff && reply_id_!=can_id)) return false;
  const uint32_t bits=uint32_t(d[4])|(uint32_t(d[5])<<8)|(uint32_t(d[6])<<16)|(uint32_t(d[7])<<24);
  float value; std::memcpy(&value,&bits,sizeof(value));
  if(slot==0) { if(bits>0x7ff || bits!=can_id) return false; master_=bits; }
  else if(slot==1) { if(bits!=id_) return false; esc_=bits; }
  else if(slot==2) { if(bits<1 || bits>7) return false; mode_=bits; }
  else {
    if(!std::isfinite(value) || value<=0 || value>1000) return false;
    if(slot==3) limits_.position=value;
    if(slot==4) limits_.velocity=value;
    if(slot==5) limits_.torque=value;
  }
  reply_id_=can_id; mask_|=1u<<slot; pending_=false; return true;
}
bool ParameterReader::ready() const {
  return complete() && master_==reply_id_ && esc_==id_ && mode_==1 && ValidLimits(limits_);
}
MotorConfig ParameterReader::motorConfig(int direction,float wrap) const {
  return {id_,uint16_t(master_),limits_,direction,1,wrap,ready()};
}
}
