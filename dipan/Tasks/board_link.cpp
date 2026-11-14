#include "board_link.hpp"
#include <cmath>
namespace gimbal {
namespace {
uint8_t Crc(const uint8_t* data, size_t size) {
  uint8_t crc = 0;
  for (size_t i=0; i<size; ++i) {
    crc ^= data[i];
    for (unsigned j=0; j<8; ++j) crc = uint8_t((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
  }
  return crc;
}
int Read16(const uint8_t* data) {
  const int n = (int(data[0]) << 8) | data[1]; return n >= 32768 ? n-65536 : n;
}
}
bool EncodeCommand(const Command& c, std::array<uint8_t,8>& out) {
  if ((c.mode != Mode::Joint && c.mode != Mode::Imu) ||
      !std::isfinite(c.yaw) || !std::isfinite(c.pitch) ||
      std::fabs(c.yaw) > 1 || std::fabs(c.pitch) > 1 ||
      (c.centered && (c.yaw != 0 || c.pitch != 0))) return false;
  const bool stopped = !c.online || c.emergency_stop;
  const auto yaw = uint16_t(int16_t(std::lround((stopped ? 0 : c.yaw)*1000)));
  const auto pitch = uint16_t(int16_t(std::lround((stopped ? 0 : c.pitch)*1000)));
  out = {0xa1, uint8_t(c.online | (c.emergency_stop << 1) |
      ((c.mode == Mode::Imu) << 2) | (c.centered << 3)), c.sequence,
      uint8_t(yaw >> 8), uint8_t(yaw), uint8_t(pitch >> 8), uint8_t(pitch), 0};
  out[7] = Crc(out.data(), 7); return true;
}
bool EncodeStatus(const BoardStatus& s,std::array<uint8_t,8>& out) {
  if(!std::isfinite(s.yaw_angle)) return false;
  const auto angle=uint16_t(int16_t(std::lround(std::remainder(s.yaw_angle,6.283185307f)*1000)));
  out={0xa2,uint8_t(s.yaw_valid|(s.drive<<1)|(s.imu_ready<<2)|(s.imu_mode<<3)),s.sequence,
      uint8_t(angle>>8),uint8_t(angle),uint8_t(s.reasons>>8),uint8_t(s.reasons),0};
  out[7]=Crc(out.data(),7); return true;
}
bool DecodeStatus(const uint8_t* data,size_t size,BoardStatus& out) {
  if(!data || size!=8 || data[0]!=0xa2 || (data[1]&0xf0) || Crc(data,7)!=data[7]) return false;
  const int angle=Read16(data+3);
  if(angle < -3142 || angle > 3142) return false;
  out={bool(data[1]&1),bool(data[1]&2),bool(data[1]&4),bool(data[1]&8),
      angle*.001f,uint16_t((uint16_t(data[5])<<8)|data[6]),data[2]};
  return true;
}
bool StatusReceiver::receive(const uint8_t* data,size_t size,uint32_t now) {
  BoardStatus next;
  if(!DecodeStatus(data,size,next)) { valid_=false; return false; }
  if(seen_ && next.sequence==status_.sequence) return false;
  if(valid_ && uint32_t(now-last_ms_)<100u && uint8_t(next.sequence-status_.sequence)>127) return false;
  status_=next;last_ms_=now;valid_=seen_=true;return true;
}
bool StatusReceiver::snapshot(uint32_t now,BoardStatus& out) const {
  out={}; if(!valid_ || uint32_t(now-last_ms_)>=100u) return false;
  out=status_;return true;
}
bool DecodeCommand(const uint8_t* data, size_t size, Command& out) {
  if (!data || size != 8 || data[0] != 0xa1 || (data[1] & 0xf0) ||
      Crc(data, 7) != data[7]) return false;
  const int yaw = Read16(data+3), pitch = Read16(data+5);
  if (yaw < -1000 || yaw > 1000 || pitch < -1000 || pitch > 1000) return false;
  Command c;
  c.online = data[1] & 1; c.emergency_stop = data[1] & 2;
  c.mode = data[1] & 4 ? Mode::Imu : Mode::Joint;
  c.centered = data[1] & 8; c.sequence = data[2];
  if ((c.centered || !c.online || c.emergency_stop) && (yaw || pitch)) return false;
  c.yaw = yaw*.001f; c.pitch = pitch*.001f; out = c; return true;
}
bool CommandReceiver::receive(const uint8_t* data, size_t size, uint32_t now) {
  Command next;
  if (!DecodeCommand(data, size, next)) { invalidate(); return false; }
  // A timeout must NOT make a frozen/repeated sequence new again.
  if (seen_sequence_ && next.sequence == command_.sequence) return false;
  if (valid_ && uint32_t(now-last_ms_) < 100u) {
    const uint8_t step = uint8_t(next.sequence-command_.sequence);
    if (step == 0 || step > 127) return false;
  }
  command_ = next; last_ms_ = now; valid_ = seen_sequence_ = true; return true;
}
Command CommandReceiver::snapshot(uint32_t now) const {
  if (!valid_ || uint32_t(now-last_ms_) >= 100u) return {};
  return command_;
}
}
