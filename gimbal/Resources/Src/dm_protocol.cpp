#include "dm_protocol.hpp"
#include <algorithm>
#include <cmath>

namespace damiao {
namespace {
uint16_t Encode(float value, float low, float high, unsigned bits) {
  return uint16_t(std::lround((std::clamp(value, low, high) - low) /
      (high - low) * float((1u << bits) - 1u)));
}
float Decode(unsigned value, float bound, unsigned bits) {
  return float(value) / float((1u << bits) - 1u) * (2 * bound) - bound;
}
Payload Special(uint8_t command) {
  Payload data; data.fill(0xff); data[7] = command; return data;
}
}
bool ValidLimits(const Limits& l) {
  return std::isfinite(l.position) && l.position > 0 &&
         std::isfinite(l.velocity) && l.velocity > 0 &&
         std::isfinite(l.torque) && l.torque > 0;
}
bool PackMit(const Limits& l, float p, float v, float kp, float kd, float t,
             Payload& out) {
  if (!ValidLimits(l) || !std::isfinite(p) || !std::isfinite(v) ||
      !std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(t)) return false;
  const auto pi = Encode(p, -l.position, l.position, 16);
  const auto vi = Encode(v, -l.velocity, l.velocity, 12);
  const auto ki = Encode(kp, 0, 500, 12), di = Encode(kd, 0, 5, 12);
  const auto ti = Encode(t, -l.torque, l.torque, 12);
  out = {uint8_t(pi >> 8), uint8_t(pi), uint8_t(vi >> 4),
         uint8_t((vi << 4) | (ki >> 8)), uint8_t(ki), uint8_t(di >> 4),
         uint8_t((di << 4) | (ti >> 8)), uint8_t(ti)};
  return true;
}
bool DecodeFeedback(const Limits& l, const uint8_t* data, size_t size, Feedback& out) {
  if (!ValidLimits(l) || !data || size != 8 || data[6] > 125 || data[7] > 125) return false;
  Feedback value;
  value.id = data[0] & 0x0f; value.state = data[0] >> 4;
  if (value.state != 0 && value.state != 1 &&
      (value.state < 8 || value.state > 14)) return false;
  value.position = Decode((unsigned(data[1]) << 8) | data[2], l.position, 16);
  value.velocity = Decode((unsigned(data[3]) << 4) | (data[4] >> 4), l.velocity, 12);
  value.torque = Decode((unsigned(data[4] & 15) << 8) | data[5], l.torque, 12);
  value.mos_temperature = data[6]; value.rotor_temperature = data[7];
  out = value; return true;
}
Payload EnableFrame() { return Special(0xfc); }
Payload DisableFrame() { return Special(0xfd); }
std::array<Payload, 2> StopFrames(const Limits& l) {
  std::array<Payload, 2> out{DisableFrame(), DisableFrame()};
  PackMit(l, 0, 0, 0, 0, 0, out[0]);
  return out;
}
}
