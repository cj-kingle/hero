#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace damiao {
using Payload = std::array<uint8_t, 8>;
// These are ENCODING ranges, not software output limits. Match PMAX/VMAX/TMAX
// read from each actual motor. Zero defaults deliberately reject torque frames.
struct Limits { float position = 0, velocity = 0, torque = 0; };
struct Feedback {
  uint8_t id = 0, state = 0, mos_temperature = 0, rotor_temperature = 0;
  float position = 0, velocity = 0, torque = 0;
};
bool ValidLimits(const Limits& limits);
bool PackMit(const Limits&, float position, float velocity, float kp, float kd,
             float torque, Payload& result);
bool DecodeFeedback(const Limits&, const uint8_t*, size_t, Feedback&);
Payload EnableFrame();
Payload DisableFrame();
// Zero effort first, then disable. If ranges are unknown, both frames disable.
std::array<Payload, 2> StopFrames(const Limits&);
}
