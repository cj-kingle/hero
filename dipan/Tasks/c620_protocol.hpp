#pragma once
#include <cstdint>
#include <array>
namespace chassis {
struct C620Feedback {
  uint16_t encoder = 0;
  int16_t rpm = 0, current_raw = 0;
  uint8_t temperature = 0;
  uint32_t updated_ms = 0;
  bool received = false;
};
bool DecodeC620(uint32_t id, const uint8_t* data, unsigned length,
                C620Feedback& result, uint32_t now_ms);
std::array<uint8_t, 8> PackC620(const std::array<int16_t, 4>& currents);
}
