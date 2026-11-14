#include "c620_protocol.hpp"
#include <algorithm>
namespace chassis {
bool DecodeC620(uint32_t id, const uint8_t* data, unsigned length,
                C620Feedback& result, uint32_t now) {
  if (!data || length != 8 || id < 0x201 || id > 0x204) return false;
  const uint16_t encoder = (uint16_t(data[0]) << 8) | data[1];
  if (encoder > 8191 || data[6] >= 125) return false;
  auto signed16 = [](unsigned high, unsigned low) {
    const int value = int((high << 8) | low);
    return int16_t(value >= 32768 ? value - 65536 : value);
  };
  result = {encoder, signed16(data[2], data[3]), signed16(data[4], data[5]),
            data[6], now, true};
  return true;
}
std::array<uint8_t, 8> PackC620(const std::array<int16_t, 4>& currents) {
  std::array<uint8_t, 8> data{};
  for (unsigned i = 0; i < 4; ++i) {
    const auto value = uint16_t(std::clamp<int>(currents[i], -16384, 16384));
    data[i*2] = uint8_t(value >> 8); data[i*2+1] = uint8_t(value);
  }
  return data;
}
}
