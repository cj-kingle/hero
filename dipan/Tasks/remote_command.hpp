#pragma once
#include "DT7.hpp"
#include <cstdint>
namespace chassis {
enum class Mode : uint8_t { Depart, Follow, Gyro };
struct RemoteCommand {
  bool online = false, emergency_stop = true, centered = false;
  Mode mode = Mode::Depart;
  float forward = 0, left = 0, yaw = 0, pitch = 0;
};
// Commands are normalized. Timestamp wrap is handled by unsigned subtraction.
RemoteCommand ProcessRemote(const hello_world::devices::remote_control::DT7& rc,
                            bool received, uint32_t last_rx_ms, uint32_t now_ms);
}
