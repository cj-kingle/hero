#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
namespace gimbal {
enum class Mode : uint8_t { Joint = 0, Imu = 1 };
struct Command {
  bool online = false, emergency_stop = true, centered = false;
  Mode mode = Mode::Joint;
  float yaw = 0, pitch = 0;
  uint8_t sequence = 0;
};
// Lower-to-upper command ID, verified during the board communication bench test.
inline constexpr uint16_t kBoardCommandId = 0x300;
inline constexpr uint16_t kBoardStatusId = 0x301;
struct BoardStatus {
  bool yaw_valid=false, drive=false, imu_ready=false, imu_mode=false;
  float yaw_angle=0; // wrapped signed motor/joint yaw, radians; chassis forward offset is LOWER configuration
  uint16_t reasons=0;
  uint8_t sequence=0;
};
bool EncodeStatus(const BoardStatus&, std::array<uint8_t,8>&);
bool DecodeStatus(const uint8_t*, size_t, BoardStatus&);
class StatusReceiver {
 public:
  bool receive(const uint8_t*,size_t,uint32_t);
  bool snapshot(uint32_t,BoardStatus&) const;
 private:
  BoardStatus status_{};
  uint32_t last_ms_=0;
  bool valid_=false,seen_=false;
};
// [A1, flags, sequence, yaw_i16_BE, pitch_i16_BE, crc8], axes in [-1000,1000].
// flags: bit0 RC online, bit1 stop, bit2 IMU mode, bit3 all controls centered.
// Lower board MUST preserve sequence while replaying the SAME RC sample.
bool EncodeCommand(const Command&, std::array<uint8_t,8>&);
bool DecodeCommand(const uint8_t*, size_t, Command&);
class CommandReceiver {
 public:
  bool receive(const uint8_t*, size_t, uint32_t now);
  Command snapshot(uint32_t now) const;
  void invalidate() { valid_ = false; }
 private:
  Command command_{};
  uint32_t last_ms_ = 0;
  bool valid_ = false, seen_sequence_ = false;
};
class WatchdogGate {
 public:
  void completedControl(uint32_t now) { last_ms_ = now; started_ = true; }
  bool mayFeed(bool new_valid_frame, bool rc_online, uint32_t now) const {
    // Deliberately independent of Dead/Preparing/Working. Valid estop frames feed.
    return new_valid_frame && rc_online && started_ && uint32_t(now-last_ms_) < 20u;
  }
 private:
  uint32_t last_ms_ = 0;
  bool started_ = false;
};
}
