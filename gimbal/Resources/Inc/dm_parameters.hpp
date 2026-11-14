#pragma once
#include "dm_motor.hpp"
namespace damiao {
// Read-only register protocol from dmBots/motor-sdk DM_CAN.py.
// Never emits register writes, flash saves, zero-setting or motor enables.
class ParameterReader {
 public:
  explicit ParameterReader(uint16_t control_id) : id_(control_id) {}
  Payload request(unsigned slot);
  bool receive(uint16_t can_id, const uint8_t*, size_t);
  bool complete() const { return mask_ == 63u; }
  bool ready() const;
  MotorConfig motorConfig(int direction, float feedback_wrap) const;
  uint32_t mask() const { return mask_; }
  uint32_t mode() const { return mode_; }
 private:
  uint16_t id_, reply_id_=0xffff;
  uint32_t mask_=0, master_=0xffff, esc_=0, mode_=0;
  Limits limits_{};
  unsigned pending_slot_=0;
  bool pending_=false;
};
}
