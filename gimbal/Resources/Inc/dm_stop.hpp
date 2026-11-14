#pragma once
#include "dm_protocol.hpp"
namespace damiao {
// HAL and host mocks implement the same small transport boundary.
class StopTransport {
 public:
  virtual ~StopTransport() = default;
  virtual bool abortPending() = 0;
  virtual bool pending() = 0;
  virtual bool send(const Payload&, uint16_t id) = 0;
};
class StopSequence {
 public:
  bool step(StopTransport&, uint16_t id, const Limits&, uint32_t now);
  bool holding() const { return phase_ == Holding; }
 private:
  enum Phase { Abort, WaitAbort, Zero, Disable, WaitComplete, Holding } phase_ = Abort;
  uint32_t last_disable_ = 0;
};
}
