#include "dm_stop.hpp"
namespace damiao {
bool StopSequence::step(StopTransport& bus, uint16_t id, const Limits& limits, uint32_t now) {
  if (id == 0 || id > 15) return false;
  if (phase_ == Abort) {
    if (!bus.abortPending()) return false;
    phase_ = WaitAbort;
  }
  if (phase_ == WaitAbort) {
    if (bus.pending()) return false;
    phase_ = Zero;
  }
  const auto frames = StopFrames(limits);
  if (phase_ == Zero) {
    if (!bus.send(frames[0], id)) return false;
    phase_ = Disable;
  }
  if (phase_ == Disable) {
    if (!bus.send(frames[1], id)) return false;
    phase_ = WaitComplete;
  }
  if (phase_ == WaitComplete) {
    if (bus.pending()) return false;
    phase_ = Holding; last_disable_ = now;
  }
  // Repeated DISABLE solicits feedback while ensuring no old motion is retained.
  if (uint32_t(now-last_disable_) >= 10u) {
    if (!bus.send(DisableFrame(), id)) return false;
    last_disable_ = now;
  }
  return phase_ == Holding;
}
}
