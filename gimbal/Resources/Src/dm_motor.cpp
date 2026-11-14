#include "dm_motor.hpp"
#include <cmath>
namespace damiao {
bool Motor::configured() const {
  return config_.confirmed && config_.control_id > 0 && config_.control_id <= 15 &&
      config_.feedback_id <= 0x7ff && ValidLimits(config_.limits) &&
      (config_.direction == 1 || config_.direction == -1) &&
      std::isfinite(config_.motor_per_joint) && config_.motor_per_joint > 0 &&
      std::isfinite(config_.feedback_wrap) && config_.feedback_wrap > 0;
}
bool Motor::onFeedback(uint16_t id, const uint8_t* data, size_t size, uint32_t now) {
  Feedback next;
  return DecodeFeedback(config_.limits, data, size, next) && acceptFeedback(id, next, now);
}
bool Motor::acceptFeedback(uint16_t id, const Feedback& next, uint32_t now) {
  if (!std::isfinite(config_.feedback_wrap) || config_.feedback_wrap <= 0 ||
      id != config_.feedback_id || !ValidLimits(config_.limits) ||
      next.id != config_.control_id || next.mos_temperature > 125 || next.rotor_temperature > 125 ||
      (next.state != 0 && next.state != 1 && (next.state < 8 || next.state > 14)) ||
      !std::isfinite(next.position) || !std::isfinite(next.velocity) || !std::isfinite(next.torque)) return false;
  if (!received_ || uint32_t(now - last_ms_) >= 100u) {
    // Relative axes lose their captured zero. Absolute axes use raw shaft position.
    continuous_ = next.position; zeroed_ = false;
    rate_anchor_ = continuous_; rate_anchor_ms_ = now; position_rate_ = 0;
  } else {
    const float raw_delta=next.position-feedback_.position;
    const float max_delta = config_.limits.velocity * float(uint32_t(now-last_ms_)) * .001f + .03f;
    float delta=std::remainder(raw_delta,config_.feedback_wrap);
    // The configured shaft wrap and the MIT packet's +/-PMAX range need not
    // match (e.g. 2*pi versus 25 rad). Recognize a native range rollover only
    // when BOTH samples are at opposite endpoints and the resulting motion is
    // physically reachable. Bounded axes still reject interior jumps below.
    const float bound=config_.limits.position;
    const float native_delta=std::remainder(raw_delta,2.f*bound);
    if(feedback_.position*next.position<0 && std::fabs(raw_delta)>bound &&
        std::fabs(feedback_.position)>bound-max_delta &&
        std::fabs(next.position)>bound-max_delta && std::fabs(native_delta)<=max_delta)
      delta=native_delta;
    // Continuous absolute Yaw can cross encoder wraps or receive a finite
    // position discontinuity without losing the whole servo. Keep unwrapping,
    // and retain protocol, freshness, raw-speed, state and temperature checks.
    const bool continuous_absolute=config_.absolute_position && config_.continuous_absolute;
    if (!std::isfinite(delta) || (!continuous_absolute && std::fabs(delta) > max_delta)) {
      received_ = false; zeroed_ = false; return false;
    }
    continuous_ += delta;
    const uint32_t elapsed = now-rate_anchor_ms_;
    if(elapsed>=10u) {
      position_rate_ = (continuous_-rate_anchor_)/(float(elapsed)*.001f);
      rate_anchor_ = continuous_; rate_anchor_ms_ = now;
    }
  }
  feedback_ = next; last_ms_ = now; received_ = true; return true;
}
float Motor::positionRate(uint32_t now) const {
  if(!configured() || !received_ || uint32_t(now-last_ms_)>=100u) return 0;
  return position_rate_*float(config_.direction)/config_.motor_per_joint;
}
JointFeedback Motor::joint(uint32_t now) const {
  JointFeedback out;
  out.online = received_ && uint32_t(now - last_ms_) < 100u;
  out.healthy = out.online && configured() && feedback_.state <= 1 &&
      feedback_.mos_temperature < 70 && feedback_.rotor_temperature < 70;
  out.zeroed = config_.absolute_position ? configured() && out.online : zeroed_;
  out.enabled = feedback_.state == 1; out.state = feedback_.state;
  if (!configured()) return out;
  const float ratio = float(config_.direction) / config_.motor_per_joint;
  // Normalize the absolute encoder's wrap around its existing zero. Never make
  // the first received angle the reference, including after reconnect/reset.
  out.position = (config_.absolute_position ?
      (config_.continuous_absolute ? continuous_ : std::remainder(feedback_.position, config_.feedback_wrap)) :
      continuous_ - zero_) * ratio;
  out.velocity = feedback_.velocity * ratio;
  // Power-conjugate joint torque. Runtime output is converted back below.
  out.torque = feedback_.torque * float(config_.direction) * config_.motor_per_joint;
  return out;
}
bool Motor::captureZero(uint32_t now) {
  if (config_.absolute_position) return false; // do not shift an absolute axis's reference
  if (!joint(now).healthy || feedback_.state != 0 || std::fabs(feedback_.velocity) > .05f) return false;
  zero_ = continuous_; zeroed_ = true; return true;
}
bool Motor::torqueFrame(float torque, Payload& data) const {
  if (!configured() || !std::isfinite(torque)) return false;
  return PackMit(config_.limits, 0, 0, 0, 0,
                 torque * float(config_.direction) / config_.motor_per_joint, data);
}
}
