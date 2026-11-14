#include "chassis_motor_task.hpp"
#include <algorithm>
#include <cmath>

namespace chassis {
FourWheelStatus four_wheel_status;
void FourWheelTest::reset() {
  previous_targets_ = {};
  previous_wheel_gyro_=false;previous_wheel_angle_=0;
  yaw_rate_pid_.reset();
  gyro_rate_pid_.reset();
  for (auto& motor : motors_) motor.reset();
}
FourWheelStatus FourWheelTest::update(const RemoteCommand& command,
    const std::array<C620Feedback, 4>& feedback, uint32_t now, bool transport_fault) {
  return updateCore(command,feedback,now,transport_fault,nullptr);
}
FourWheelStatus FourWheelTest::updateIntegrated(const RemoteCommand& command,
    const std::array<C620Feedback,4>& feedback,const BoardFeedback& board,uint32_t now,bool fault) {
  return updateCore(command,feedback,now,fault,&board);
}
FourWheelStatus FourWheelTest::updateCore(const RemoteCommand& command,
    const std::array<C620Feedback,4>& feedback,uint32_t now,bool transport_fault,const BoardFeedback* board) {
  FourWheelStatus result;
  const uint32_t elapsed=have_update_ ? uint32_t(now-previous_update_ms_) : 1u;
  previous_update_ms_=now;have_update_=true;
  const float dt=board && elapsed>=1u && elapsed<=config::kMaxControlGapMs ? float(elapsed)*.001f : .001f;
  result.rc_online = command.online;
  result.transport_fault = transport_fault;
  bool healthy = true, fault = transport_fault;
  for (unsigned i = 0; i < 4; ++i) {
    const auto& f = feedback[i];
    const bool online = f.received && uint32_t(now - f.updated_ms) < 100u;
    if (online) result.online_mask |= 1u << i;
    healthy = healthy && online && f.temperature < 70;
    fault = fault || (online && (f.temperature >= 70 ||
        ((!board || config::kWheelOverspeedTripEnabled) &&
         std::fabs(float(f.rpm)) > (board ? config::kOverspeedMotorRpm : wheel_config::kOverspeedRpm))));
    motors_[i].setFeedback(f.rpm, f.temperature, f.updated_ms);
  }
  const bool finite = std::isfinite(command.forward) && std::isfinite(command.left) &&
      std::isfinite(command.yaw) && std::isfinite(config_.max_translation_mps) &&
      config_.max_translation_mps > 0 && std::isfinite(config_.max_yaw_rate) &&
      config_.max_yaw_rate > 0;
  if (finite) {
    const float x = std::clamp(command.forward, -1.0f, 1.0f);
    const float y = std::clamp(command.left, -1.0f, 1.0f);
    const float norm = std::max(1.0f, std::hypot(x, y));
    result.velocity = {x / norm * config_.max_translation_mps,
                       y / norm * config_.max_translation_mps,
                       std::clamp(command.yaw, -1.0f, 1.0f) * config_.max_yaw_rate};
    result.targets = SolveMecanumX(result.velocity, config_.kinematics);
  }
  const bool valid = finite && result.targets.valid && result.targets.motor_targets_ready &&
      config_.kinematics.max_motor_rpm > 0;
  auto bench_command = command;
  // Bench mode is manual body-coordinate control, without upper-board/yaw/IMU data.
  // MID/DOWN on the right switch are observe-only; no automatic spin is enabled.
  bench_command.emergency_stop = command.emergency_stop || (!board && command.mode != Mode::Depart);
  bool board_ready=true;
  if(board) {
    // The upper can hold its encoder loop while an initial IMU-mode request
    // is pending. Keep completed local preparation, but the output gate below
    // can optionally inhibit wheel output until actual IMU-mode confirmation.
    const bool awaiting_imu_mode=command.mode!=Mode::Depart &&
        board->readiness.upper_board_ready && !board->upper_mode_ready;
    board_ready=uint32_t(now-board->updated_ms)<100u && board->readiness.yaw_ready &&
        std::isfinite(board->relative_yaw) &&
        (!config::kImuLossTripEnabled || command.mode==Mode::Depart || board->readiness.imu_ready || awaiting_imu_mode);
    if(config::kYawRateControlEnabled && config::kImuLossTripEnabled) board_ready=board_ready && board->chassis_imu.ready &&
        uint32_t(now-board->chassis_imu.updated_ms)<20u &&
        std::isfinite(board->chassis_imu.yaw_rate);
    if(command.mode!=previous_mode_) {
      // Mode selection does not revoke a completed startup preparation.
      // Preserve wheel PID and ramp state for a continuous handover; the planner
      // resets its follow-angle PID when it detects the new mode itself.
      yaw_rate_pid_.reset();gyro_rate_pid_.reset();
    }
    previous_mode_=command.mode;
  }
  state_.update(bench_command, {healthy && valid && board_ready, true, true, true, fault || !valid}, now,
      board ? config::kMaxControlGapMs : 1u,!board || board->readiness.upper_board_ready,board!=nullptr);
  result.state = state_.state(); result.ready_count = state_.readyCount();
  if (result.state != State::Working || !valid ||
      (!board && command.forward == 0 && command.left == 0 && command.yaw == 0)) {
    reset(); planner_.reset(); result.velocity = {}; result.targets.motor_rpm = {}; result.targets.wheel_rpm = {};
    return result;
  }
  if(board && config::kImuLossTripEnabled && !board->upper_mode_ready) {
    // Hold wheels until the upper confirms its requested IMU mode. Keep Working
    // so a normal CAN round trip cannot start another 2000-sample preparation.
    reset();planner_.reset();result.velocity={};result.targets.motor_rpm={};result.targets.wheel_rpm={};
    return result;
  }
  if(board) {
    auto& rate_pid=command.mode==Mode::Gyro ? gyro_rate_pid_ : yaw_rate_pid_;
    const bool usable_imu=board->chassis_imu.ready &&
        uint32_t(now-board->chassis_imu.updated_ms)<20u && std::isfinite(board->chassis_imu.yaw_rate);
    result.velocity=planner_.update(command,true,board->relative_yaw,dt,
        usable_imu ? board->chassis_imu.yaw_rate : std::numeric_limits<float>::quiet_NaN());
    result.target_yaw_rate=result.velocity.yaw_rate;
    if(config::kYawRateControlEnabled && usable_imu) {
      if(std::fabs(result.target_yaw_rate)<.00001f && std::fabs(board->chassis_imu.yaw_rate)<config::kZeroYawRateDeadband) {
        rate_pid.reset();result.velocity.yaw_rate=0;
      } else result.velocity.yaw_rate=rate_pid.update(result.target_yaw_rate,
          board->chassis_imu.yaw_rate,dt,result.target_yaw_rate);
    } else rate_pid.reset(); // never integrate stale/invalid rate; retain the planner's finite feedforward rate
    auto kinematics=config_.kinematics;
    kinematics.max_motor_rpm=command.mode==Mode::Gyro ? config::kGyroMaxMotorRpm : config::kNormalMaxMotorRpm;
    result.targets=SolveMecanumX(result.velocity,kinematics);
    if(!result.targets.valid || !result.targets.motor_targets_ready) { reset();return {}; }
    bool wheels_settled=true;
    for(const auto& f:feedback)wheels_settled &= std::abs(int(f.rpm))<10;
    if(std::fabs(result.velocity.forward)<.000001f && std::fabs(result.velocity.left)<.000001f &&
        std::fabs(result.velocity.yaw_rate)<.000001f && wheels_settled) {
      // Clear residual wheel integral only after near standstill; retain active braking above it.
      reset();result.targets.motor_rpm={};result.targets.wheel_rpm={};return result;
    }
  }
  if(board && command.mode==Mode::Gyro && previous_wheel_gyro_) {
    // Transport only translation into the new body frame before comparing
    // wheel increments. Keep the rotation component and joystick/yaw ramps.
    float p[4];for(unsigned i=0;i<4;++i)p[i]=previous_targets_[i]*config_.kinematics.motor_direction[i];
    const float x=(p[0]+p[1]+p[2]+p[3])*.25f;
    const float y=(-p[0]+p[1]-p[2]+p[3])*.25f;
    const float rotation=(-p[0]-p[1]+p[2]+p[3])*.25f;
    const float delta=std::remainder(board->relative_yaw-previous_wheel_angle_,6.283185307f);
    const float nx=std::cos(delta)*x-std::sin(delta)*y,ny=std::sin(delta)*x+std::cos(delta)*y;
    const float moved[]{nx-ny-rotation,nx+ny-rotation,nx-ny+rotation,nx+ny+rotation};
    float peak=0;for(float rpm:moved)peak=std::max(peak,std::fabs(rpm));
    const float scale=peak>config::kGyroMaxMotorRpm ? config::kGyroMaxMotorRpm/peak : 1.f;
    for(unsigned i=0;i<4;++i)previous_targets_[i]=moved[i]*scale*config_.kinematics.motor_direction[i];
  }
  previous_wheel_gyro_=board && command.mode==Mode::Gyro;
  if(board)previous_wheel_angle_=board->relative_yaw;
  // One common factor limits the largest rotor-speed increment; it preserves the
  // four-wheel relationship instead of independently clipping wheel accelerations.
  float largest_step = 0;
  for (unsigned i = 0; i < 4; ++i)
    largest_step = std::max(largest_step,
        std::fabs(result.targets.motor_rpm[i] - previous_targets_[i]));
  const bool normal=board && command.mode!=Mode::Gyro;
  float ramp=board ? (normal ? config::kNormalRampRpmPerSecond : config::kGyroRampRpmPerSecond) :
      wheel_config::kRampRpmPerSecond;
  if(board && command.mode==Mode::Follow && command.yaw==0) {
    // Project out translation using the inverse-kinematic rotation signs.
    // Only braking/reversal of the applied yaw component gets the faster
    // common wheel ramp; acceleration from rest and gyro mode keep theirs.
    float before_rotation=0,next_rotation=0;
    for(unsigned i=0;i<4;++i) {
      const float sign=(i<2 ? -1.f : 1.f)*config_.kinematics.motor_direction[i];
      before_rotation+=sign*previous_targets_[i];
      next_rotation+=sign*result.targets.motor_rpm[i];
    }
    if(std::fabs(before_rotation)>.001f &&
        (before_rotation*next_rotation<=0 || std::fabs(next_rotation)<std::fabs(before_rotation)))
      ramp=config::kFollowBrakingRampRpmPerSecond;
  }
  const float current_limit=board ? (normal ? config::kNormalCurrentLimitRaw : config::kGyroCurrentLimitRaw) :
      wheel_config::kCurrentLimitRaw;
  const float max_step = ramp * dt;
  const float fraction = largest_step > max_step ? max_step / largest_step : 1.0f;
  for (unsigned i = 0; i < 4; ++i) {
    const float target = previous_targets_[i] + fraction *
        (result.targets.motor_rpm[i] - previous_targets_[i]);
    previous_targets_[i] = result.targets.motor_rpm[i] = target;
    result.targets.wheel_rpm[i] = target * config_.kinematics.motor_direction[i] /
        config_.kinematics.total_reduction;
    // Raw motor targets and raw feedback share the same sign convention.
    motors_[i].setCurrentLimit(current_limit);
    result.commands[i] = int16_t(std::clamp(motors_[i].calculate(target, true, now, dt),
        -current_limit,current_limit));
  }
  return result;
}
}
