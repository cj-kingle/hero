#include "chassis_imu_feedback.hpp"
#include "chassis_imu_config.hpp"
#include "chassis_config.hpp"
#include "chassis_motor_task.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
int main(){
  using namespace chassis;
  float rate=0;const float g[]{0,2,3};
  assert(EulerYawRate(0,0,g,rate) && std::fabs(rate-3)<1e-5f);
  assert(EulerYawRate(1.570796327f,0,g,rate) && std::fabs(rate-2)<1e-5f);
  assert(!EulerYawRate(0,1.570796327f,g,rate));
  const float bad[]{0,0,std::numeric_limits<float>::infinity()};assert(!EulerYawRate(0,0,bad,rate));
  Pid pid(config::kYawRatePid);
  const float saturated_target=.9f*config::kMotion.max_yaw_rate;
  for(unsigned i=0;i<10000;++i) {
    const auto out=pid.update(saturated_target,0,.001f,saturated_target);
    assert(out<=config::kYawRatePid.output_limit && out>=saturated_target);
  }
  assert(std::fabs(pid.integral())<.001f); // saturation includes feedforward
  pid.reset();assert(std::fabs(pid.update(.2f,.2f,.001f,.2f)-.2f)<1e-5f);
  pid.reset();assert(pid.update(0,.1f,.001f,0)<0); // oppose unwanted positive spin
  pid.reset();assert(pid.update(.1f,.2f,.001f,.1f)<.1f); // gyro faster than request: reduce effort
  auto motion=config::kMotion;motion.follow_pid=config::kFollowPid;
  // Actual closing speed advances braking; moving away does not. Once stopped,
  // the reserve disappears and alignment resumes without a timer lockout.
  for(float sign : {-1.f,1.f}) {
    ChassisController fast(motion),slow(motion),away(motion);
    RemoteCommand cmd;cmd.online=true;cmd.emergency_stop=false;cmd.mode=Mode::Follow;
    Velocity f,s,a;
    for(unsigned n=0;n<2000;++n) {
      f=fast.update(cmd,true,sign*.3f,.001f,sign*2.f);
      s=slow.update(cmd,true,sign*.3f,.001f,0);
      a=away.update(cmd,true,sign*.3f,.001f,-sign*2.f);
    }
    assert(std::fabs(f.yaw_rate)+.2f<std::fabs(s.yaw_rate));
    assert(std::fabs(a.yaw_rate-s.yaw_rate)<1e-5f);
    for(unsigned n=0;n<2000;++n)f=fast.update(cmd,true,sign*.3f,.001f,0);
    assert(std::fabs(f.yaw_rate-s.yaw_rate)<1e-5f);
  }
  ChassisController follow(motion);RemoteCommand follow_cmd;
  follow_cmd.online=true;follow_cmd.emergency_stop=false;follow_cmd.centered=true;follow_cmd.mode=Mode::Follow;
  // Earlier arrival slowdown and faster braking preserve gyro/manual turning.
  // Inputs here are prescribed, not a plant model.
  auto old_motion=motion;old_motion.follow_braking_acceleration=4.8f;
  old_motion.follow_deceleration=6.0f;
  for(float sign : {-1.f,1.f}) {
    ChassisController earlier(motion),old(old_motion);
    Velocity a,b;
    for(unsigned i=0;i<2000;++i) {
      a=earlier.update(follow_cmd,true,sign*1.f,.001f);
      b=old.update(follow_cmd,true,sign*1.f,.001f);
    }
    assert(std::fabs(a.yaw_rate)+.2f<std::fabs(b.yaw_rate));
    ChassisController braking(motion);
    for(unsigned i=0;i<2000;++i)a=braking.update(follow_cmd,true,sign*3.14f,.001f);
    assert(std::fabs(a.yaw_rate-sign*motion.max_yaw_rate)<1e-4f);
    const float before=a.yaw_rate;
    a=braking.update(follow_cmd,true,0,.001f);
    assert(std::fabs(before-a.yaw_rate-sign*motion.follow_deceleration*.001f)<1e-5f);
    ChassisController manual(motion),old_manual(old_motion);
    auto turning=follow_cmd;turning.yaw=sign;
    for(unsigned i=0;i<1000;++i) {
      a=manual.update(turning,true,sign*.2f,.001f);
      b=old_manual.update(turning,true,sign*.2f,.001f);
      assert(std::fabs(a.yaw_rate-b.yaw_rate)<1e-6f);
    }
  }
  const float below_stop=motion.follow_stop_angle*.5f;
  const float within_band=(motion.follow_stop_angle+motion.follow_start_angle)*.5f;
  assert(follow.update(follow_cmd,true,below_stop,.001f).yaw_rate==0);
  assert(follow.update(follow_cmd,true,within_band,.001f).yaw_rate==0); // below restart threshold
  assert(follow.update(follow_cmd,true,motion.follow_start_angle*2,.001f).yaw_rate>0);
  assert(follow.update(follow_cmd,true,within_band,.001f).yaw_rate>0); // remain active until stop threshold
  Velocity aligned;
  for(unsigned i=0;i<2000;++i)aligned=follow.update(follow_cmd,true,below_stop,.001f);
  assert(aligned.yaw_rate==0 && follow.update(follow_cmd,true,within_band,.001f).yaw_rate==0);
  // Follow is bounded by both its ceiling and the remaining stopping distance;
  // gyro has a distinct, continuously adjustable target.
  for(unsigned i=0;i<3000;++i)aligned=follow.update(follow_cmd,true,2.5f,.001f);
  const float approach_limit=std::sqrt(2.f*motion.follow_braking_acceleration*(2.5f-motion.follow_stop_angle));
  assert(std::fabs(aligned.yaw_rate-std::min(config::kFollowMaxYawRate,approach_limit))<1e-4f);
  follow.reset();
  for(unsigned i=0;i<1000;++i)aligned=follow.update(follow_cmd,true,.05f,.001f);
  assert(std::fabs(aligned.yaw_rate-config::kFollowPid.kp*.05f)<1e-4f);
  // Ideal chassis following a fixed world heading checks reference braking,
  // not real tire traction or mechanical closed-loop stability.
  for(float sign : {-1.f,1.f}) {
    follow.reset();float angle=sign*2.5f,previous=0;
    for(unsigned i=0;i<8000;++i) {
      aligned=follow.update(follow_cmd,true,angle,.001f);
      assert(std::fabs(aligned.yaw_rate-previous)<=std::max(motion.yaw_acceleration,motion.follow_deceleration)*.001f+1e-5f);
      assert(sign*aligned.yaw_rate>=-1e-5f); // no alternating rotation near alignment
      previous=aligned.yaw_rate;angle-=aligned.yaw_rate*.001f;
    }
    assert(std::fabs(angle)<=motion.follow_stop_angle+.001f && aligned.yaw_rate==0);
  }
  follow_cmd.mode=Mode::Gyro;
  for(float sign : {-1.f,1.f}) {
    ChassisController moving(motion);auto cmd=follow_cmd;cmd.mode=Mode::Follow;cmd.yaw=sign*.6f;
    float angle=0,last_rate=0;
    for(unsigned i=0;i<6000;++i) {
      const auto v=moving.update(cmd,true,angle,.001f);
      assert(std::fabs(v.yaw_rate)<=motion.max_yaw_rate);
      assert(std::fabs(v.yaw_rate-last_rate)<=std::max(motion.yaw_acceleration,motion.follow_deceleration)*.001f+1e-5f);
      angle+=(cmd.yaw*motion.follow_yaw_feedforward-v.yaw_rate)*.001f;last_rate=v.yaw_rate;
    }
    assert(std::fabs(angle)<.011f && std::fabs(last_rate-cmd.yaw*motion.follow_yaw_feedforward)<.01f);
    cmd.yaw=0;
    for(unsigned i=0;i<6000;++i) {
      const auto v=moving.update(cmd,true,angle,.001f);angle-=v.yaw_rate*.001f;last_rate=v.yaw_rate;
    }
    assert(std::fabs(angle)<=motion.follow_stop_angle+.001f && last_rate==0);
  }
  for(unsigned i=0;i<3000;++i)aligned=follow.update(follow_cmd,true,1.f,.001f);
  assert(std::fabs(aligned.yaw_rate-config::kGyroYawRate)<1e-4f);
  FourWheelTest drive;RemoteCommand cmd;cmd.online=true;cmd.emergency_stop=false;cmd.centered=true;
  BoardFeedback board;board.readiness.yaw_ready=board.readiness.upper_board_ready=true;
  std::array<C620Feedback,4> wheels{};
  FourWheelStatus out;
  auto step=[&](uint32_t now){
    board.updated_ms=now;for(auto& f:wheels)f={0,0,0,30,now,true};
    return drive.updateIntegrated(cmd,wheels,board,now,false);
  };
  for(uint32_t t=0;t<2200;++t)out=step(t);
  assert(out.state==State::Working); // no chassis IMU: use planner feedforward, without a readiness interlock
  for(uint32_t t=2200;t<4400;++t){board.chassis_imu={0,0,t,true};out=step(t);}
  assert(out.state==State::Working);
  board.chassis_imu={0,.1f,4400,true};out=step(4400);
  assert(out.target_yaw_rate==0 && out.velocity.yaw_rate<0);
  out=step(4420);assert(out.state!=State::Working);for(auto current:out.commands)assert(current==0); // actual 20 ms task gap still trips
  // Replay a follow stop through the complete wheel-ramp path. Fresh wheel
  // feedback follows the prior command, so this checks command latency, not traction.
  for(float sign : {-1.f,1.f}) {
    FourWheelTest stopping;RemoteCommand stop_cmd;
    stop_cmd.online=true;stop_cmd.emergency_stop=false;stop_cmd.centered=true;stop_cmd.mode=Mode::Follow;
    BoardFeedback stop_board;stop_board.readiness.yaw_ready=stop_board.readiness.upper_board_ready=stop_board.readiness.imu_ready=true;
    stop_board.upper_mode_ready=true;uint32_t time=0;FourWheelStatus current;
    const auto tick=[&]() {
      stop_board.updated_ms=++time;stop_board.chassis_imu={0,0,time,true};
      for(unsigned i=0;i<4;++i)wheels[i]={0,int16_t(std::lround(current.targets.motor_rpm[i])),0,30,time,true};
      current=stopping.updateIntegrated(stop_cmd,wheels,stop_board,time,false);
      return current;
    };
    for(unsigned i=0;i<2200;++i)tick();
    assert(current.state==State::Working);stop_board.relative_yaw=sign*1.f;
    for(unsigned i=0;i<1800;++i) {
      const auto before=current.targets.motor_rpm;tick();
      for(unsigned j=0;j<4;++j)assert(std::fabs(current.targets.motor_rpm[j]-before[j])<=config::kNormalRampRpmPerSecond*.001f+.01f);
    }
    assert(std::fabs(current.targets.motor_rpm[0])>1000);
    stop_board.relative_yaw=0;float largest_brake_step=0;
    for(unsigned i=0;i<1000;++i) {
      const auto before=current.targets.motor_rpm;tick();assert(current.state==State::Working);
      for(unsigned j=0;j<4;++j) {
        largest_brake_step=std::max(largest_brake_step,std::fabs(current.targets.motor_rpm[j]-before[j]));
        assert(std::abs(int(current.commands[j]))<=config::kNormalCurrentLimitRaw);
      }
    }
    assert(largest_brake_step>config::kNormalRampRpmPerSecond*.001f+.1f);
    assert(largest_brake_step<=config::kFollowBrakingRampRpmPerSecond*.001f+.01f);
    for(auto rpm:current.targets.motor_rpm)assert(std::fabs(rpm)<.01f);
  }
  // Fixed gimbal-frame translation survives multiple rotating-body wraps all
  // the way through the wheel ramp, in either spin direction.
  for(float spin : {-1.f,1.f}) {
    FourWheelTest gyro;RemoteCommand moving;
    moving.online=true;moving.emergency_stop=false;moving.centered=true;moving.mode=Mode::Gyro;
    BoardFeedback b;b.readiness.yaw_ready=b.readiness.upper_board_ready=true;b.upper_mode_ready=true;
    FourWheelStatus current;float angle=0;
    for(uint32_t n=0;n<9000;++n) {
      if(n>2200) {moving.forward=.6f;moving.left=.2f;moving.centered=false;angle-=spin*config::kGyroYawRate*.001f;}
      b.relative_yaw=std::remainder(angle,6.283185307f);b.updated_ms=n;
      // No IMU rate correction here: isolate reference transport and wheel ramp.
      for(unsigned i=0;i<4;++i)wheels[i]={0,int16_t(std::lround(current.targets.motor_rpm[i])),0,30,n,true};
      current=gyro.updateIntegrated(moving,wheels,b,n,false);
      if(n>5000) {
        assert(current.state==State::Working);
        float p[4];for(unsigned i=0;i<4;++i)p[i]=current.targets.motor_rpm[i]*config::kWheels.motor_direction[i];
        const float conversion=60.f*config::kWheels.total_reduction/(6.283185307f*config::kWheels.wheel_radius_m);
        const float x=(p[0]+p[1]+p[2]+p[3])*.25f/conversion;
        const float y=(-p[0]+p[1]-p[2]+p[3])*.25f/conversion;
        assert(std::fabs(std::cos(angle)*x+std::sin(angle)*y-.6f*motion.max_speed)<.01f);
        assert(std::fabs(-std::sin(angle)*x+std::cos(angle)*y-.2f*motion.max_speed)<.01f);
        for(auto rpm:current.targets.motor_rpm)assert(std::fabs(rpm)<=config::kGyroMaxMotorRpm+.01f);
      }
    }
    moving.emergency_stop=true;b.updated_ms=9000;
    current=gyro.updateIntegrated(moving,wheels,b,9000,false);
    for(auto effort:current.commands)assert(effort==0);
  }
  FourWheelTest jitter_drive;RemoteCommand neutral;
  neutral.online=true;neutral.emergency_stop=false;neutral.centered=true;
  BoardFeedback ready;ready.readiness.yaw_ready=ready.readiness.upper_board_ready=true;
  uint32_t t=0;int16_t simulated_rpm=0;
  auto jitter_step=[&](unsigned gap){
    t+=gap;ready.updated_ms=t;ready.chassis_imu={0,0,t,true};
    for(auto& f:wheels)f={0,simulated_rpm,0,30,t,true};
    return jitter_drive.updateIntegrated(neutral,wheels,ready,t,false);
  };
  for(unsigned n=0;n<2100;++n)out=jitter_step(n%31==0?2:1);
  assert(out.state==State::Working); // bounded jitter must not reset neutral preparation
  neutral.forward=1;neutral.centered=false;const auto before=out.targets.motor_rpm;
  out=jitter_step(2);assert(out.state==State::Working);
  for(unsigned i=0;i<4;++i)assert(std::fabs(out.targets.motor_rpm[i]-before[i])<=config::kNormalRampRpmPerSecond*.002f+.001f);
  const auto before_switch=out.targets.motor_rpm;
  ready.readiness.imu_ready=true;
  for(auto mode : {Mode::Follow,Mode::Gyro,Mode::Depart,Mode::Gyro,Mode::Follow}) {
    neutral.mode=mode;out=jitter_step(1);
    assert(out.state==State::Working && out.ready_count==2000);
  }
  assert(std::fabs(out.targets.motor_rpm[0]-before_switch[0])<=config::kNormalRampRpmPerSecond*.005f+.01f);
  ready.upper_mode_ready=false;ready.readiness.imu_ready=false;
  for(unsigned n=0;n<20;++n) {out=jitter_step(1);assert(out.state==State::Working);assert(out.targets.motor_rpm[0]!=0);}
  ready.readiness.imu_ready=true;ready.upper_mode_ready=true;out=jitter_step(1);
  assert(out.state==State::Working && out.targets.motor_rpm[0]!=0);
  neutral.mode=Mode::Depart;
  for(unsigned n=0;n<1500;++n)out=jitter_step(1);
  bool increased_current=false;
  for(auto c:out.commands) {assert(std::abs(int(c))<=config::kNormalCurrentLimitRaw);increased_current|=std::abs(int(c))>wheel_config::kCurrentLimitRaw;}
  assert(increased_current); // verifies the internal PID cap was raised too
  neutral.mode=Mode::Gyro;out=jitter_step(1);
  for(auto c:out.commands)assert(std::abs(int(c))<=config::kGyroCurrentLimitRaw);
  // Gyro must keep its ramped reference through every allowed 11..19 ms
  // interval. Previously the state stayed Working while the planner reset to 0.
  for(unsigned n=0;n<2000;++n)out=jitter_step(1);
  for(unsigned gap : {11u,19u,12u,18u}) {
    out=jitter_step(gap);assert(out.state==State::Working);
    assert(std::fabs(out.target_yaw_rate-config::kGyroYawRate)<.01f);
  }
  // Speeds beyond the old 10000-rpm threshold must only saturate current,
  // not latch either automatic mode off. Keep all health feedback fresh.
  simulated_rpm=12000;ready.relative_yaw=.25f;
  for(auto mode : {Mode::Follow,Mode::Gyro}) {
    neutral.mode=mode;
    for(unsigned n=0;n<2500;++n) {
      out=jitter_step(n%127==0 ? 19u : 1u);
      assert(out.state==State::Working && out.target_yaw_rate>0);
      for(auto value:out.commands)assert(std::abs(int(value))<=config::kNormalCurrentLimitRaw);
    }
  }
  simulated_rpm=0;
  out=jitter_step(8);assert(out.state==State::Working);
  out=jitter_step(config::kMaxControlGapMs+1);assert(out.state==State::Preparing);for(auto value:out.commands)assert(value==0);
  // Local neutral preparation overlaps upper-board preparation. No wheel
  // output is allowed before the upper actually enables, even after 2 seconds.
  FourWheelTest parallel;BoardFeedback waiting;
  waiting.readiness.yaw_ready=true;
  RemoteCommand centered;centered.online=true;centered.emergency_stop=false;centered.centered=true;
  uint32_t time=0;
  auto parallel_step=[&](unsigned gap){
    time+=gap;waiting.updated_ms=time;waiting.chassis_imu={0,0,time,true};
    for(auto& f:wheels)f={0,0,0,30,time,true};
    return parallel.updateIntegrated(centered,wheels,waiting,time,false);
  };
  out=parallel_step(1);
  for(unsigned n=0;n<999;++n)out=parallel_step(2);
  assert(out.state==State::Preparing && out.ready_count==1998);
  out=parallel_step(2);assert(out.ready_count==2000 && out.state==State::Preparing);
  for(auto c:out.commands)assert(c==0);
  waiting.readiness.upper_board_ready=true;out=parallel_step(1);
  assert(out.state==State::Working); // immediate, without a second 2-second wait
  waiting.readiness.upper_board_ready=false;out=parallel_step(1);
  assert(out.state==State::Preparing && out.ready_count==2000); // pause retains completed arming, not permission to output
  for(auto c:out.commands)assert(c==0);
  // Full translation in every direction plus the largest yaw correction
  // must retain one common scaling factor. Reconstruct the requested twist
  // independently from the four rim speeds to check that both are retained.
  for(float cap : {config::kNormalMaxMotorRpm,config::kGyroMaxMotorRpm}) {
    auto k=config::kWheels;k.max_motor_rpm=cap;
    const float conversion=60.f/(2.f*3.14159265358979323846f*k.wheel_radius_m);
    const float lever=(k.wheelbase_m+k.track_width_m)*.5f;
    for(float yaw_budget : {config::kYawRatePid.output_limit,config::kGyroYawRatePid.output_limit}) {
    const auto spin=SolveMecanumX({0,0,yaw_budget},k);
    assert(spin.valid && spin.motor_targets_ready && spin.scale==1.f);
    for(unsigned n=0;n<32;++n) for(float sign : {-1.f,1.f}) {
      const float angle=float(n)*6.283185307f/32.f;
      Velocity goal{config::kMotion.max_speed*std::cos(angle),
                    config::kMotion.max_speed*std::sin(angle),sign*yaw_budget};
      const auto combined=SolveMecanumX(goal,k);
      // The wheel capacity must retain at least 92% of both components
      // even when translation and the configured yaw correction are combined.
      assert(combined.valid && combined.motor_targets_ready && combined.scale>.92f && combined.scale<=1.f);
      for(float rpm:combined.motor_rpm)assert(std::fabs(rpm)<=cap+.01f);
      const auto& w=combined.wheel_rpm;
      assert(std::fabs((w[0]+w[1]+w[2]+w[3])/(4*conversion)-goal.forward*combined.scale)<1e-5f);
      assert(std::fabs((-w[0]+w[1]-w[2]+w[3])/(4*conversion)-goal.left*combined.scale)<1e-5f);
      assert(std::fabs((-w[0]-w[1]+w[2]+w[3])/(4*lever*conversion)-goal.yaw_rate*combined.scale)<1e-5f);
    }
    }
  }
  // Integrated speed telemetry does not latch off; a real thermal fault still stops.
  waiting.readiness.upper_board_ready=true;
  for(unsigned n=0;n<2001;++n) {
    waiting.updated_ms=++time;waiting.chassis_imu={0,0,time,true};
    for(auto& f:wheels)f={0,0,0,30,time,true};
    out=parallel.updateIntegrated(centered,wheels,waiting,time,false);
  }
  assert(out.state==State::Working);
  waiting.updated_ms=++time;waiting.chassis_imu.updated_ms=time;
  for(auto& f:wheels)f.updated_ms=time;
  wheels[0].rpm=int16_t(config::kOverspeedMotorRpm+1);
  out=parallel.updateIntegrated(centered,wheels,waiting,time,false);
  assert(out.state==State::Working && out.commands[0]<0); // active speed braking, no overspeed latch
  waiting.updated_ms=++time;waiting.chassis_imu.updated_ms=time;
  for(auto& f:wheels)f.updated_ms=time;
  wheels[0].temperature=71;
  out=parallel.updateIntegrated(centered,wheels,waiting,time,false);
  assert(out.state==State::Fault);for(auto c:out.commands)assert(c==0);
  std::cout<<"Chassis IMU Euler rates, feedforward anti-windup, braking direction and readiness passed\n";
}
