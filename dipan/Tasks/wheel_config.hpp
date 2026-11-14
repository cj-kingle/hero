#pragma once
#include "pid.hpp"
#include "chassis_task.hpp"
#include <array>
namespace chassis::wheel_config {
// Shared physical wheel parameters and PID gains. Integrated motion limits
// live in chassis_config.hpp; the low bench limits below serve offline fixtures.
inline constexpr unsigned kObservedMotorId = 4; // Ozone scalar mirrors: 1 LF, 2 LR, 3 RR, 4 RF
inline constexpr float kGearRatio = 14.0f;
inline constexpr float kMaxMotorRpm = 460.0f; // Symmetric +/- rotor rpm, NOT wheel rpm
inline constexpr float kRampRpmPerSecond = 600.0f;
inline constexpr float kOverspeedRpm = 1200.0f; // legacy bench protection; integrated limit is in chassis_config.hpp
// C620 raw current units (16384 = 20 A). Initial bench values, NOT tuned gains.
inline constexpr float kCurrentLimitRaw = 4000.0f; // about 4.88 A per wheel; loaded-start trial
// Keep independent gains for each wheel; none of these initial gains are validated.
inline constexpr std::array<PidConfig, 4> kWheelPids{{
    {12.0f, 2.0f, 0.0f, kCurrentLimitRaw, 300.0f}, // ID1 left front
    {12.0f, 2.0f, 0.0f, kCurrentLimitRaw, 300.0f}, // ID2 left rear
    {12.0f, 2.0f, 0.0f, kCurrentLimitRaw, 300.0f}, // ID3 right rear向后
    {12.0f, 2.0f, 0.0f, kCurrentLimitRaw, 300.0f}, // ID4 right front向后
}};
static_assert(kObservedMotorId >= 1 && kObservedMotorId <= 4);
static_assert(kCurrentLimitRaw > 0 && kCurrentLimitRaw <= 16384);
}
