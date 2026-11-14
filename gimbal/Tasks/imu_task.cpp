/**
*******************************************************************************
 * @file      :imu_task.cpp
* @brief     : IMU数据处理任务
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      2024-09-08      Jinletian       1. Create this file.
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "imu_task.hpp"

#include "imu.hpp"
#include "mahony.hpp"
#include "spi.h"
#include "gyro_bias.hpp"
#include "imu_config.hpp"
#include <cmath>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::imu::BMI088 *bmi088_ptr = nullptr;
float acc_data[3], gyro_data[3], temp;
float quat[4];
namespace {
gimbal::GyroBias gyro_bias{gimbal::imu_config::kMaxSampleGapMs};
hello_world::ahrs::Mahony processing_filter(1000.0f, 0.5f, 0.0f);
uint32_t last_sample_ms = 0;
bool processing_ready = false, have_sample = false, hardware_ready = false;
float corrected_gyro[3]{}, continuous_yaw = 0, previous_yaw = 0;
bool yaw_started = false;
}
/* External variables --------------------------------------------------------*/

float euler_angles[3];
volatile uint32_t g_dbg_imu_init_error=0,g_dbg_imu_samples=0,g_dbg_imu_calibrated=0;
volatile uint32_t g_dbg_imu_gap_ms=0,g_dbg_imu_reset_count=0;
volatile float g_dbg_imu_yaw_deg=0,g_dbg_imu_pitch_deg=0,g_dbg_imu_roll_deg=0;
/* Private function prototypes -----------------------------------------------*/

static void Quat2Euler(float *quat, float *euler);

/**
 * @brief       IMU 初始化
 * @retval       None
 * @note        使用 IMU 前必须调用此函数进行初始化，并且注意该函数会阻塞直到
 * IMU 初始化完成
 */
void ImuInit() {
  g_dbg_imu_reset_count=0;
  ImuResetProcessing();
  hardware_ready = false;
  hello_world::imu::BMI088HWConfig default_params = {
      .hspi = &hspi1,
      .acc_cs_port = GPIOA,
      .acc_cs_pin = GPIO_PIN_4,
      .gyro_cs_port = GPIOB,
      .gyro_cs_pin = GPIO_PIN_0,
  };
  auto rot_mat_flatten=gimbal::imu_config::kSensorToBody;

  if (!bmi088_ptr) bmi088_ptr = new hello_world::imu::BMI088(default_params, rot_mat_flatten.data());
  // Skip the driver's unbounded gyro self-test wait. Register-ID/config checks
  // and the independent 2000-sample stationary calibration remain active.
  const auto error=bmi088_ptr->imuInit(false);
  g_dbg_imu_init_error=uint32_t(error);
  hardware_ready = error == hello_world::imu::kBMI088ErrStateNoErr;
}

/**
 * @brief       IMU 更新
 * @retval       None
 * @note
 */
void ImuUpdate() {
  if (!hardware_ready || !bmi088_ptr) return;
  bmi088_ptr->getData(acc_data, gyro_data, &temp);
  ImuProcessSample(acc_data, gyro_data, HAL_GetTick());
}

void ImuResetProcessing() {
  g_dbg_imu_reset_count=g_dbg_imu_reset_count+1;
  gyro_bias.reset();
  processing_filter.init(1000.0f, 0.5f, 0.0f);
  processing_ready = have_sample = false;
  g_dbg_imu_samples=0; g_dbg_imu_calibrated=0;
  yaw_started = false; continuous_yaw = previous_yaw = 0;
  for (float& rate : corrected_gyro) rate = 0;
  for (float& angle : euler_angles) angle = 0;
}

bool ImuReady(uint32_t now_ms) {
  return processing_ready && have_sample && uint32_t(now_ms - last_sample_ms) < 20u;
}

bool ImuProcessSample(const float accel[3], const float gyro[3], uint32_t sample_ms) {
  bool finite=accel && gyro;
  if(finite)for(unsigned i=0;i<3;++i)finite=finite && std::isfinite(accel[i]) && std::isfinite(gyro[i]);
  if(!finite) {
    if(!gyro_bias.ready())ImuResetProcessing();
    else processing_ready=false; // reject the sample without discarding the known zero-rate bias
    return false;
  }
  // Count real samples, allowing bounded scheduling jitter without losing bias.
  if (have_sample && sample_ms == last_sample_ms) return processing_ready;
  uint32_t gap=have_sample ? uint32_t(sample_ms-last_sample_ms) : 1u;
  g_dbg_imu_gap_ms=gap;
  if(gyro_bias.ready()) {
    if(gap>=20u)gap=1; // resume from a fresh sample; do not invent rotation over a missing interval
  } else if(have_sample && gap>gimbal::imu_config::kMaxSampleGapMs){ImuResetProcessing();gap=1;}
  have_sample = true; last_sample_ms = sample_ms;
  const bool calibrated=gyro_bias.update(accel, gyro, sample_ms);
  g_dbg_imu_samples=gyro_bias.samples(); g_dbg_imu_calibrated=calibrated;
  if (!calibrated) {
    if (processing_ready) processing_filter.init(1000.0f, 0.5f, 0.0f);
    processing_ready = false; return false;
  }
  float corrected[3];
  for (int i = 0; i < 3; ++i) corrected[i] = gyro[i] - gyro_bias.bias()[i];
  processing_filter.getQuat(quat);
  processing_filter.init(quat,1000.f/float(gap),.5f,0); // preserve attitude, integrate actual sample interval
  const float acc_norm=std::sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
  const float no_gravity_correction[3]{};
  processing_filter.update(std::fabs(acc_norm-9.80665f)<=1.f ? accel : no_gravity_correction, corrected);
  processing_filter.getQuat(quat);
  for (float value : quat) if (!std::isfinite(value)) { ImuResetProcessing(); return false; }
  Quat2Euler(quat, euler_angles);
  g_dbg_imu_yaw_deg=euler_angles[0]*57.2957795f;
  g_dbg_imu_pitch_deg=euler_angles[1]*57.2957795f;
  g_dbg_imu_roll_deg=euler_angles[2]*57.2957795f;
  if (!yaw_started) { continuous_yaw = euler_angles[0]; yaw_started = true; }
  else continuous_yaw += std::remainder(euler_angles[0]-previous_yaw, 6.283185307f);
  previous_yaw = euler_angles[0];
  for (int i=0; i<3; ++i) corrected_gyro[i] = corrected[i];
  processing_ready = true;
  return true;
}

bool ImuGetFeedback(uint32_t now_ms, gimbal::ImuFeedback& result) {
  result = {};
  if (!ImuReady(now_ms) || !yaw_started) return false;
  const float roll = euler_angles[2], pitch = euler_angles[1];
  const float cp = std::cos(pitch);
  if (!std::isfinite(cp) || std::fabs(cp) < .1f) return false;
  // Convert BODY gyro rates to Euler rates; yaw_rate is not simply gyro Z.
  result.yaw = continuous_yaw; result.pitch = pitch;
  result.pitch_rate = corrected_gyro[1]*std::cos(roll)-corrected_gyro[2]*std::sin(roll);
  result.yaw_rate = (corrected_gyro[1]*std::sin(roll)+corrected_gyro[2]*std::cos(roll))/cp;
  result.yaw*=gimbal::imu_config::kYawSign;
  result.yaw_rate*=gimbal::imu_config::kYawSign;
  result.pitch=(result.pitch-gimbal::imu_config::kPitchHorizontalImuRad)*gimbal::imu_config::kPitchMotorPerImu;
  result.pitch_rate*=gimbal::imu_config::kPitchMotorPerImu;
  result.ready = std::isfinite(result.yaw) && std::isfinite(result.pitch) &&
      std::isfinite(result.pitch_rate) && std::isfinite(result.yaw_rate);
  return result.ready;
}

/**
 * @brief       四元数转欧拉角
 * @param        q: 四元数，[w, x, y, z]
 * @param        euler: 欧拉角，[yaw, pitch, roll]，单位：rad
 * @retval       None
 * @note        涉及四元数转欧拉角的部分请调用该函数
 */
static void Quat2Euler(float *quat, float *euler) {
  arm_atan2_f32(quat[0] * quat[1] + quat[2] * quat[3],
                quat[0] * quat[0] + quat[3] * quat[3] - 0.5f, euler + 2);
  float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (sinp > 1.0f)
    sinp = 1.0;
  else if (sinp < -1.0f)
    sinp = -1.0f;
  euler[1] = asinf(sinp);
  arm_atan2_f32(quat[0] * quat[3] + quat[1] * quat[2],
                quat[0] * quat[0] + quat[1] * quat[1] - 0.5f, euler + 0);
}
