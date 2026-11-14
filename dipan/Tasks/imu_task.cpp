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
#include "main.h"
#include "chassis_imu_config.hpp"

#include "imu.hpp"
#include "mahony.hpp"
#include "spi.h"
#include "gyro_bias.hpp"
#include <cmath>
#include <algorithm>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::imu::BMI088 *bmi088_ptr = nullptr;
float acc_data[3], gyro_data[3], temp;
float quat[4];
namespace {
chassis::GyroBias gyro_bias{chassis::imu_config::kMaxSampleGapMs};
hello_world::ahrs::Mahony processing_filter(1000.0f, 0.5f, 0.0f);
uint32_t last_sample_ms = 0;
bool processing_ready = false, have_sample = false, hardware_ready = false;
float continuous_yaw=0,previous_yaw=0,filtered_yaw_rate=0;
bool yaw_started=false;
}
/* External variables --------------------------------------------------------*/

float euler_angles[3];
volatile uint32_t dbg_chassis_imu_init_error=0,dbg_chassis_imu_samples=0,dbg_chassis_imu_calibrated=0;
volatile uint32_t dbg_chassis_imu_mounting_confirmed=chassis::imu_config::kMountingConfirmed;
volatile float dbg_chassis_imu_yaw_deg=0,dbg_chassis_imu_pitch_deg=0,dbg_chassis_imu_roll_deg=0;
volatile float dbg_chassis_imu_raw_yaw_rate=0;
volatile uint32_t dbg_chassis_imu_update_count=0,dbg_chassis_imu_gap_ms=0,dbg_chassis_imu_reset_count=0;
volatile uint32_t dbg_chassis_imu_stationary=0;
volatile float dbg_chassis_imu_acc_norm=0,dbg_chassis_imu_max_gyro=0;
/* Private function prototypes -----------------------------------------------*/

static void Quat2Euler(float *quat, float *euler);

/**
 * @brief       IMU 初始化
 * @retval       None
 * @note        使用 IMU 前必须调用此函数进行初始化，并且注意该函数会阻塞直到
 * IMU 初始化完成
 */
void ImuInit() {
  dbg_chassis_imu_reset_count=0;dbg_chassis_imu_update_count=0;
  ImuResetProcessing();
  hardware_ready = false;
  hello_world::imu::BMI088HWConfig default_params = {
      .hspi = &hspi1,
      .acc_cs_port = GPIOA,
      .acc_cs_pin = GPIO_PIN_4,
      .gyro_cs_port = GPIOB,
      .gyro_cs_pin = GPIO_PIN_0,
  };
  auto rot_mat_flatten=chassis::imu_config::kSensorToBody;

  if (!bmi088_ptr) bmi088_ptr = new hello_world::imu::BMI088(default_params, rot_mat_flatten.data());
  // Avoid the driver's unbounded self-test loop; retain register/config checks.
  const auto error=bmi088_ptr->imuInit(false);
  dbg_chassis_imu_init_error=uint32_t(error);
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
  dbg_chassis_imu_update_count=dbg_chassis_imu_update_count+1;
  ImuProcessSample(acc_data, gyro_data, HAL_GetTick());
}

void ImuResetProcessing() {
  dbg_chassis_imu_reset_count=dbg_chassis_imu_reset_count+1;
  gyro_bias.reset();
  processing_filter.init(1000.0f, 0.5f, 0.0f);
  processing_ready = have_sample = false;
  yaw_started=false;continuous_yaw=previous_yaw=filtered_yaw_rate=0;
  dbg_chassis_imu_samples=0;dbg_chassis_imu_calibrated=0;dbg_chassis_imu_raw_yaw_rate=0;
  dbg_chassis_imu_stationary=0;
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
    else processing_ready=false; // reject invalid data but retain established gyro calibration
    return false;
  }
  // Initial calibration counts distinct stationary samples. Once established,
  // retain the known bias through gaps without inventing missing rotation.
  if (have_sample && sample_ms == last_sample_ms) return processing_ready;
  uint32_t gap=have_sample ? uint32_t(sample_ms-last_sample_ms) : 1u;
  dbg_chassis_imu_gap_ms=gap;
  // Freshness still marks an outage unavailable; the next valid sample may
  // resume with the existing bias while the vehicle is moving.
  const auto allowed_gap=processing_ready ? chassis::imu_config::kMaxCalibratedSampleGapMs :
      chassis::imu_config::kMaxSampleGapMs;
  if(gyro_bias.ready()) {
    if(gap>=20u)gap=1; // do not integrate an unknown outage using the new rate
  } else if(have_sample && gap>allowed_gap){ImuResetProcessing();gap=1;}
  have_sample = true; last_sample_ms = sample_ms;
  dbg_chassis_imu_acc_norm=std::sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
  dbg_chassis_imu_max_gyro=std::max({std::fabs(gyro[0]),std::fabs(gyro[1]),std::fabs(gyro[2])});
  const bool calibrated=gyro_bias.update(accel, gyro, sample_ms);
  dbg_chassis_imu_samples=gyro_bias.samples();dbg_chassis_imu_calibrated=calibrated;
  dbg_chassis_imu_stationary=gyro_bias.stationary();
  if (!calibrated) {
    if (processing_ready) processing_filter.init(1000.0f, 0.5f, 0.0f);
    processing_ready = false; return false;
  }
  float corrected[3];
  for (int i = 0; i < 3; ++i) corrected[i] = gyro[i] - gyro_bias.bias()[i];
  // Reconfigure the sample period while preserving the quaternion. Mahony Ki=0,
  // so its integral reset here does not discard an enabled correction state.
  processing_filter.getQuat(quat);
  processing_filter.init(quat,1000.f/float(gap),.5f,0);
  // Linear/centripetal acceleration is not a gravity direction. During a
  // dynamic acceleration sample integrate gyro only rather than tilting the
  // attitude estimate or throwing away the established zero-rate calibration.
  const float no_gravity_correction[3]{};
  const bool gravity_reliable=std::fabs(dbg_chassis_imu_acc_norm-9.80665f)<=1.0f;
  processing_filter.update(gravity_reliable ? accel : no_gravity_correction, corrected);
  processing_filter.getQuat(quat);
  for (float value : quat) if (!std::isfinite(value)) { ImuResetProcessing(); return false; }
  Quat2Euler(quat, euler_angles);
  for(float angle:euler_angles)if(!std::isfinite(angle)){ImuResetProcessing();return false;}
  float rate=0;
  if(!chassis::EulerYawRate(euler_angles[2],euler_angles[1],corrected,rate)){ImuResetProcessing();return false;}
  rate*=chassis::imu_config::kYawSign;
  dbg_chassis_imu_raw_yaw_rate=rate;
  if(!yaw_started){continuous_yaw=euler_angles[0];filtered_yaw_rate=rate;yaw_started=true;}
  else {
    continuous_yaw+=std::remainder(euler_angles[0]-previous_yaw,6.283185307f);
    const float dt=float(gap)*.001f;
    const float alpha=dt/(chassis::imu_config::kYawRateFilterTau+dt);
    filtered_yaw_rate+=alpha*(rate-filtered_yaw_rate);
  }
  previous_yaw=euler_angles[0];
  dbg_chassis_imu_yaw_deg=continuous_yaw*chassis::imu_config::kYawSign*57.2957795f;
  dbg_chassis_imu_pitch_deg=euler_angles[1]*57.2957795f;
  dbg_chassis_imu_roll_deg=euler_angles[2]*57.2957795f;
  processing_ready = true;
  return true;
}
bool ImuGetFeedback(uint32_t now_ms,chassis::ChassisImuFeedback& result){
  result={};
  if(!ImuReady(now_ms) || !yaw_started)return false;
  result.yaw=continuous_yaw*chassis::imu_config::kYawSign;
  result.yaw_rate=filtered_yaw_rate;result.updated_ms=last_sample_ms;
  result.ready=std::isfinite(result.yaw) && std::isfinite(result.yaw_rate);
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
