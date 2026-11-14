#include "dm4310_drv.hpp"

#include "HW_can.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
float Hex_To_Float(uint32_t *Byte, int num) // 十六进制到浮点数
{
  float value = 0;
  if (Byte) std::memcpy(&value, Byte, sizeof(value));
  return value;
}

uint32_t FloatTohex(float HEX) // 浮点数到十六进制转换
{
  uint32_t value; std::memcpy(&value, &HEX, sizeof(value)); return value;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max]
*内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits) {
  if (!std::isfinite(x_float) || !std::isfinite(x_min) ||
      !std::isfinite(x_max) || x_max <= x_min || bits < 1 || bits > 16) return 0;
  return int(std::lround((std::clamp(x_float, x_min, x_max)-x_min) *
      float((1u << bits)-1u) / (x_max-x_min)));
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max]
*内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  if (!std::isfinite(x_min) || !std::isfinite(x_max) ||
      x_max <= x_min || bits < 1 || bits > 16) return 0;
  const int maximum = int((1u << bits)-1u);
  return float(std::clamp(x_int, 0, maximum)) * (x_max-x_min) / maximum + x_min;
}

void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode) {
  if (!motor) return;
  *motor = {};
  motor->mode = mode;
  motor->para.id = id;
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (!motor) return;
  damiao::Feedback fb;
  if (!damiao::DecodeFeedback(motor->limits, rx_data, data_len, fb)) return;
  motor->para.id = fb.id; motor->para.state = fb.state;
  motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
  motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
  motor->para.t_int = ((rx_data[4] & 15) << 8) | rx_data[5];
  motor->para.pos = fb.position; motor->para.vel = fb.velocity;
  motor->para.tor = fb.torque;
  motor->para.Tmos = fb.mos_temperature; motor->para.Tcoil = fb.rotor_temperature;
}

void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                       uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;

  CAN_Send_Msg(hcan, data, id, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                        uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;

  CAN_Send_Msg(hcan, data, id, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:
*指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,
              float kp, float kd, float torq) {
  damiao::Payload data;
  if (!damiao::PackMit({P_MAX, V_MAX, T_MAX}, pos, vel, kp, kd, torq, data)) {
    data = damiao::DisableFrame();
  }
  CAN_TrySend(hcan, data.data(), motor_id + MIT_MODE, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:
*指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos,
                    float vel) {
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  if (!std::isfinite(pos) || !std::isfinite(vel)) { disable_motor_mode(hcan, motor_id, POS_MODE); return; }
  id = motor_id + POS_MODE;
  pbuf = (uint8_t *)&pos;
  vbuf = (uint8_t *)&vel;

  data[0] = *pbuf;
  data[1] = *(pbuf + 1);
  data[2] = *(pbuf + 2);
  data[3] = *(pbuf + 3);

  data[4] = *vbuf;
  data[5] = *(vbuf + 1);
  data[6] = *(vbuf + 2);
  data[7] = *(vbuf + 3);

  CAN_Send_Msg(hcan, data, id, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel) {
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  if (!std::isfinite(vel)) { disable_motor_mode(hcan, motor_id, SPEED_MODE); return; }
  id = motor_id + SPEED_MODE;
  vbuf = (uint8_t *)&vel;

  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  CAN_Send_Msg(hcan, data, id, 4);
}
