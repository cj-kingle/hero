#include "dm4340_drv.hpp"

#include "HW_can.hpp"

static int dm4340_float_to_uint(float x_float, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

static float dm4340_uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void dm4340_joint_motor_init(DM4340_Motor_t *motor, uint16_t id, uint16_t mode) {
  motor->mode = mode;
  motor->para.id = id;
}

void dm4340_fbdata(DM4340_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == 8) { // 返回的数据有8个字节
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = dm4340_uint_to_float(motor->para.p_int, DM4340_P_MIN, DM4340_P_MAX, 16);
    motor->para.vel = dm4340_uint_to_float(motor->para.v_int, DM4340_V_MIN, DM4340_V_MAX, 12);
    motor->para.tor = dm4340_uint_to_float(motor->para.t_int, DM4340_T_MIN, DM4340_T_MAX, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm4340_enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
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

void dm4340_disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
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

void dm4340_mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos,
              float vel, float kp, float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + DM4340_MIT_MODE;

  // 将浮点数据等比例转换成整数
  pos_tmp = dm4340_float_to_uint(pos, DM4340_P_MIN, DM4340_P_MAX, 16);
  vel_tmp = dm4340_float_to_uint(vel, DM4340_V_MIN, DM4340_V_MAX, 12);
  kp_tmp = dm4340_float_to_uint(kp, DM4340_KP_MIN, DM4340_KP_MAX, 12);
  kd_tmp = dm4340_float_to_uint(kd, DM4340_KD_MIN, DM4340_KD_MAX, 12);
  tor_tmp = dm4340_float_to_uint(torq, DM4340_T_MIN, DM4340_T_MAX, 12);

  data[0] = (pos_tmp >> 8); // data[0]存放pos_tmp高八位
  data[1] = pos_tmp;        // data[1]存放pos_tmp低八位
  data[2] = (vel_tmp >> 4); // data[2]存放vel_tmp11位到4位
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  // 通过can总线 发送到电机驱动器
  CAN_Send_Msg(hcan, data, id, 8);
}

void dm4340_pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos,
                    float vel) {
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  id = motor_id + DM4340_POS_MODE;
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

void dm4340_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel) {
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + DM4340_SPEED_MODE;
  vbuf = (uint8_t *)&vel;

  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  CAN_Send_Msg(hcan, data, id, 4);
}
