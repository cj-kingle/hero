#ifndef DM4340_DRV_HPP_
#define DM4340_DRV_HPP_
#include "system_user.hpp"

#define DM4340_MIT_MODE 0x000
#define DM4340_POS_MODE 0x100
#define DM4340_SPEED_MODE 0x200

// 映射范围沿用原驱动，须与电机的 PMAX、VMAX、TMAX 配置一致。
#define DM4340_P_MIN -3.141593f
#define DM4340_P_MAX 3.141593f
#define DM4340_V_MIN -10.0f
#define DM4340_V_MAX 10.0f
#define DM4340_KP_MIN 0.0f
#define DM4340_KP_MAX 500.0f
#define DM4340_KD_MIN 0.0f
#define DM4340_KD_MAX 5.0f
#define DM4340_T_MIN -28.0f
#define DM4340_T_MAX 28.0f

struct DM4340_MotorFbPara_t {
  uint16_t id;
  uint16_t state;
  int p_int;
  int v_int;
  int t_int;

  float pos;
  float vel;
  float tor;

  float Tmos;
  float Tcoil;
};

struct DM4340_Motor_t {
  uint16_t mode;
  DM4340_MotorFbPara_t para;
};

extern void dm4340_fbdata(DM4340_Motor_t *motor, uint8_t *rx_data,
                          uint32_t data_len);

extern void dm4340_enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              uint16_t mode_id);
extern void dm4340_disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                               uint16_t mode_id);

// 关节电机
extern void dm4340_mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos,
                     float vel, float kp, float kd, float torq);
extern void dm4340_pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                           float pos, float vel);
extern void dm4340_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                       float _vel);

extern void dm4340_joint_motor_init(DM4340_Motor_t *motor, uint16_t id, uint16_t mode);

#endif /* DM4340_DRV_HPP_ */
