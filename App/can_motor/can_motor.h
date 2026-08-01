#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"
#include "contract.h"

/* ========== Motor CAN IDs (defined in bsp_config.h) ========== */
#ifndef MOTOR_LEG_0_ID
#define MOTOR_LEG_0_ID      0x01
#endif
#ifndef MOTOR_LEG_1_ID
#define MOTOR_LEG_1_ID      0x02
#endif
#ifndef MOTOR_ARM_0_ID
#define MOTOR_ARM_0_ID      0x10
#endif
#ifndef MOTOR_ARM_1_ID
#define MOTOR_ARM_1_ID      0x11
#endif
#ifndef MOTOR_ARM_2_ID
#define MOTOR_ARM_2_ID      0x12
#endif
#ifndef MOTOR_ARM_3_ID
#define MOTOR_ARM_3_ID      0x13
#endif

/* ========== CyberGear MIT Protocol Constants ========== */
/* Position: 0-65535 → -12.5 ~ 12.5 rad */
#define CG_P_MIN            -12.5f
#define CG_P_MAX            12.5f
/* Velocity: 0-65535 → -30 ~ 30 rad/s */
#define CG_V_MIN            -30.0f
#define CG_V_MAX            30.0f
/* Kp: 0-65535 → 0.0 ~ 500.0 */
#define CG_KP_MIN           0.0f
#define CG_KP_MAX           500.0f
/* Kd: 0-65535 → 0.0 ~ 5.0 */
#define CG_KD_MIN           0.0f
#define CG_KD_MAX           5.0f
/* Torque: 0-65535 → -12 ~ 12 N.m */
#define CG_T_MIN            -12.0f
#define CG_T_MAX            12.0f

/* ========== RS01 MIT Protocol Constants ========== */
/* Angle: 0-65535 → -12.57 ~ 12.57 rad */
#define RS01_P_MIN          -12.57f
#define RS01_P_MAX          12.57f
/* Speed: 12-bit 0-4096 → -44 ~ 44 rad/s */
#define RS01_V_MIN          -44.0f
#define RS01_V_MAX          44.0f
#define RS01_V_BITS         12
/* Kp: 12-bit 0-4096 → 0 ~ 500 */
#define RS01_KP_MIN         0.0f
#define RS01_KP_MAX         500.0f
#define RS01_KP_BITS        12
/* Kd: 12-bit 0-4096 → 0 ~ 5 */
#define RS01_KD_MIN         0.0f
#define RS01_KD_MAX         5.0f
#define RS01_KD_BITS        12
/* Torque: 12-bit 0-4096 → -17 ~ 17 N.m */
#define RS01_T_MIN          -17.0f
#define RS01_T_MAX          17.0f
#define RS01_T_BITS         12

/* ========== RS01 MIT Commands ========== */
#define RS01_CMD_ENABLE     0xFC    /* 指令1: 电机使能运行 */
#define RS01_CMD_DISABLE    0xFD    /* 指令2: 电机停止运行 */
#define RS01_CMD_CTRL       0x00    /* 指令3: MIT动态参数 (mode=0 in ID) */
#define RS01_CMD_SET_ZERO   0xFE    /* 指令4: 设置零点 */
#define RS01_CMD_CLEAR_ERR  0xFB    /* 指令5: 清除错误 */

#define CAN_MIT_FRAME_LEN   8

/* ========== API ========== */
void can_motor_init(void);
void can_motor_send_command(CAN_HandleTypeDef *hcan, uint32_t id,
                            int32_t target, uint8_t mode);
void can_motor_receive_status(CAN_HandleTypeDef *hcan, JointStatus_t *status);
void can_motor_disable_all(void);
void can_motor_disable_bus(CAN_HandleTypeDef *hcan);
uint8_t can_set_mode(CAN_HandleTypeDef *hcan, uint32_t mode);

/* RS01 MIT protocol specific */
void rs01_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void rs01_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id);

/* CyberGear current/torque mode (CMD=0x01) - alternative to MIT Kd feedforward */
void cybergear_send_current_cmd(CAN_HandleTypeDef *hcan, uint32_t motor_id, float current_a);

#endif