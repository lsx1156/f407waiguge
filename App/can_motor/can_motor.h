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
/* Position: 0-65535 → -4π ~ 4π rad (≈ -12.566 ~ 12.566 rad), 参考 can.txt L92 */
#define CG_P_MIN            (-3.14159265358979f * 4.0f)
#define CG_P_MAX            ( 3.14159265358979f * 4.0f)
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

/* ========== RS01 MIT Protocol Constants (标准帧, 11-bit ID, 半字节压缩) ========== */
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

/* RS01/L91 扩展帧命令类型 (出厂默认私有协议, 29-bit ExtId, 与 CyberGear 一致)
 * ExtId = (mode << 24) | (data << 8) | motor_id
 *   mode:     bit 28-24 (5bit)  — 通信类型 (1=MIT运控, 2=反馈, 3=使能, 4=停止, 6=零位)
 *   data:     bit 23-8  (16bit) — MIT运控时=torque_uint16, 其他命令=MasterID
 *   motor_id: bit 7-0   (8bit)  — 目标电机 CAN ID
 *
 * ★ v1.6.8fix RS01 通信协议 (与 CyberGear 完全一致):
 *   1. MIT运控: ExtId data=torque_uint16, Data=[pos_16][vel_16][Kp_16][Kd_16]
 *      TORQUE模式:  ExtId data=torque, Data=全0 (Kp=Kd=0 → 纯力矩控制)
 *      POSITION模式: ExtId data=0x7FFF(零FF), Data=[pos][vel][Kp][Kd]
 *   2. Data 格式: 16-bit 字节对齐 (非 12-bit 压缩 — 12-bit 导致 Kp 误读为满量程)
 *   3. 反馈帧: ExtId bit23-22=mode_status, bit21-16=fault_flags, bit15-8=motor_id
 *             Data[0-5]=pos/vel/torque (各16-bit), Data[6-7]=温度(int16, 0.1°C)
 *   4. 必须发 0x18 (自动上报) 才有反馈帧, 否则电机不主动上报
 *
 * ★ RS01 出厂默认 = 私有协议 = 29-bit 扩展帧 (说明书 P27~P30)
 *   仅切换到 MIT 协议后 (指令8/类型25, F_CMD=2, 需重启) 才用 11-bit 标准帧
 *   当前使用默认私有协议, 保持扩展帧 */
#define RS01_CMD_MIT_CTRL   1   /* MIT 运控模式 (torque 或 pos/vel/kp/kd 在 Data 中) */
#define RS01_CMD_FEEDBACK   2   /* 电机反馈 (接收用) */
#define RS01_CMD_ENABLE     3   /* 电机使能 */
#define RS01_CMD_STOP       4   /* 电机停止/清故障 */
#define RS01_CMD_SET_ZERO   6   /* 设置机械零位 */
#define RS01_CMD_WRITE_PARAM 0x12  /* 参数写入 (float 小端) */
#define RS01_CMD_AUTO_REPORT 0x18  /* 使能自动上报 (无此命令电机不发反馈) */
#define RS01_MASTER_ID      0x0001  /* 主控 ID (16bit) */

/* ========== CyberGear Communication Types (Extended Frame) ========== */
#define CG_CMD_GET_ID       0   /* 获取设备ID */
#define CG_CMD_MIT_CTRL     1   /* 运控模式控制 (MIT 5参数) */
#define CG_CMD_FEEDBACK     2   /* 电机反馈 */
#define CG_CMD_ENABLE       3   /* 电机使能 */
#define CG_CMD_STOP         4   /* 电机停止/清故障 */
#define CG_CMD_SET_ZERO     6   /* 设置机械零位 */
#define CG_CMD_SET_ID       7   /* 设置电机 CAN_ID */
#define CG_CMD_READ_PARAM   17  /* 单个参数读取 */
#define CG_CMD_WRITE_PARAM  18  /* 单个参数写入 */
#define CG_CMD_FAULT        21  /* 故障反馈 */
#define CG_CMD_SET_BAUD     22  /* 波特率修改 */

#define CG_MASTER_ID        0   /* 主机 CAN ID */

#define CAN_MIT_FRAME_LEN   8

/* ========== API ========== */
void can_motor_init(void);
uint8_t can_bus_detect(CAN_HandleTypeDef *hcan, uint32_t test_id);
void can_motor_send_command(CAN_HandleTypeDef *hcan, uint32_t id,
                            int32_t target, uint8_t mode);
void can_motor_receive_status(CAN_HandleTypeDef *hcan, JointStatus_t *status);
void can_motor_disable_all(void);
void can_motor_disable_bus(CAN_HandleTypeDef *hcan);
uint8_t can_set_mode(CAN_HandleTypeDef *hcan, uint32_t mode);

/* Motor online status tracking */
#define MAX_MOTORS          8
#define MOTOR_TIMEOUT_MS    2000    /* no feedback for 2s = offline */

void can_motor_online_tick(void);
uint8_t can_motor_is_online(uint8_t display_id);
void can_motor_set_mapping(uint8_t display_id, CAN_HandleTypeDef *hcan, uint32_t can_id);
JointStatus_t *can_motor_get_status(uint8_t can_id);

/* RS01 MIT protocol specific */
uint8_t rs01_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id);
uint8_t rs01_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id);
uint8_t rs01_send_zero_torque(CAN_HandleTypeDef *hcan, uint32_t motor_id);
uint8_t rs01_mit_set_zero(CAN_HandleTypeDef *hcan, uint32_t motor_id);
uint8_t rs01_write_param(CAN_HandleTypeDef *hcan, uint32_t motor_id, uint16_t index, float value);
uint8_t rs01_enable_auto_report(CAN_HandleTypeDef *hcan, uint32_t motor_id, uint16_t period_ms);
uint8_t rs01_init_one(CAN_HandleTypeDef *hcan, uint32_t motor_id);

/* CyberGear MIT protocol specific */
void cybergear_mit_enable(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void cybergear_mit_disable(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void cybergear_mit_zero_torque(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void cybergear_mit_set_zero(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void cybergear_mit_set_position(CAN_HandleTypeDef *hcan, uint32_t motor_id,
                                 int32_t pos_mdeg, float kp, float kd);
void cybergear_mit_set_torque(CAN_HandleTypeDef *hcan, uint32_t motor_id, int32_t torque_mnm);
void cybergear_read_param(CAN_HandleTypeDef *hcan, uint32_t motor_id, uint16_t index);

/* CyberGear current/torque mode (CMD=0x01) - alternative to MIT Kd feedforward */
void cybergear_send_current_cmd(CAN_HandleTypeDef *hcan, uint32_t motor_id, float current_a);

#endif