#ifndef __CONTROL_ISR_H__
#define __CONTROL_ISR_H__

#include "stm32f4xx.h"
#include "contract.h"

#define CAN_TX_FIFO_SIZE    16

typedef struct {
    uint8_t  bus;        /* 0 = CAN1, 1 = CAN2 */
    uint8_t  motor_id;
    int32_t  target;
    uint8_t  mode;
} CanTxEntry_t;

extern JointStatus_t g_leg_status[2];
extern JointStatus_t g_arm_status[4];
extern ABOState_t    g_abo_state[6];          /* v1.7: 工业负载估计器需要访问 */
extern int32_t       g_abo_assist_torque[6];  /* v1.7: ZERO 模式清零需要 */
extern volatile float g_gait_phase;           /* AO 步态相位 0.0~1.0, control_isr.c 定义 */

void control_isr_init(void);
void control_isr_process(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

uint8_t can_tx_fifo_write(CanTxEntry_t *entry);
uint8_t can_tx_fifo_read(CanTxEntry_t *entry);
void can_tx_task_drain(void);

/* ===== STANDALONE 模式本地控制接口 ===== */
void    local_set_mode(uint8_t mode);       /* 0=零扭矩, 1=步态, 2=鳌臂助力 */
uint8_t local_get_mode(void);
void    local_gait_start_stop(uint8_t start); /* 1=开始, 0=停止 */
uint8_t local_gait_is_running(void);
void    local_gait_reset_phase(void);
int32_t local_get_left_hip(void);
int32_t local_get_right_hip(void);

/* ====== ABO Observer 参数接口 (RK3506 调参用) ====== */
uint8_t abo_set_param(uint8_t joint_idx, uint16_t assist_gain_q10,
                       uint16_t hpf_alpha_q16, uint16_t bias_leak_q16,
                       uint8_t enable);
uint8_t abo_get_param(uint8_t joint_idx, ABOState_t *out);
uint8_t abo_set_bias(uint8_t joint_idx, int32_t bias_est);   /* EEPROM 恢复用 */
void    abo_state_reset_all(void);  /* v1.6.2: 模式切换时重置 ABO */

#endif
