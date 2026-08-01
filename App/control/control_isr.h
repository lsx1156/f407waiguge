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

void control_isr_init(void);
void control_isr_process(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

uint8_t can_tx_fifo_write(CanTxEntry_t *entry);
uint8_t can_tx_fifo_read(CanTxEntry_t *entry);
void can_tx_task_drain(void);

#endif
