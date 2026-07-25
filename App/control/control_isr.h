#ifndef __CONTROL_ISR_H__
#define __CONTROL_ISR_H__

#include "stm32f4xx.h"
#include "contract.h"

extern TIM_HandleTypeDef g_tim6_handle;

extern JointStatus_t g_leg_status[2];
extern JointStatus_t g_arm_status[4];

void control_isr_init(void);
void control_isr_process(void);

#endif
