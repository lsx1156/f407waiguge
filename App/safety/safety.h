#ifndef __SAFETY_H__
#define __SAFETY_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "bsp_config.h"
#include "contract.h"

#define SAFETY_VELOCITY_LIMIT    30000
#define SAFETY_TORQUE_LIMIT      10000
#define SAFETY_POSITION_JUMP     1000
#define SAFETY_VOLTAGE_LOW_THRESHOLD 1800

typedef struct {
    uint16_t fault_code;
    uint8_t  estop_active;
    uint8_t  system_enabled;
} SafetyState_t;

extern volatile SafetyState_t g_safety_state;

void safety_init(void);
void safety_check_estop(void);
void safety_check_motor_limits(JointStatus_t *leg, JointStatus_t *arm);
void safety_check_voltage(uint16_t voltage_adc);
void safety_check_weight_sensor(uint16_t weight_adc);
void safety_handle_fault(void);
uint8_t safety_is_system_safe(void);
uint16_t safety_get_faults(void);

#endif
