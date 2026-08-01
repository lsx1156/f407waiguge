#ifndef __SAFETY_H__
#define __SAFETY_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "bsp_config.h"
#include "contract.h"

#define SAFETY_VELOCITY_LIMIT    30000
#define SAFETY_TORQUE_LIMIT      10000
#define SAFETY_POSITION_JUMP     1000

#define SAFETY_RECOVERY_OK_CYCLES   100

typedef enum {
    FAULT_LEVEL_NONE = 0,
    FAULT_LEVEL_WARN,
    FAULT_LEVEL_LATCH,
    FAULT_LEVEL_HW_ESTOP
} FaultLevel_e;

typedef struct {
    uint16_t fault_code;
    uint8_t  estop_active;
    uint8_t  system_enabled;
    uint8_t  recovery_counter;
    FaultLevel_e fault_level;
} SafetyState_t;

extern volatile SafetyState_t g_safety_state;

void safety_init(void);
void safety_check_estop(void);
void safety_check_motor_limits(JointStatus_t *leg, JointStatus_t *arm);
void safety_handle_fault(void);
void safety_task_run(void);
uint8_t safety_is_system_safe(void);
uint16_t safety_get_faults(void);
void safety_request_enable(void);

#endif
