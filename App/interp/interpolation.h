#ifndef __INTERPOLATION_H__
#define __INTERPOLATION_H__

#include "stm32f4xx.h"
#include "bsp_config.h"
#include "contract.h"

typedef struct {
    int32_t vel[DAMPING_TABLE_SIZE];
    int32_t damping[DAMPING_TABLE_SIZE];
} DampingTable_t;

typedef struct {
    int32_t vel[FRICTION_TABLE_SIZE];
    int32_t friction[FRICTION_TABLE_SIZE];
} FrictionTable_t;

extern DampingTable_t g_damping_table;
extern FrictionTable_t g_friction_table;
extern PIDParams_t g_pid_params[PID_PARAM_SET_COUNT];

void interpolation_init(void);
int32_t lookup_damping(int32_t velocity);
int32_t lookup_friction(int32_t velocity);
int32_t trapezoidal_interpolate(int32_t current, int32_t target, int32_t max_rate, int32_t period_ms);
int32_t torque_rate_limit(int32_t current, int32_t target, int32_t rate_limit, int32_t period_ms);

#endif
