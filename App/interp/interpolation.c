#include "interpolation.h"

DampingTable_t g_damping_table = {0};
FrictionTable_t g_friction_table = {0};
PIDParams_t g_pid_params[PID_PARAM_SET_COUNT] = {0};

void interpolation_init(void)
{
    for (int i = 0; i < DAMPING_TABLE_SIZE; i++) {
        g_damping_table.damping[i] = 0;
    }
    for (int i = 0; i < FRICTION_TABLE_SIZE; i++) {
        g_friction_table.friction[i] = 0;
    }
    for (int i = 0; i < PID_PARAM_SET_COUNT; i++) {
        g_pid_params[i].kp = 10;
        g_pid_params[i].kd = 2;
        g_pid_params[i].torque_limit = 10000;
        g_pid_params[i].velocity_limit = 1000;
        g_pid_params[i].dead_zone = 0;
        g_pid_params[i].reserved = 0;
    }
}

int32_t lookup_damping(int32_t velocity)
{
    int32_t abs_vel = (velocity < 0) ? -velocity : velocity;
    uint32_t index = (abs_vel >= DAMPING_TABLE_SIZE) ? (DAMPING_TABLE_SIZE - 1) : abs_vel;
    return g_damping_table.damping[index];
}

int32_t lookup_friction(int32_t velocity)
{
    int32_t abs_vel = (velocity < 0) ? -velocity : velocity;
    uint32_t index = (abs_vel >= FRICTION_TABLE_SIZE) ? (FRICTION_TABLE_SIZE - 1) : abs_vel;
    int32_t friction = g_friction_table.friction[index];
    
    if (velocity > 0) return friction;
    if (velocity < 0) return -friction;
    return 0;
}

int32_t trapezoidal_interpolate(int32_t current, int32_t target, int32_t max_rate, int32_t period_ms)
{
    int32_t diff = target - current;
    int32_t max_step = max_rate * period_ms / 1000;
    
    if (diff > max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}

int32_t torque_rate_limit(int32_t current, int32_t target, int32_t rate_limit, int32_t period_ms)
{
    int32_t diff = target - current;
    int32_t max_step = rate_limit * period_ms / 1000;
    
    if (diff > max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}
