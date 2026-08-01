#include "interpolation.h"

DampingTable_t g_damping_table = {0};
FrictionTable_t g_friction_table = {0};
PIDParams_t g_pid_params[PID_PARAM_SET_COUNT] = {0};

static int32_t g_vel_max_mdeg_s = 1000000;

void interpolation_init(void)
{
    for (int i = 0; i < DAMPING_TABLE_SIZE; i++) {
        g_damping_table.vel[i] = (i * g_vel_max_mdeg_s) / (DAMPING_TABLE_SIZE - 1);
        g_damping_table.damping[i] = 0;
    }
    for (int i = 0; i < FRICTION_TABLE_SIZE; i++) {
        g_friction_table.vel[i] = (i * g_vel_max_mdeg_s) / (FRICTION_TABLE_SIZE - 1);
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

static int32_t linear_interpolate(int32_t x, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    if (x1 == x0) return y0;
    int64_t dy = (int64_t)(y1 - y0) * (x - x0);
    return y0 + (int32_t)(dy / (x1 - x0));
}

int32_t lookup_damping(int32_t velocity)
{
    int32_t abs_vel = (velocity < 0) ? -velocity : velocity;

    if (abs_vel >= g_damping_table.vel[DAMPING_TABLE_SIZE - 1]) {
        return g_damping_table.damping[DAMPING_TABLE_SIZE - 1];
    }

    for (int i = 1; i < DAMPING_TABLE_SIZE; i++) {
        if (abs_vel <= g_damping_table.vel[i]) {
            return linear_interpolate(abs_vel,
                                      g_damping_table.vel[i - 1], g_damping_table.damping[i - 1],
                                      g_damping_table.vel[i], g_damping_table.damping[i]);
        }
    }
    return g_damping_table.damping[DAMPING_TABLE_SIZE - 1];
}

int32_t lookup_friction(int32_t velocity)
{
    int32_t abs_vel = (velocity < 0) ? -velocity : velocity;
    int32_t friction_val;

    if (abs_vel >= g_friction_table.vel[FRICTION_TABLE_SIZE - 1]) {
        friction_val = g_friction_table.friction[FRICTION_TABLE_SIZE - 1];
    } else {
        friction_val = 0;
        for (int i = 1; i < FRICTION_TABLE_SIZE; i++) {
            if (abs_vel <= g_friction_table.vel[i]) {
                friction_val = linear_interpolate(abs_vel,
                                                   g_friction_table.vel[i - 1], g_friction_table.friction[i - 1],
                                                   g_friction_table.vel[i], g_friction_table.friction[i]);
                break;
            }
        }
    }

    if (velocity > 0) return friction_val;
    if (velocity < 0) return -friction_val;
    return 0;
}

int32_t trapezoidal_interpolate(int32_t current, int32_t target, int32_t max_rate, int32_t period_ms)
{
    int32_t diff = target - current;
    int32_t max_step = (max_rate * period_ms + 500) / 1000;

    if (diff > max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}

int32_t torque_rate_limit(int32_t current, int32_t target, int32_t rate_limit, int32_t period_ms)
{
    int32_t diff = target - current;
    int32_t max_step = (rate_limit * period_ms + 500) / 1000;

    if (diff > max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}
