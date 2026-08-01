#include "safety.h"
#include "can_motor.h"

volatile SafetyState_t g_safety_state = {0};

static JointStatus_t g_prev_leg[2] = {0};
static JointStatus_t g_prev_arm[4] = {0};

static void safety_set_fault_level(void)
{
    if (g_safety_state.fault_code & FAULT_ESTOP) {
        g_safety_state.fault_level = FAULT_LEVEL_HW_ESTOP;
    } else if (g_safety_state.fault_code & (FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT)) {
        g_safety_state.fault_level = FAULT_LEVEL_LATCH;
    } else if (g_safety_state.fault_code & (FAULT_POSITION_JUMP | FAULT_VELOCITY_LIMIT | FAULT_TORQUE_LIMIT)) {
        g_safety_state.fault_level = FAULT_LEVEL_WARN;
    } else {
        g_safety_state.fault_level = FAULT_LEVEL_NONE;
    }
}

static void safety_pull_en_low(void)
{
    HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_LEG_R_PORT, EN_LEG_R_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_1_PORT, EN_ARM_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_2_PORT, EN_ARM_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_3_PORT, EN_ARM_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_4_PORT, EN_ARM_4_PIN, GPIO_PIN_RESET);
}

static void safety_set_en_high(void)
{
    HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_LEG_R_PORT, EN_LEG_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_ARM_1_PORT, EN_ARM_1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_ARM_2_PORT, EN_ARM_2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_ARM_3_PORT, EN_ARM_3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_ARM_4_PORT, EN_ARM_4_PIN, GPIO_PIN_SET);
}

void safety_init(void)
{
    g_safety_state.fault_code = FAULT_NONE;
    g_safety_state.estop_active = 0;
    g_safety_state.system_enabled = 0;
    g_safety_state.recovery_counter = 0;
    g_safety_state.fault_level = FAULT_LEVEL_NONE;

    safety_pull_en_low();
}

void safety_check_estop(void)
{
    if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET) {
        g_safety_state.estop_active = 1;
        g_safety_state.fault_code |= FAULT_ESTOP;
    } else {
        g_safety_state.estop_active = 0;
        g_safety_state.fault_code &= ~FAULT_ESTOP;
    }
}

void safety_check_motor_limits(JointStatus_t *leg, JointStatus_t *arm)
{
    uint8_t position_jump = 0;
    uint8_t velocity_fault = 0;
    uint8_t torque_fault = 0;

    for (int i = 0; i < 2; i++) {
        int32_t pos_diff = leg[i].position - g_prev_leg[i].position;
        if (pos_diff < 0) pos_diff = -pos_diff;
        if (pos_diff > SAFETY_POSITION_JUMP) {
            position_jump = 1;
        }

        int32_t abs_vel = leg[i].velocity;
        if (abs_vel < 0) abs_vel = -abs_vel;
        if (abs_vel > SAFETY_VELOCITY_LIMIT) {
            velocity_fault = 1;
        }

        int32_t abs_torque = leg[i].torque;
        if (abs_torque < 0) abs_torque = -abs_torque;
        if (abs_torque > SAFETY_TORQUE_LIMIT) {
            torque_fault = 1;
        }

        g_prev_leg[i] = leg[i];
    }

    for (int i = 0; i < 4; i++) {
        int32_t pos_diff = arm[i].position - g_prev_arm[i].position;
        if (pos_diff < 0) pos_diff = -pos_diff;
        if (pos_diff > SAFETY_POSITION_JUMP) {
            position_jump = 1;
        }

        int32_t abs_vel = arm[i].velocity;
        if (abs_vel < 0) abs_vel = -abs_vel;
        if (abs_vel > SAFETY_VELOCITY_LIMIT) {
            velocity_fault = 1;
        }

        int32_t abs_torque = arm[i].torque;
        if (abs_torque < 0) abs_torque = -abs_torque;
        if (abs_torque > SAFETY_TORQUE_LIMIT) {
            torque_fault = 1;
        }

        g_prev_arm[i] = arm[i];
    }

    if (position_jump) {
        g_safety_state.fault_code |= FAULT_POSITION_JUMP;
    } else {
        g_safety_state.fault_code &= ~FAULT_POSITION_JUMP;
    }
    if (velocity_fault) {
        g_safety_state.fault_code |= FAULT_VELOCITY_LIMIT;
    } else {
        g_safety_state.fault_code &= ~FAULT_VELOCITY_LIMIT;
    }
    if (torque_fault) {
        g_safety_state.fault_code |= FAULT_TORQUE_LIMIT;
    } else {
        g_safety_state.fault_code &= ~FAULT_TORQUE_LIMIT;
    }

    safety_set_fault_level();
}

void safety_handle_fault(void)
{
    if (g_safety_state.fault_code != FAULT_NONE) {
        can_motor_disable_all();
        safety_pull_en_low();
        g_safety_state.system_enabled = 0;
    }
}

void safety_task_run(void)
{
    uint16_t faults = g_safety_state.fault_code;

    if (faults != FAULT_NONE) {
        if (faults & FAULT_ESTOP) {
            can_motor_disable_all();
            safety_pull_en_low();
            g_safety_state.system_enabled = 0;
            g_safety_state.recovery_counter = 0;
        }

        if (faults & (FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT)) {
            can_motor_disable_all();
            safety_pull_en_low();
            g_safety_state.system_enabled = 0;
            g_safety_state.recovery_counter = 0;
        }

        if (faults & (FAULT_POSITION_JUMP | FAULT_VELOCITY_LIMIT | FAULT_TORQUE_LIMIT)) {
            safety_pull_en_low();
            g_safety_state.system_enabled = 0;
        }
    } else {
        if (g_safety_state.system_enabled == 0) {
            if (g_safety_state.recovery_counter < SAFETY_RECOVERY_OK_CYCLES) {
                g_safety_state.recovery_counter++;
            } else {
                safety_set_en_high();
                g_safety_state.system_enabled = 1;
                g_safety_state.recovery_counter = 0;
            }
        }
    }
}

uint8_t safety_is_system_safe(void)
{
    return (g_safety_state.fault_code == FAULT_NONE && !g_safety_state.estop_active);
}

uint16_t safety_get_faults(void)
{
    return g_safety_state.fault_code;
}

void safety_request_enable(void)
{
    g_safety_state.recovery_counter = SAFETY_RECOVERY_OK_CYCLES;
}
