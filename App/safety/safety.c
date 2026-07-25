#include "safety.h"

volatile SafetyState_t g_safety_state = {0};

static JointStatus_t g_prev_leg[2] = {0};
static JointStatus_t g_prev_arm[4] = {0};

void safety_init(void)
{
    g_safety_state.fault_code = FAULT_NONE;
    g_safety_state.estop_active = 0;
    g_safety_state.system_enabled = 0;
    
    HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_LEG_R_PORT, EN_LEG_R_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_1_PORT, EN_ARM_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_2_PORT, EN_ARM_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_3_PORT, EN_ARM_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_4_PORT, EN_ARM_4_PIN, GPIO_PIN_RESET);
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

    /* Set or clear fault flags based on current check results */
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
}

void safety_check_voltage(uint16_t voltage_adc)
{
    if (voltage_adc < SAFETY_VOLTAGE_LOW_THRESHOLD) {
        g_safety_state.fault_code |= FAULT_VOLTAGE_LOW;
    } else {
        g_safety_state.fault_code &= ~FAULT_VOLTAGE_LOW;
    }
}

void safety_check_weight_sensor(uint16_t weight_adc)
{
    if (weight_adc < 100 || weight_adc > 4000) {
        g_safety_state.fault_code |= FAULT_WEIGHT_OPEN;
    } else {
        g_safety_state.fault_code &= ~FAULT_WEIGHT_OPEN;
    }
}

void safety_handle_fault(void)
{
    if (g_safety_state.fault_code != FAULT_NONE) {
        HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_LEG_R_PORT, EN_LEG_R_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_ARM_1_PORT, EN_ARM_1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_ARM_2_PORT, EN_ARM_2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_ARM_3_PORT, EN_ARM_3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_ARM_4_PORT, EN_ARM_4_PIN, GPIO_PIN_RESET);
        g_safety_state.system_enabled = 0;
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
