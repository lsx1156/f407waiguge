#include "control_isr.h"
#include "safety.h"
#include "bsp_config.h"
#include "can_motor.h"
#include "interpolation.h"
#include "udp_protocol.h"

TIM_HandleTypeDef g_tim6_handle;
ADC_HandleTypeDef g_adc1_handle;

JointStatus_t g_leg_status[2] = {0};
JointStatus_t g_arm_status[4] = {0};

static uint8_t g_arm_div = 0;
static uint8_t g_adc_state = 0;

#define CAN_TIMEOUT_MS      100
static uint32_t g_can1_timeout_cnt = 0;
static uint32_t g_can2_timeout_cnt = 0;

static void read_adc_channels(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    if (g_adc_state == 0) {
        sConfig.Channel = ADC_WEIGHT_CHANNEL;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        HAL_ADC_ConfigChannel(&g_adc1_handle, &sConfig);
        HAL_ADC_Start(&g_adc1_handle);
        g_adc_state = 1;
    } else if (g_adc_state == 1) {
        if (__HAL_ADC_GET_FLAG(&g_adc1_handle, ADC_FLAG_EOC) != RESET) {
            g_weight_adc_value = (uint16_t)HAL_ADC_GetValue(&g_adc1_handle);
            __HAL_ADC_CLEAR_FLAG(&g_adc1_handle, ADC_FLAG_EOC);
            
            sConfig.Channel = ADC_VOLTAGE_CHANNEL;
            sConfig.Rank = 1;
            sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
            HAL_ADC_ConfigChannel(&g_adc1_handle, &sConfig);
            HAL_ADC_Start(&g_adc1_handle);
            g_adc_state = 2;
        }
    } else if (g_adc_state == 2) {
        if (__HAL_ADC_GET_FLAG(&g_adc1_handle, ADC_FLAG_EOC) != RESET) {
            g_voltage_adc_value = (uint16_t)HAL_ADC_GetValue(&g_adc1_handle);
            __HAL_ADC_CLEAR_FLAG(&g_adc1_handle, ADC_FLAG_EOC);
            g_adc_state = 0;
        }
    }
}

void control_isr_init(void)
{
    g_adc1_handle.Instance = ADC1;
    g_adc1_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_adc1_handle.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc1_handle.Init.ScanConvMode = DISABLE;
    g_adc1_handle.Init.ContinuousConvMode = DISABLE;
    g_adc1_handle.Init.DiscontinuousConvMode = DISABLE;
    g_adc1_handle.Init.NbrOfDiscConversion = 0;
    g_adc1_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc1_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_adc1_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc1_handle.Init.NbrOfConversion = 1;
    g_adc1_handle.Init.DMAContinuousRequests = DISABLE;
    g_adc1_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&g_adc1_handle);

    g_tim6_handle.Instance = TIM6;
    g_tim6_handle.Init.Prescaler = 84 - 1;
    g_tim6_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_tim6_handle.Init.Period = 1000 - 1;
    g_tim6_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_tim6_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&g_tim6_handle);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&g_tim6_handle);
}

void control_isr_process(void)
{
    read_adc_channels();

    safety_check_estop();
    
    if (!safety_is_system_safe()) {
        safety_handle_fault();
        HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
        return;
    }
    
    /* Read all pending CAN messages and route by CAN ID */
    {
        JointStatus_t tmp;
        int can1_count = 0, can2_count = 0;
        /* Drain CAN1 FIFO (max 2 messages per cycle for 2 motors) */
        while (can1_count < 2) {
            can_motor_receive_status(&g_can1_handle, &tmp);
            if (tmp.joint_id == 0) break;  /* No more messages */
            if (tmp.joint_id == MOTOR_LEG_0_ID) {
                g_leg_status[0] = tmp;
            } else if (tmp.joint_id == MOTOR_LEG_1_ID) {
                g_leg_status[1] = tmp;
            }
            can1_count++;
        }
        /* Drain CAN2 FIFO (max 4 messages per cycle for 4 motors) */
        while (can2_count < 4) {
            can_motor_receive_status(&g_can2_handle, &tmp);
            if (tmp.joint_id == 0) break;
            int idx = tmp.joint_id - MOTOR_ARM_0_ID;
            if (idx >= 0 && idx < 4) {
                g_arm_status[idx] = tmp;
            }
            can2_count++;
        }

        /* CAN timeout detection (1ms per tick) */
        if (can1_count > 0) {
            g_can1_timeout_cnt = 0;
            g_safety_state.fault_code &= ~FAULT_CAN1_TIMEOUT;
        } else {
            if (g_can1_timeout_cnt < CAN_TIMEOUT_MS) {
                g_can1_timeout_cnt++;
            } else {
                g_safety_state.fault_code |= FAULT_CAN1_TIMEOUT;
            }
        }

        if (can2_count > 0) {
            g_can2_timeout_cnt = 0;
            g_safety_state.fault_code &= ~FAULT_CAN2_TIMEOUT;
        } else {
            if (g_can2_timeout_cnt < CAN_TIMEOUT_MS) {
                g_can2_timeout_cnt++;
            } else {
                g_safety_state.fault_code |= FAULT_CAN2_TIMEOUT;
            }
        }
    }
    
    safety_check_motor_limits(g_leg_status, g_arm_status);
    safety_check_voltage(g_voltage_adc_value);
    safety_check_weight_sensor(g_weight_adc_value);
    
    if (!safety_is_system_safe()) {
        safety_handle_fault();
        HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
        return;
    }
    
    for (int i = 0; i < 2; i++) {
        __disable_irq();
        int32_t target = g_active_command[i].syn_target;
        uint8_t mode = g_active_command[i].control_mode;
        uint16_t rate_limit = g_active_command[i].torque_rate_limit;
        int32_t max_vel = g_active_command[i].max_velocity;
        uint8_t joint_id = g_active_command[i].joint_id;
        __enable_irq();
        
        if (mode == CTRL_MODE_TORQUE) {
            int32_t damp = lookup_damping(g_leg_status[i].velocity);
            int32_t fric = lookup_friction(g_leg_status[i].velocity);
            target += damp + fric;
            
            target = torque_rate_limit(g_leg_status[i].torque, target, 
                                      rate_limit, CONTROL_PERIOD_LEG);
        } else if (mode == CTRL_MODE_POSITION) {
            target = trapezoidal_interpolate(g_leg_status[i].position, target, 
                                             max_vel, CONTROL_PERIOD_LEG);
        }
        
        can_motor_send_command(&g_can1_handle, joint_id, target, mode);
    }
    
    if (++g_arm_div >= CONTROL_PERIOD_ARM) {
        g_arm_div = 0;
        
        for (int i = 0; i < 4; i++) {
            __disable_irq();
            int32_t target = g_active_command[i+2].syn_target;
            uint8_t mode = g_active_command[i+2].control_mode;
            uint16_t rate_limit = g_active_command[i+2].torque_rate_limit;
            int32_t max_vel = g_active_command[i+2].max_velocity;
            uint8_t joint_id = g_active_command[i+2].joint_id;
            __enable_irq();
            
            if (mode == CTRL_MODE_TORQUE) {
                int32_t damp = lookup_damping(g_arm_status[i].velocity);
                int32_t fric = lookup_friction(g_arm_status[i].velocity);
                target += damp + fric;
                
                target = torque_rate_limit(g_arm_status[i].torque, target, 
                                          rate_limit, CONTROL_PERIOD_ARM);
            } else if (mode == CTRL_MODE_POSITION) {
                target = trapezoidal_interpolate(g_arm_status[i].position, target, 
                                                 max_vel, CONTROL_PERIOD_ARM);
            }
            
            can_motor_send_command(&g_can2_handle, joint_id, target, mode);
        }
    }
    
    ReportFrame_t frame;
    report_frame_build(&frame, g_leg_status, g_arm_status);
    report_fifo_write(&frame);
    
    HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
}

void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_tim6_handle, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&g_tim6_handle, TIM_FLAG_UPDATE);
        control_isr_process();
    }
}
