#include "control_isr.h"
#include "main.h"
#include "safety.h"
#include "bsp_config.h"
#include "can_motor.h"
#include "interpolation.h"
#include "udp_protocol.h"

static int32_t torque_compensate(int32_t target, int32_t velocity)
{
    int32_t damping = lookup_damping(velocity);
    int32_t friction = lookup_friction(velocity);
    return target + damping + friction;
}

static volatile CanTxEntry_t g_can_tx_fifo[CAN_TX_FIFO_SIZE];
static volatile uint8_t g_can_tx_head = 0;
static volatile uint8_t g_can_tx_tail = 0;

uint8_t can_tx_fifo_write(CanTxEntry_t *entry)
{
    uint8_t next = (g_can_tx_head + 1) % CAN_TX_FIFO_SIZE;
    if (next == g_can_tx_tail) return 0;
    g_can_tx_fifo[g_can_tx_head] = *entry;
    __DMB();
    g_can_tx_head = next;
    return 1;
}

uint8_t can_tx_fifo_read(CanTxEntry_t *entry)
{
    if (g_can_tx_head == g_can_tx_tail) return 0;
    *entry = g_can_tx_fifo[g_can_tx_tail];
    __DMB();
    g_can_tx_tail = (g_can_tx_tail + 1) % CAN_TX_FIFO_SIZE;
    return 1;
}

void can_tx_task_drain(void)
{
    CanTxEntry_t entry;
    while (can_tx_fifo_read(&entry)) {
        CAN_HandleTypeDef *hcan = (entry.bus == 0) ? &hcan1 : &hcan2;
        can_motor_send_command(hcan, entry.motor_id, entry.target, entry.mode);
    }
}

static void push_can_tx(uint8_t bus, uint8_t motor_id, int32_t target, uint8_t mode)
{
    CanTxEntry_t e;
    e.bus = bus;
    e.motor_id = motor_id;
    e.target = target;
    e.mode = mode;
    can_tx_fifo_write(&e);
}

JointStatus_t g_leg_status[2] = {0};
JointStatus_t g_arm_status[4] = {0};

static uint8_t g_arm_div = CONTROL_PERIOD_ARM - 1;

#define CAN_TIMEOUT_MS      100
static uint32_t g_can1_timeout_cnt = 0;
static uint32_t g_can2_timeout_cnt = 0;

void control_isr_init(void)
{
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        control_isr_process();
    }
}

static void safety_isr_fault(void)
{
    HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_LEG_R_PORT, EN_LEG_R_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_1_PORT, EN_ARM_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_2_PORT, EN_ARM_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_3_PORT, EN_ARM_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_4_PORT, EN_ARM_4_PIN, GPIO_PIN_RESET);
    g_safety_state.system_enabled = 0;
}

void control_isr_process(void)
{
    HAL_IWDG_Refresh(&hiwdg);

    safety_check_estop();

    if (!safety_is_system_safe()) {
        safety_isr_fault();
        HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
        return;
    }

    {
        JointStatus_t tmp;
        int can1_count = 0, can2_count = 0;
        while (can1_count < 2) {
            can_motor_receive_status(&hcan1, &tmp);
            if (tmp.joint_id == 0) break;
            if (tmp.joint_id == MOTOR_LEG_0_ID) {
                g_leg_status[0] = tmp;
            } else if (tmp.joint_id == MOTOR_LEG_1_ID) {
                g_leg_status[1] = tmp;
            }
            can1_count++;
        }
        while (can2_count < 4) {
            can_motor_receive_status(&hcan2, &tmp);
            if (tmp.joint_id == 0) break;
            int idx = tmp.joint_id - MOTOR_ARM_0_ID;
            if (idx >= 0 && idx < 4) {
                g_arm_status[idx] = tmp;
            }
            can2_count++;
        }

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

    if (!safety_is_system_safe()) {
        safety_isr_fault();
        HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
        return;
    }

    JointCommand_t active_cmd[6];
    safe_cmd_read(active_cmd);

    for (int i = 0; i < 2; i++) {
        int32_t target = active_cmd[i].syn_target;
        uint8_t mode = active_cmd[i].control_mode;
        uint16_t rate_limit = active_cmd[i].torque_rate_limit;
        int32_t max_vel = active_cmd[i].max_velocity;
        uint8_t joint_id = active_cmd[i].joint_id;
        uint8_t pid_idx = active_cmd[i].pid_set_index;

        PIDParams_t *pp = &g_pid_params[pid_idx < PID_PARAM_SET_COUNT ? pid_idx : 0];
        int32_t tlimit = pp->torque_limit;

        if (mode == CTRL_MODE_TORQUE || mode == CTRL_MODE_MIXED) {
            target = torque_rate_limit(g_leg_status[i].torque, target,
                                      rate_limit ? rate_limit : 5000, CONTROL_PERIOD_LEG);
            target = torque_compensate(target, g_leg_status[i].velocity);
            if (tlimit > 0) {
                if (target > tlimit) target = tlimit;
                if (target < -tlimit) target = -tlimit;
            }
            push_can_tx(0, joint_id, target, CTRL_MODE_TORQUE);
        } else if (mode == CTRL_MODE_POSITION) {
            int32_t vlimit = pp->velocity_limit;
            target = trapezoidal_interpolate(g_leg_status[i].position, target,
                                             vlimit ? vlimit : max_vel, CONTROL_PERIOD_LEG);
            push_can_tx(0, joint_id, target, CTRL_MODE_POSITION);
        }
    }

    if (++g_arm_div >= CONTROL_PERIOD_ARM) {
        g_arm_div = 0;

        for (int i = 0; i < 4; i++) {
            int32_t target = active_cmd[i+2].syn_target;
            uint8_t mode = active_cmd[i+2].control_mode;
            uint16_t rate_limit = active_cmd[i+2].torque_rate_limit;
            int32_t max_vel = active_cmd[i+2].max_velocity;
            uint8_t joint_id = active_cmd[i+2].joint_id;
            uint8_t pid_idx = active_cmd[i+2].pid_set_index;

            PIDParams_t *pp = &g_pid_params[pid_idx < PID_PARAM_SET_COUNT ? pid_idx : 0];
            int32_t tlimit = pp->torque_limit;

            if (mode == CTRL_MODE_TORQUE || mode == CTRL_MODE_MIXED) {
                target = torque_rate_limit(g_arm_status[i].torque, target,
                                          rate_limit ? rate_limit : 5000, CONTROL_PERIOD_ARM);
                target = torque_compensate(target, g_arm_status[i].velocity);
                if (tlimit > 0) {
                    if (target > tlimit) target = tlimit;
                    if (target < -tlimit) target = -tlimit;
                }
                push_can_tx(1, joint_id, target, CTRL_MODE_TORQUE);
            } else if (mode == CTRL_MODE_POSITION) {
                int32_t vlimit = pp->velocity_limit;
                target = trapezoidal_interpolate(g_arm_status[i].position, target,
                                                 vlimit ? vlimit : max_vel, CONTROL_PERIOD_ARM);
                push_can_tx(1, joint_id, target, CTRL_MODE_POSITION);
            }
        }
    }

    ReportFrame_t frame;
    report_frame_build(&frame, g_leg_status, g_arm_status);
    report_fifo_write(&frame);

    HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
}
