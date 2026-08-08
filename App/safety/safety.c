#include "safety.h"
#include "can_motor.h"
#include "mode_manager.h"
#include "joint_unit.h"          /* v2.0: g_joint_state[].eso_div_flag (ESO 发散事件) */
#include "global_coordinator.h"  /* v2.0: g_global_coord.jacobian_singular[] (雅可比奇异电平) */
#include "global_pose.h"         /* v2.0: g_global_pose.zupt_fail_flag (ZUPT 失效电平) */

volatile SafetyState_t g_safety_state = {0};

static JointStatus_t g_prev_leg[2] = {0};
static JointStatus_t g_prev_arm[4] = {0};

static void safety_set_fault_level(void)
{
    uint32_t faults = g_safety_state.fault_code;

    /* v1.2: STANDALONE 模式下忽略通信相关故障, 不触发急停 */
    if (g_comm_mode == COMM_MODE_STANDALONE) {
        faults &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
    }

    if (faults & FAULT_ESTOP) {
        g_safety_state.fault_level = FAULT_LEVEL_HW_ESTOP;
    } else if (faults & (FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT)) {
        g_safety_state.fault_level = FAULT_LEVEL_LATCH;
    } else if (faults & (FAULT_POSITION_JUMP | FAULT_VELOCITY_LIMIT | FAULT_TORQUE_LIMIT |
                          FAULT_ARM_SOFT_LIMIT | FAULT_COLLISION | FAULT_ESO_DIVERGE |
                          FAULT_JACOBIAN_SINGULAR | FAULT_ZUPT_FAIL)) {
        g_safety_state.fault_level = FAULT_LEVEL_WARN;
    } else {
        g_safety_state.fault_level = FAULT_LEVEL_NONE;
    }
}

void safety_pull_en_low(void)
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
    /* v2.0: 算法故障保持定时器清零 */
    g_safety_state.eso_div_hold_ms = 0u;
    g_safety_state.jac_sing_hold_ms = 0u;
    g_safety_state.zupt_fail_hold_ms = 0u;

    safety_pull_en_low();
}

void safety_check_estop(void)
{
    /* v1.6.2: ESTOP 引脚(PC3)未接急停按钮, 悬空读到低电平导致误触发.
     * 硬件没接急停时禁用检测, 避免阻止系统运行.
     * TODO: 接上急停按钮后恢复此检测. */
#if 0
    if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET) {
        g_safety_state.estop_active = 1;
        SAFETY_LOCK();
        g_safety_state.fault_code |= FAULT_ESTOP;
        SAFETY_UNLOCK();
    } else {
        g_safety_state.estop_active = 0;
        SAFETY_LOCK();
        g_safety_state.fault_code &= ~FAULT_ESTOP;
        SAFETY_UNLOCK();
    }
#else
    g_safety_state.estop_active = 0;
    SAFETY_LOCK();
    g_safety_state.fault_code &= ~FAULT_ESTOP;
    SAFETY_UNLOCK();
#endif

}

void safety_check_motor_limits(JointStatus_t *leg, JointStatus_t *arm)
{
    uint8_t position_jump = 0;
    uint8_t velocity_fault = 0;
    uint8_t torque_fault = 0;
    uint8_t arm_soft_limit_fault = 0;
    uint8_t collision_fault = 0;

    /* v1.6.6: ZERO_TORQUE 透明模式跳过速度/力矩/位置检查
     * 透明模式用户可自由转动电机, 手动速度轻松超过 30°/s (SAFETY_VELOCITY_LIMIT),
     * 误报 FAULT_VELOCITY_LIMIT 导致 ATP 显示 ERROR. 仅在 GAIT/ARM_ASSIST 检查. */
    if (g_comm_mode == COMM_MODE_STANDALONE && mode_get_local_mode() == 0) {
        SAFETY_LOCK();
        g_safety_state.fault_code &= ~(FAULT_POSITION_JUMP | FAULT_VELOCITY_LIMIT |
                                       FAULT_TORQUE_LIMIT | FAULT_ARM_SOFT_LIMIT |
                                       FAULT_COLLISION);
        SAFETY_UNLOCK();
        for (int i = 0; i < 2; i++) g_prev_leg[i] = leg[i];
        for (int i = 0; i < 4; i++) g_prev_arm[i] = arm[i];
        return;
    }

    /* v1.6.3+fix: 双重防误触发
     *   1) 上电 3 秒内全部跳过 (保留原逻辑)
     *   2) 每关节 "首次有效帧检测": g_prev.*position==0 && temperature>0
     *      说明是从"初始全0态"第一次进入真实数据态, 直接赋值prev不做差值比较.
     *      比固定时间窗口更可靠: 网络/CAN初始化耗时可能超过3秒.  */
    static uint32_t s_power_on_ts = 0;
    if (s_power_on_ts == 0) s_power_on_ts = HAL_GetTick();
    uint8_t skip_time_window = (HAL_GetTick() - s_power_on_ts < 3000) ? 1 : 0;

    for (int i = 0; i < 2; i++) {
        int32_t pos_diff = leg[i].position - g_prev_leg[i].position;
        if (pos_diff < 0) pos_diff = -pos_diff;
        /* 首次有效帧判定: prev全0 且 当前温度>0 (温度>0 = CAN已握手成功) */
        uint8_t first_valid = (g_prev_leg[i].position == 0) && (leg[i].temperature > 0);
        uint8_t skip_position_check = skip_time_window || first_valid;
        if (!skip_position_check && pos_diff > SAFETY_POSITION_JUMP) {
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
        uint8_t first_valid = (g_prev_arm[i].position == 0) && (arm[i].temperature > 0);
        uint8_t skip_position_check = skip_time_window || first_valid;
        if (!skip_position_check && pos_diff > SAFETY_POSITION_JUMP) {
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

        /* ★ v1.6.8fix: 臂关节软限位检查 — 按关节类型分别判断
         * 肩(i=0,2): -30° ~ 150°, 肘(i=1,3): 0° ~ 140° */
        if (!skip_position_check) {
            int32_t pos = arm[i].position;
            if (i == 0 || i == 2) {  /* 左肩/右肩 */
                if (pos > ARM_SHOULDER_POS_MAX || pos < ARM_SHOULDER_POS_MIN)
                    arm_soft_limit_fault = 1;
            } else {                 /* 左肘/右肘 */
                if (pos > ARM_ELBOW_POS_MAX || pos < ARM_ELBOW_POS_MIN)
                    arm_soft_limit_fault = 1;
            }
        }

        /* ★ v1.6.8: 碰撞检测 — 力矩突变 (|Δτ|/Δt > threshold) 触发故障
         * 首帧跳过 (prev 全 0 时差值无意义) */
        if (!skip_position_check) {
            int32_t torque_delta = arm[i].torque - g_prev_arm[i].torque;
            if (torque_delta < 0) torque_delta = -torque_delta;
            if (torque_delta > SAFETY_COLLISION_TORQUE_DELTA) {
                collision_fault = 1;
            }
        }

        g_prev_arm[i] = arm[i];
    }

    SAFETY_LOCK();
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
    /* v1.6.8: 臂软限位 + 碰撞检测 */
    if (arm_soft_limit_fault) {
        g_safety_state.fault_code |= FAULT_ARM_SOFT_LIMIT;
    } else {
        g_safety_state.fault_code &= ~FAULT_ARM_SOFT_LIMIT;
    }
    if (collision_fault) {
        g_safety_state.fault_code |= FAULT_COLLISION;
    } else {
        g_safety_state.fault_code &= ~FAULT_COLLISION;
    }
    SAFETY_UNLOCK();

    safety_set_fault_level();
}

/* v2.0: 算法故障正式接入安全状态机 (ESO 发散 / 雅可比奇异 / ZUPT 失效)
 * 1kHz 调用 (control_isr_process 内, 紧随 joint_drift_immune_update1kHz 之后).
 * 设计: 事件锁存 + 保持定时器 + 电平跟踪, WARN 级, 不拉低 EN, 不锁死机器人.
 *   - ESO 发散 (事件型): 消费 g_joint_state[].eso_div_flag 锁存事件, 清0 + 保持1s
 *   - 雅可比奇异 (电平型): 跟踪 g_global_coord.jacobian_singular[], 不消费源标志
 *   - ZUPT 失效 (电平型): 跟踪 g_global_pose.zupt_fail_flag */
void safety_check_algorithm_faults(void)
{
    uint8_t eso_div = 0u;
    uint8_t jac     = 0u;
    uint8_t zupt    = 0u;

    /* 1. ESO 发散: 遍历 6 关节, 任一 eso_div_flag 置位 → 触发 + 消费(清0)
     *    eso_div_flag 为锁存事件 (JointUnit_ESO3_Update1kHz 仅置位不清),
     *    此处消费后由保持定时器维持可见, 避免降级后 z3 冻结导致标志永不清。 */
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        if (g_joint_state[i].eso_div_flag) {
            eso_div = 1u;
            g_joint_state[i].eso_div_flag = 0u;   /* 消费事件 */
        }
    }

    /* 2. 雅可比奇异: 电平跟踪 (不消费源标志, global_coordinator 每拍自行 set/clear) */
    if (g_global_coord.jacobian_singular[0] || g_global_coord.jacobian_singular[1]) {
        jac = 1u;
    }

    /* 3. ZUPT 失效: 电平跟踪 (global_pose 100Hz 维护 zupt_fail_flag) */
    if (g_global_pose.zupt_fail_flag) {
        zupt = 1u;
    }

    /* 4. 保持定时器: 触发时重装, 否则递减到 0 停止 */
    if (eso_div) {
        g_safety_state.eso_div_hold_ms = ALGO_FAULT_HOLD_MS;
    } else if (g_safety_state.eso_div_hold_ms > 0u) {
        g_safety_state.eso_div_hold_ms--;
    }
    if (jac) {
        g_safety_state.jac_sing_hold_ms = ALGO_FAULT_HOLD_MS;
    } else if (g_safety_state.jac_sing_hold_ms > 0u) {
        g_safety_state.jac_sing_hold_ms--;
    }
    if (zupt) {
        g_safety_state.zupt_fail_hold_ms = ALGO_FAULT_HOLD_MS;
    } else if (g_safety_state.zupt_fail_hold_ms > 0u) {
        g_safety_state.zupt_fail_hold_ms--;
    }

    /* 5. 按保持定时器设置/清除故障位 (临界区保护读-改-写) */
    SAFETY_LOCK();
    if (g_safety_state.eso_div_hold_ms > 0u) {
        g_safety_state.fault_code |= FAULT_ESO_DIVERGE;
    } else {
        g_safety_state.fault_code &= ~FAULT_ESO_DIVERGE;
    }
    if (g_safety_state.jac_sing_hold_ms > 0u) {
        g_safety_state.fault_code |= FAULT_JACOBIAN_SINGULAR;
    } else {
        g_safety_state.fault_code &= ~FAULT_JACOBIAN_SINGULAR;
    }
    if (g_safety_state.zupt_fail_hold_ms > 0u) {
        g_safety_state.fault_code |= FAULT_ZUPT_FAIL;
    } else {
        g_safety_state.fault_code &= ~FAULT_ZUPT_FAIL;
    }
    SAFETY_UNLOCK();
}

void safety_handle_fault(void)
{
    static uint32_t last_handle_ts = 0;
    uint32_t now = HAL_GetTick();
    uint32_t faults = g_safety_state.fault_code;

    /* v1.2: STANDALONE 模式下忽略通信相关故障 */
    if (g_comm_mode == COMM_MODE_STANDALONE) {
        faults &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
    }

    /* v1.6.3+fix: 仅 LATCH/HW_ESTOP 级别才触发 disable_all+拉低EN
     * WARN 级(POSITION_JUMP等) 只上报故障, 不关闭使能 — 防止电机被关后编码器异常反复触发跳变 */
    if (faults & (FAULT_ESTOP | FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT)) {
        if (now - last_handle_ts >= 100) {
            last_handle_ts = now;
            can_motor_disable_all();
            safety_pull_en_low();
            g_safety_state.system_enabled = 0;
        }
    }
}

void safety_task_run(void)
{
    static uint32_t last_fault_code = FAULT_NONE;  /* v1.6.3: 初始化为 FAULT_NONE, 避免首次调用误触发 recovery_counter 重置 */
    uint32_t faults = g_safety_state.fault_code;

    /* v1.2: STANDALONE 模式下忽略通信相关故障, 不触发电机急停 */
    if (g_comm_mode == COMM_MODE_STANDALONE) {
        faults &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
    }

    if (faults != FAULT_NONE) {
        if (faults != last_fault_code) {
            last_fault_code = faults;
            if (faults & (FAULT_ESTOP | FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT)) {
                can_motor_disable_all();
                safety_pull_en_low();
                g_safety_state.system_enabled = 0;
                g_safety_state.recovery_counter = 0;
            } else if (faults & (FAULT_POSITION_JUMP | FAULT_VELOCITY_LIMIT | FAULT_TORQUE_LIMIT |
                                  FAULT_ARM_SOFT_LIMIT | FAULT_COLLISION |
                                  FAULT_ESO_DIVERGE | FAULT_JACOBIAN_SINGULAR | FAULT_ZUPT_FAIL)) {
                /* v1.6.3+fix: WARN 级别 (位置/速度/扭矩超限/臂软限位/碰撞) 不再拉低 EN
                 * 旧逻辑: 拉低EN → 电机失能进入Reset态 → 下次读到的编码器值异常 → 再次触发POSITION_JUMP
                 * 新逻辑: 只上报故障码, 仍保留 system_enabled.  连续 100 个无故障周期后自动清零.
                 * 若真有硬件损坏, 速度/扭矩超限会继续触发并最终由上层策略处理 */
                g_safety_state.recovery_counter = 0;
            }
        }
    } else {
        if (faults != last_fault_code) {
            last_fault_code = faults;
            g_safety_state.recovery_counter = 0;
        }
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
    /* v1.6.4: STANDALONE 模式下忽略通信相关故障, 与 safety_set_fault_level /
     * safety_task_run / safety_handle_fault 保持一致.
     * 原实现直接判断 fault_code==FAULT_NONE, 导致 STANDALONE 模式下电机
     * 不在线时 control_isr 永远走安全分支只发零力矩, 即使切换到 GAIT/ARM
     * 模式也无法执行实际控制 —— 这是"能跑但达不到实际作用"的根本原因. */
    uint32_t faults = g_safety_state.fault_code;
    if (g_comm_mode == COMM_MODE_STANDALONE) {
        faults &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
    }
    return (faults == FAULT_NONE && !g_safety_state.estop_active);
}

uint32_t safety_get_faults(void)
{
    return g_safety_state.fault_code;
}

void safety_request_enable(void)
{
    g_safety_state.recovery_counter = SAFETY_RECOVERY_OK_CYCLES;
}
