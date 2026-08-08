#include "control_isr.h"
#include "main.h"
#include "safety.h"
#include "bsp_config.h"
#include "can_motor.h"
#include "interpolation.h"
#include "udp_protocol.h"
#include "mode_manager.h"
#include "joint_unit.h"
#include <string.h>
#include <math.h>

/* ====== 前置声明 ====== */
static int32_t clamp_int32_local(int32_t v, int32_t lo, int32_t hi);

/* v1.6.8: 弧度→毫度转换 (与 can_motor.c 一致, 1 rad = 57295.78 mdeg) */
#define RAD_TO_MDEG           57295.78f

/* ====== ABO Observer (v1.1) 每关节运行时状态 (必须在 abo_update_one 之前声明) ======
 *  2腿(0~1) + 4臂(2~5), 共6个关节
 *  默认 G=1.0 (零助力, 人感觉不到电机存在), α≈0.01, 泄漏≈0.001 */
ABOState_t g_abo_state[6];  /* v1.7: 非 static, 供 tasks.c/mode_manager.c 访问 */

/* v1.6.2: ABO 助力力矩输出 (mNm), 由 can_motor_send_command() 读取
 * 在 POSITION 模式下作为 MIT 帧前馈力矩叠加 */
int32_t g_abo_assist_torque[6] = {0};

/* v1.6.9: 步态相位 (0.0~1.0), 步态运行时更新, 供 ABO 参数切相 + 上肢反相摆臂
 * ※ 必须放在 abo_update_one() 之前 (函数内会读 g_gait_phase) */
volatile float g_gait_phase = 0.0f;

static void abo_state_init(void)
{
    for (int i = 0; i < 6; i++) {
        g_abo_state[i].bias_est        = 0;
        g_abo_state[i].hpf_state       = 0;
        g_abo_state[i].assist_gain_q10 = ABO_DEFAULT_GAIN_Q10;
        g_abo_state[i].hpf_alpha_q16   = ABO_DEFAULT_ALPHA_Q16;
        g_abo_state[i].bias_leak_q16   = ABO_DEFAULT_LEAK_Q16;
        g_abo_state[i].enable          = 1;
        g_abo_state[i].industrial_mode = 0;       /* v1.7: 默认医疗模式 */
        g_abo_state[i].load_est_q10    = 0;
        g_abo_state[i].load_freeze_cnt = 0;
        g_abo_state[i].bp_lpf_state    = 0;
        g_abo_state[i].tau_prev        = 0;
    }
}

/* v1.6.2: 公共接口, 供 mode_manager 模式切换时重置 ABO
 * 只清 HPF 状态, 保留 bias_est (偏置估计已收敛, 清零会导致误判) */
void abo_state_reset_all(void)
{
    for (int i = 0; i < 6; i++) {
        g_abo_state[i].hpf_state = 0;
    }
}

/* ★ v1.6.8fix: 臂关节重力前馈 + 粘性/库仑摩擦补偿系数
 * 索引: 0=左肩, 1=左肘, 2=右肩, 3=右肘 (相对 arm_status[0~3])
 * 依据 RS01 说明书 + 机械臂实测:
 *   MGL: 重力力矩 (mNm, cos(θ)=1 时) — 肩≈7N·m, 肘≈1.6N·m
 *   BV:  粘性摩擦系数 (×velocity>>10 → mNm)
 *   CC:  库仑摩擦 (mNm, 带方向)
 * 腿关节 (idx 0,1) 系数为 0 — 腿部重力由 ABO 偏置估计器吸收 */
static const int32_t g_gravity_mgl[4]  = { 6963, 1638, 6963, 1638 };  /* 肩/肘 mNm */
static const int32_t g_gravity_bv[4]   = {   80,   40,   80,   40 };  /* 粘性系数 */
static const int32_t g_gravity_cc[4]   = {  300,  150,  300,  150 };  /* 库仑摩擦 mNm */

static int32_t torque_compensate(int32_t target, int32_t velocity,
                                  int32_t position, uint8_t joint_idx)
{
    /* ★ v1.6.8fix2: ZERO_TORQUE 透明模式 — 纯零力矩, 不加任何补偿
     * 之前: 阻尼/摩擦查表返回正值, 速度>0时正反馈导致电机狂转 */
    if (mode_get_local_mode() == 0) {
        return target;  /* 透明模式直接返回原始 target (应为 0) */
    }

    int32_t damping = lookup_damping(velocity);
    int32_t friction = lookup_friction(velocity);
    int32_t ff = 0;

    /* ★ v1.6.8fix: 臂关节重力前馈 + 粘性/库仑摩擦
     * 仅在 ARM_ASSIST 模式 (local mode=2) 下对臂关节施加
     * ZERO_TORQUE / GAIT 模式下不施加 — 否则透明模式下电机会被驱动持续转动 */
    if (joint_idx >= 2 && joint_idx <= 5 &&
        g_comm_mode == COMM_MODE_STANDALONE && mode_get_local_mode() == 2) {
        uint8_t arm_idx = joint_idx - 2;
        float theta = (float)position / RAD_TO_MDEG;  /* mdeg → rad */
        ff  = (int32_t)((float)g_gravity_mgl[arm_idx] * cosf(theta));  /* 重力 */
        ff += (g_gravity_bv[arm_idx] * velocity) >> 10;                 /* 粘性 */
        ff += (velocity > 0) ? g_gravity_cc[arm_idx] : -g_gravity_cc[arm_idx]; /* 库仑 */
    }

    return target + damping + friction + ff;
}

/**
 * @brief  ABO 观测器单关节更新 (运行在 1ms ISR)
 *
 *  核心流程 (Q 定点, 全整数无浮点):
 *    1. 偏置估计器: bias_est += leak * (tau_meas - bias_est)
 *       → 极慢积分 (~1s 时间常数), 吸收重力 + 常值摩擦偏置
 *    2. 高通滤波器: hpf_state += alpha * ((tau_meas - bias_est) - hpf_state)
 *       → 提取人体主动力矩 tau_human (截止 ~1.6 Hz @ 1ms 采样)
 *    3. 助力合成:     tau_assist = gain * tau_human / 1024
 *       → 力放大 G 倍, G=0 为零助力(透明度模式), G>1 为助力模式
 *    4. 叠加命令:     cmd->syn_target += tau_assist
 *       → 仅在 TORQUE / MIXED 模式下生效
 *
 * @param idx    关节索引 (0~5: 0=左髋,1=右髋/膝,2~5=四臂)
 * @param js     关节实时状态 (含 torque 反馈)
 * @param cmd    关节命令 (就地修改 syn_target)
 */
static void abo_update_one(uint8_t idx, JointStatus_t *js, JointCommand_t *cmd)
{
    ABOState_t *abo = &g_abo_state[idx];
    if (!abo->enable) return;

    /* 1. 实时测量力矩 (mNm, 电机编码器推算) */
    int32_t tau_meas = js->torque;

    if (abo->industrial_mode) {
        /* ========== v1.7: 工业模式 ==========
         * 原则: 偏置只吸「电机零漂+恒定摩擦」, 绝不吸外部负载/人力
         *   - leak 极小 (~60s 时常数), 负载突变时冻结
         *   - 带通 0.5-5Hz 提取人力 (双 HPF 串联近似)
         *   - 增益按负载估计调度: 轻载高增益, 重载低增益 */

        /* (a) 负载突变检测: 力矩跳变 > 2000 mNm 或方向反转 */
        int32_t dtau = tau_meas - abo->tau_prev;
        abo->tau_prev = tau_meas;
        if (dtau > 2000 || dtau < -2000) {
            abo->load_freeze_cnt = 500;  /* 冻结偏置 500ms */
        }
        /* 冻结期间不积分偏置 (leak=0), 解冻后恢复极小 leak */
        int32_t ind_leak = abo->load_freeze_cnt ? 0 : 10;  /* 10/65536 ≈ 60s */
        if (abo->load_freeze_cnt) abo->load_freeze_cnt--;

        /* (b) 偏置极慢积分 (只跟零漂/恒定摩擦) */
        int32_t bias_err = tau_meas - abo->bias_est;
        abo->bias_est += (ind_leak * bias_err) >> 16;

        /* (c) 人力提取: 带通 0.5-5Hz
         *   HPF1: fc≈0.5Hz, alpha=200/65536 ≈ 0.003
         *   LPF:  fc≈5Hz,   alpha=2000/65536 ≈ 0.03 */
        int32_t tau_disturb = tau_meas - abo->bias_est;
        int32_t hpf1_err = tau_disturb - abo->hpf_state;
        abo->hpf_state += (200 * hpf1_err) >> 16;     /* HPF1: 去 0Hz */
        int32_t lpf_err = abo->hpf_state - abo->bp_lpf_state;
        abo->bp_lpf_state += (2000 * lpf_err) >> 16;   /* LPF: 去 >5Hz */
        int32_t tau_human = abo->bp_lpf_state;

        /* (d) 负载估计由 tasks.c:industrial_load_estimator() 异步更新 (100Hz LPF)
         *   此处直接读取 load_est_q10, 不再自行估算 */

        /* (e) 增益调度: 按准静态负载分档
         *   load < 5 N·m  → 1.0× (空载/轻载, 全力跟随)
         *   load < 15 N·m → 0.5× (中载, 防超功率)
         *   load ≥ 15 N·m → 0.25× (重载, 仅微辅助) */
        int32_t load_abs = abo->load_est_q10;
        if (load_abs < 0) load_abs = -load_abs;
        int32_t gain;
        if (load_abs < 5000) {
            gain = 1024;        /* 1.0× */
        } else if (load_abs < 15000) {
            gain = 512;         /* 0.5× */
        } else {
            gain = 256;         /* 0.25× */
        }
        abo->assist_gain_q10 = (uint16_t)gain;

        /* (f) 助力输出 */
        int32_t tau_assist = mode_get_abo_enabled() ? ((gain * tau_human) >> 10) : 0;
        tau_assist = clamp_int32_local(tau_assist, -3000, 3000);

        g_abo_assist_torque[idx] = tau_assist;
        if (cmd->control_mode == CTRL_MODE_TORQUE || cmd->control_mode == CTRL_MODE_MIXED) {
            cmd->syn_target += tau_assist;
        }
        return;
    }

    /* ========== 医疗模式 (原有逻辑) ========== */
    /* v1.6.9: 腿/臂 ABO 参数显式分离 */
    if (idx < 2) {
        uint8_t is_stance;
        if (idx == 0) {
            is_stance = mode_get_gait_running() && (g_gait_phase <= 0.5f);
        } else {
            is_stance = mode_get_gait_running() && (g_gait_phase > 0.5f);
        }
        if (is_stance) {
            abo->hpf_alpha_q16 = 4000;
            abo->bias_leak_q16 = 66;
        } else {
            abo->hpf_alpha_q16 = 1000;
            abo->bias_leak_q16 = 200;
        }
    } else {
        abo->hpf_alpha_q16 = 1000;
        abo->bias_leak_q16 = 20;
    }

    int32_t alpha = abo->hpf_alpha_q16;
    int32_t leak  = abo->bias_leak_q16;
    int32_t gain  = abo->assist_gain_q10;

    /* 2. 偏置估计器 (泄漏积分, 跟踪重力 + 常值摩擦) */
    int32_t bias_err = tau_meas - abo->bias_est;
    abo->bias_est += (leak * bias_err) >> 16;

    /* 3. 一阶 HPF 提取人体主动力矩 */
    int32_t tau_disturb = tau_meas - abo->bias_est;
    int32_t hpf_err = tau_disturb - abo->hpf_state;
    abo->hpf_state += (alpha * hpf_err) >> 16;
    int32_t tau_human = abo->hpf_state;

    /* 4. 助力力矩放大 */
    int32_t tau_assist = mode_get_abo_enabled() ? ((gain * tau_human) >> 10) : 0;
    tau_assist = clamp_int32_local(tau_assist, -3000, 3000);

    /* 5. 输出 */
    g_abo_assist_torque[idx] = tau_assist;
    if (cmd->control_mode == CTRL_MODE_TORQUE || cmd->control_mode == CTRL_MODE_MIXED) {
        cmd->syn_target += tau_assist;
    }
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
        CAN_TypeDef *can = hcan->Instance;
        /* ★ v1.6.8fix2: 检查 TX 邮箱是否空闲, 满则重新入队等待下次发送 (防 0x13 丢帧) */
        if ((can->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0) {
            can_tx_fifo_write(&entry);  /* 重新入队 */
            break;  /* 邮箱满, 等下次再发 */
        }
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

/* ====== 本地控制 (STANDALONE 模式) 状态 ====== */
typedef enum {
    LOCAL_MODE_ZERO_TORQUE = 0,   /* 零扭矩透明 (默认安全态, ABO 仍跑偏置收敛) */
    LOCAL_MODE_GAIT        = 1,   /* 自适应步态 (下肢 ABO 助力 + 位置步态) */
    LOCAL_MODE_ARM_ASSIST  = 2    /* 鳌臂助力 (上肢 ABO 重力悬停) */
} LocalMode_e;

volatile LocalMode_e g_local_mode = LOCAL_MODE_ZERO_TORQUE;
volatile uint8_t     g_local_gait_running = 0;     /* 0=停止, 1=运行 */
volatile uint32_t    g_local_gait_start_ms = 0;

/* 步态轨迹参数 (与 basic 版 main.c 一致) */
#define LOCAL_GAIT_PERIOD_MS      4000
#define LOCAL_GAIT_MAX_VEL_MDEG_S 60000
#define LOCAL_GAIT_NUM_POINTS     5

static const int32_t g_local_gait_table[LOCAL_GAIT_NUM_POINTS][2] = {
    /* Left Hip, Right Hip (mdeg) */
    {       0,   60000 },   /*   0% */
    {   30000,   30000 },   /*  25% */
    {   60000,       0 },   /*  50% */
    {   30000,   30000 },   /*  75% */
    {       0,   60000 }    /* 100% */
};

/* 平滑输出 (当前目标角度, 用于梯形速度插值) */
static int32_t g_local_left_hip  = 0;
static int32_t g_local_right_hip = 0;

/* ★ v1.6.9fix2: GAIT→ZERO 平滑回归状态 (文件作用域, 跨模式清除) */
static uint8_t  g_gait_exit_active = 0;
static uint32_t g_gait_exit_ts = 0;
static int32_t  g_exit_left_hip = 0;
static int32_t  g_exit_right_hip = 0;

static int32_t clamp_int32_local(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief  STANDALONE 模式下生成本地命令 (步态/零扭矩/鳌臂助力)
 * @note   运行在 1ms ISR 上下文中, 必须无阻塞
 *         v1.2: 所有本地模式下 ABO 均使能 (偏置持续收敛)
 *               - ZERO_TORQUE: 零力矩 + ABO零助力 (透明跟随)
 *               - GAIT:        下肢位置步态 + ABO助力, 上肢零力矩
 *               - ARM_ASSIST:  下肢零力矩, 上肢ABO重力悬停
 */
static void local_cmd_generate(JointCommand_t cmd[6])
{
    /* v1.6.3: arm_level → ABO gain 映射表 (Q10)
     * OFF=0x0, LIGHT=0x200(0.5x), NORM=0x400(1.0x), STRONG=0x600(1.5x) */
    static const uint16_t arm_gain_map[4] = {0, 512, 1024, 1536};
    uint16_t arm_gain = arm_gain_map[mode_get_arm_level() & 0x03];

    /* 初始化所有关节 */
    for (int i = 0; i < 6; i++) {
        cmd[i].joint_id = 0;
        cmd[i].control_mode = CTRL_MODE_TORQUE;
        cmd[i].syn_target = 0;
        cmd[i].max_velocity = 60000;
        cmd[i].max_acceleration = (uint16_t)20000;
        cmd[i].torque_rate_limit = (uint16_t)50000;
        cmd[i].pid_set_index = 0;
        cmd[i].kp = 10;
        cmd[i].kd = 2;
        /* v1.2: STANDALONE 模式默认开启 ABO, 偏置始终收敛 */
        cmd[i].abo_enable = 1;
        cmd[i].assist_gain_q10 = ABO_DEFAULT_GAIN_Q10;
        cmd[i].hpf_alpha_q16 = ABO_DEFAULT_ALPHA_Q16;
        cmd[i].bias_leak_q16 = ABO_DEFAULT_LEAK_Q16;
    }

    /* 关节 ID 映射: leg[0]=ID=0x01(左髋 L-Hip), leg[1]=ID=0x02(右髋 R-Hip), arm[0~3]=ID=0x10~0x13 */
    cmd[0].joint_id = MOTOR_LEG_0_ID;   /* 0x01 */
    cmd[1].joint_id = MOTOR_LEG_1_ID;   /* 0x02 */
    cmd[2].joint_id = MOTOR_ARM_0_ID;    /* 0x10 */
    cmd[3].joint_id = MOTOR_ARM_0_ID + 1;
    cmd[4].joint_id = MOTOR_ARM_0_ID + 2;
    cmd[5].joint_id = MOTOR_ARM_0_ID + 3;

    /* v1.6.3: 先将 ABO 使能和增益同步到 g_abo_state
     * 后续各模式分支再根据模式覆盖 */
    for (int i = 0; i < 6; i++) {
        g_abo_state[i].enable = 1;
        g_abo_state[i].assist_gain_q10 = ABO_DEFAULT_GAIN_Q10;
    }

    if (mode_get_local_mode() == 0) {  /* MODE_ZERO_TORQUE */
        /* ★ v1.6.9fix2: GAIT→ZERO 平滑回归 — 防止腿部瞬间失力暴摔
         * 原逻辑: GAIT(POSITION) → ZERO(TORQUE 0) 瞬间切换, 电机失支撑, 腿自由落下
         * 新逻辑: 先用 POSITION 控制以 15°/s 缓慢回归 0°, 3 秒后切换为纯 TORQUE 0 */

        /* 检测 GAIT→ZERO 切换 */
        if (g_mode_mgr.prev_mode == MODE_GAIT &&
            g_mode_mgr.transitioning && !g_gait_exit_active) {
            g_gait_exit_active = 1;
            g_gait_exit_ts = HAL_GetTick();
            g_exit_left_hip  = g_leg_status[0].position;  /* 记录当前实际位置 */
            g_exit_right_hip = g_leg_status[1].position;
        }

        if (g_gait_exit_active) {
            uint32_t elapsed = HAL_GetTick() - g_gait_exit_ts;
            if (elapsed < 3000) {  /* 3 秒平滑回归 */
                /* 预插值目标: 15°/s 向 0° 收敛 */
                g_exit_left_hip = trapezoidal_interpolate(g_exit_left_hip, 0,
                                                           15000, CONTROL_PERIOD_LEG);
                g_exit_right_hip = trapezoidal_interpolate(g_exit_right_hip, 0,
                                                            15000, CONTROL_PERIOD_LEG);
                /* 下肢: POSITION 模式缓慢回归 */
                cmd[0].control_mode = CTRL_MODE_POSITION;
                cmd[0].syn_target = g_exit_left_hip;
                cmd[0].max_velocity = 15000;  /* 15°/s 安全速度 */
                cmd[0].abo_enable = 0;          /* ★ v1.7: ZERO 期间 ABO 全关 */
                g_abo_state[0].enable = 0;
                g_abo_state[0].assist_gain_q10 = 0;
                cmd[0].assist_gain_q10 = 0;

                cmd[1].control_mode = CTRL_MODE_POSITION;
                cmd[1].syn_target = g_exit_right_hip;
                cmd[1].max_velocity = 15000;
                cmd[1].abo_enable = 0;          /* ★ v1.7: ZERO 期间 ABO 全关 */
                g_abo_state[1].enable = 0;
                g_abo_state[1].assist_gain_q10 = 0;
                cmd[1].assist_gain_q10 = 0;

                /* 上肢: TORQUE 0 + ABO 关闭 */
                for (int i = 2; i < 6; i++) {
                    cmd[i].control_mode = CTRL_MODE_TORQUE;
                    cmd[i].syn_target = 0;
                    cmd[i].abo_enable = 0;
                    g_abo_state[i].enable = 0;
                    g_abo_state[i].assist_gain_q10 = 0;
                    cmd[i].assist_gain_q10 = 0;
                    g_abo_assist_torque[i] = 0;
                }
                return;
            } else {
                /* 回归完成, 切换为纯零力矩 */
                g_gait_exit_active = 0;
            }
        }

        /* ★ v1.7: 正常 ZERO 模式 — 彻底切断 ABO (工业安全要求)
         * enable=0: 观测器停止更新 (不积分偏置, 不滤波)
         * assist_gain=0: 即使有残留也不输出
         * g_abo_assist_torque=0: 清空前馈通道
         * 原逻辑 enable=1 保留偏置收敛 → 工业场景有残留助力风险 */
        for (int i = 0; i < 6; i++) {
            cmd[i].control_mode = CTRL_MODE_TORQUE;
            cmd[i].syn_target = 0;
            cmd[i].abo_enable = 0;          /* ★ 关观测器 */
            g_abo_state[i].enable = 0;      /* ★ 关观测器 */
            g_abo_state[i].assist_gain_q10 = 0;
            cmd[i].assist_gain_q10 = 0;
            g_abo_assist_torque[i] = 0;     /* ★ 清前馈 */
        }
        g_local_left_hip = 0;
        g_local_right_hip = 0;
        return;
    }

    /* GAIT 模式: 左髋 + 右髋 跑步态 (位置控制) + ABO助力, 手臂零力矩
     * ★ v1.6.8fix: 步态未启动时腿部 HOLD 当前位置 (非拉回0位), 手臂强制零增益 */
    if (mode_get_local_mode() == 1) {  /* MODE_GAIT */
        g_gait_exit_active = 0;  /* ★ v1.6.9fix2: 进入 GAIT 时清除回归标志 */
        int32_t lhip_target = 0, rhip_target = 0;

        if (mode_get_gait_running()) {
            uint32_t elapsed = HAL_GetTick() - mode_get_gait_start_ms();

            /* v1.6.2: 启动后 2 秒渐变, 防止位置/力矩突变暴走 */
            uint32_t ramp_ms = 2000;
            int32_t ramp_scale = (elapsed >= ramp_ms) ? 1024 :
                (int32_t)((uint32_t)1024 * elapsed / ramp_ms);

            uint32_t phase = elapsed % LOCAL_GAIT_PERIOD_MS;
            float t = (float)phase / (float)LOCAL_GAIT_PERIOD_MS;
            g_gait_phase = t;   /* v1.6.9: 更新全局相位, 供上肢反相摆臂使用 */
            float seg = t * (float)(LOCAL_GAIT_NUM_POINTS - 1);
            uint8_t idx = (uint8_t)seg;
            float frac = seg - (float)idx;

            if (idx >= LOCAL_GAIT_NUM_POINTS - 1) idx = LOCAL_GAIT_NUM_POINTS - 2;

            int32_t l0 = g_local_gait_table[idx][0];
            int32_t l1 = g_local_gait_table[idx + 1][0];
            int32_t r0 = g_local_gait_table[idx][1];
            int32_t r1 = g_local_gait_table[idx + 1][1];

            lhip_target = l0 + (int32_t)((float)(l1 - l0) * frac);
            rhip_target = r0 + (int32_t)((float)(r1 - r0) * frac);

            /* 渐变缩放: ramp_scale/1024 */
            lhip_target = (lhip_target * ramp_scale) >> 10;
            rhip_target = (rhip_target * ramp_scale) >> 10;

            /* 步态启动 → POSITION 模式跑步态轨迹 */
            cmd[0].control_mode = CTRL_MODE_POSITION;
            cmd[0].syn_target = g_local_left_hip;
            cmd[0].max_velocity = LOCAL_GAIT_MAX_VEL_MDEG_S;
            cmd[0].abo_enable = 1;

            cmd[1].control_mode = CTRL_MODE_POSITION;
            cmd[1].syn_target = g_local_right_hip;
            cmd[1].max_velocity = LOCAL_GAIT_MAX_VEL_MDEG_S;
            cmd[1].abo_enable = 1;
        } else {
            /* ★ v1.6.8fix: 步态未启动 → 零力矩透明 (与 ZERO_TORQUE 一致)
             * 不能用 POSITION 模式 HOLD 当前位置 — 反馈未建立时 position=0,
             * Kp=10 会把腿强行拉到 0 位 → 电机转 */
            cmd[0].control_mode = CTRL_MODE_TORQUE;
            cmd[0].syn_target = 0;
            cmd[0].abo_enable = 1;
            g_abo_state[0].assist_gain_q10 = 0;

            cmd[1].control_mode = CTRL_MODE_TORQUE;
            cmd[1].syn_target = 0;
            cmd[1].abo_enable = 1;
            g_abo_state[1].assist_gain_q10 = 0;
        }

        /* 安全限位 (仅步态运行时用于目标值, 零力矩时无意义但不影响) */
        lhip_target = clamp_int32_local(lhip_target, LOCAL_HIP_POS_MIN_MDEG, LOCAL_HIP_POS_MAX_MDEG);
        rhip_target = clamp_int32_local(rhip_target, LOCAL_HIP_POS_MIN_MDEG, LOCAL_HIP_POS_MAX_MDEG);

        /* 梯形速度平滑 (仅步态运行时有效) */
        g_local_left_hip  = trapezoidal_interpolate(g_local_left_hip,  lhip_target,
                                                      LOCAL_GAIT_MAX_VEL_MDEG_S, CONTROL_PERIOD_LEG);
        g_local_right_hip = trapezoidal_interpolate(g_local_right_hip, rhip_target,
                                                      LOCAL_GAIT_MAX_VEL_MDEG_S, CONTROL_PERIOD_LEG);

        /* === v1.6.9: 上肢全身助力 (GAIT 模式) ===
         * P0 重力补偿: τ_ff = mgl·cos(θ), 仅步态运行时开 (无负载/静止时关闭防暴走)
         * P1 阻抗跟踪: 低刚度跟踪反相摆臂轨迹, K/B 连续调制避免 phi=0.5 跳变振荡
         * 步态未运行时: 零力矩透明 (与 ZERO 一致), 不加任何补偿
         *   ※ 无负载测试时重力前馈 7N·m 无处消耗会驱动电机暴走, 必须步态运行才开 */
        {
            float phi = g_gait_phase;  /* 0.0~1.0, 步态运行时更新 */

            for (int i = 2; i < 6; i++) {
                uint8_t arm_idx = i - 2;  /* 0=左肩,1=左肘,2=右肩,3=右肘 */
                int is_shoulder = (arm_idx == 0 || arm_idx == 2);

                if (!mode_get_gait_running()) {
                    /* 步态未运行: 零力矩透明, ABO 偏置继续收敛 */
                    cmd[i].control_mode = CTRL_MODE_TORQUE;
                    cmd[i].syn_target   = 0;
                    cmd[i].abo_enable   = 1;
                    g_abo_state[i].enable = 1;
                    g_abo_state[i].assist_gain_q10 = 0;
                    cmd[i].assist_gain_q10 = 0;
                    continue;
                }

                /* 步态运行: 阻抗跟踪 (反相摆臂)
                 * ★ v1.6.9fix2: 关闭重力前馈 — 未做零位标定, cos(θ) 计算错误
                 *   原代码 tau_ff = MGL×cos(θ) = 6963 mNm → clamp 2000 → 2N·m 常值力矩
                 *   导致「稍微施力, 手臂剧烈对抗」。需硬件标定 θ₀ + 符号验证后才能开启 */
                float pos_rad = (float)g_arm_status[arm_idx].position / RAD_TO_MDEG;
                float vel_rad = (float)g_arm_status[arm_idx].velocity / RAD_TO_MDEG;

                /* 1) 重力前馈 — 暂时关闭 (需零位标定后用 MGL×cos(θ-θ₀) 重开启) */
                float tau_ff = 0.0f;

                /* 2) 阻抗跟踪: 反相摆臂, 低刚度 */
                float arm_phi = (arm_idx >= 2) ? phi : phi + 0.5f;
                if (arm_phi >= 1.0f) arm_phi -= 1.0f;

                float two_pi = 2.0f * 3.14159265f;
                float ref = is_shoulder ?
                    (0.5f * (-cosf(two_pi * arm_phi))) :
                    (0.5f + 0.3f * sinf(two_pi * arm_phi));

                /* v1.6.9fix: K/B 连续调制 (避免 phi=0.5 跳变振荡)
                 * 站立相(phi≈0.5)大, 摆动相(phi≈0/1)小, 用余弦平滑过渡 */
                float phase_gain = 0.5f - 0.5f * cosf(two_pi * phi);  /* 0→1→0, 站立相=1 */
                float K = is_shoulder ? (0.3f + 1.2f * phase_gain) : (0.1f + 0.7f * phase_gain);
                float B = is_shoulder ? (0.2f + 1.0f * phase_gain) : (0.05f + 0.3f * phase_gain);

                /* 阻抗力矩 (Nm → mNm): τ = K(ref-pos) + B(0-vel) */
                float tau_imp = (K * (ref - pos_rad) + B * (0.0f - vel_rad)) * 1000.0f;

                /* 3) 合成 + 限幅 (mNm)
                 * ★ v1.6.9fix2: 限幅从 ±2000 降到 ±500 (0.5 N·m), 防止大力矩对抗用户 */
                int32_t tau_total = (int32_t)(tau_ff + tau_imp);
                tau_total = clamp_int32_local(tau_total, -500, 500);  /* ±0.5 N·m, 柔软安全 */

                cmd[i].control_mode = CTRL_MODE_TORQUE;
                cmd[i].syn_target   = tau_total;
                cmd[i].abo_enable   = 1;
                g_abo_state[i].enable = 1;
                g_abo_state[i].assist_gain_q10 = 0;
                cmd[i].assist_gain_q10 = 0;
            }
        }
        return;
    }

    /* ARM_ASSIST 模式: 下肢零力矩 + ABO, 上肢 ABO 重力悬停
     * v1.6.3: arm_level 映射到实际增益:
     *   OFF→0x0, LIGHT→0x200(0.5x), NORM→0x400(1.0x), STRONG→0x600(1.5x)
     * ★ v1.6.8fix: 臂部专用 ABO 参数 (低增益 0.25×, 高频 HPF 3Hz, 慢漏 4s) */
    if (mode_get_local_mode() == 2) {  /* MODE_ARM_ASSIST */
        g_local_gait_running = 0;
        /* 下肢: 零力矩 + ABO 透明跟随, 增益=0 (OFF时不助力) */
        uint16_t leg_gain = (mode_get_arm_level() == ARM_ASSIST_OFF) ? 0 : ABO_DEFAULT_GAIN_Q10;
        for (int i = 0; i < 2; i++) {
            g_abo_state[i].assist_gain_q10 = leg_gain;
            cmd[i].assist_gain_q10 = leg_gain;
        }
        /* 上肢: ABO + 重力前馈
         * ★ v1.6.8fix: 臂部专用 ABO 参数 — 低频、慢漏、低增益
         *   gain=256 (0.25×): 温和助力, 防振荡
         *   alpha=1500 (~3 Hz): 比默认 1.6 Hz 高, 更快响应人体主动力矩
         *   leak=25 (~4s): 比默认 1s 慢, 偏置更稳定 */
        uint16_t arm_gain_q10 = (mode_get_arm_level() == ARM_ASSIST_OFF) ? 0 : 256;
        for (int i = 2; i < 6; i++) {
            cmd[i].control_mode = CTRL_MODE_TORQUE;
            cmd[i].syn_target = 0;
            cmd[i].abo_enable = (arm_gain_q10 > 0) ? 1 : 0;
            g_abo_state[i].enable = (arm_gain_q10 > 0) ? 1 : 0;
            g_abo_state[i].assist_gain_q10 = arm_gain_q10;
            cmd[i].assist_gain_q10 = arm_gain_q10;
            /* ★ 臂部专用 ABO 滤波参数 (覆盖默认值) */
            g_abo_state[i].hpf_alpha_q16 = 1500;   /* ~3 Hz */
            g_abo_state[i].bias_leak_q16 = 25;     /* ~4s */
        }
        return;
    }

    /* ★ v1.7: 工业助力模式 — 全关节 ABO 工业模式, TORQUE 控制 + 负载自适应增益
     * 无步态相位, 无位置控制, 纯力矩透明跟随 + 人力放大
     * ABO 工业分支 (abo_update_one) 负责:
     *   - 极慢偏置泄漏 (60s), 负载突变冻结
     *   - 带通 0.5-5Hz 提取人力
     *   - 增益按负载调度 (轻载 1.0× / 中载 0.5× / 重载 0.25×) */
    if (mode_get_local_mode() == 3) {  /* MODE_INDUSTRIAL */
        g_gait_exit_active = 0;
        g_local_gait_running = 0;

        for (int i = 0; i < 6; i++) {
            cmd[i].control_mode = CTRL_MODE_TORQUE;  /* 纯力矩模式 */
            cmd[i].syn_target = 0;                    /* ABO 叠加助力 */
            cmd[i].abo_enable = 1;
            g_abo_state[i].enable = 1;
            /* gain 由 abo_update_one 工业分支动态设置, 此处不覆盖 */
            cmd[i].assist_gain_q10 = g_abo_state[i].assist_gain_q10;
        }
        return;
    }

    /* 兜底: 未知模式 → 安全零力矩 */
    for (int i = 0; i < 6; i++) {
        cmd[i].control_mode = CTRL_MODE_TORQUE;
        cmd[i].syn_target = 0;
        cmd[i].abo_enable = 0;
        g_abo_state[i].enable = 0;
        g_abo_state[i].assist_gain_q10 = 0;
    }
}

#define CAN_TIMEOUT_MS      100
static uint32_t g_can1_timeout_cnt = 0;
static uint32_t g_can2_timeout_cnt = 0;

void control_isr_init(void)
{
    abo_state_init();
    joint_unit_init();   /* 架构评审 #2: 关节标准化接口 (配置表校验) */

    /* TIM6 完整初始化 (之前缺少这步!) */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 84 - 1;       /* 84MHz / 84 = 1MHz (1us) */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000 - 1;         /* 1000 * 1us = 1ms */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        printf("[TIM6] HAL_TIM_Base_Init FAILED!\r\n");
    } else {
        printf("[TIM6] Init OK: 1ms tick\r\n");
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        printf("[TIM6] HAL_TIM_Base_Start_IT FAILED!\r\n");
    }
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
    /* 架构评审 #2: 经 JointUnit 统一切断所有关节驱动级使能 */
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        joint_disable_hw(i);
    }
    g_safety_state.system_enabled = 0;
}

/* CANopen SYNC 帧 (COB-ID 0x80): 广播给所有电机同步采样编码器
 * 确保 6 个电机在同一时刻采样, 状态时间戳对齐, 消除 ±1ms 抖动 */
#define CAN_SYNC_ID  0x80
static void can_send_sync(CAN_HandleTypeDef *hcan)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef txh;
    uint8_t data = 0;  /* SYNC 帧 0 字节数据 (计数器可选) */
    txh.DLC = 0;
    txh.IDE = CAN_ID_STD;
    txh.RTR = CAN_RTR_DATA;
    txh.StdId = CAN_SYNC_ID;
    HAL_CAN_AddTxMessage(hcan, &txh, &data, &tx_mailbox);
}

extern volatile uint32_t g_dbg_isr_calls;

void control_isr_process(void)
{
    g_dbg_isr_calls++;

    /* SYNC 先发出: CyberGear 在 SYNC 边沿采样, PDO 在本周期内到达
     * v1.6.4: 不对 CAN2 发 SYNC — RS01 不需要 SYNC, 且电机未上电时
     * 无 ACK 的 SYNC 帧会累积错误计数导致 CAN2 Bus-Off, 电机上线后也无法通信 */
    can_send_sync(&hcan1);

    HAL_IWDG_Refresh(&hiwdg);

    safety_check_estop();

    if (!safety_is_system_safe()) {
        /* v1.6.4: 仅急停或 HOST 模式故障才强制零力矩返回;
         * STANDALONE 模式下警告级故障(POSITION_JUMP等)降级继续执行本地控制,
         * 避免系统卡死无响应 (原逻辑任何故障→零力矩→切换模式无效) */
        if (g_safety_state.estop_active || g_comm_mode != COMM_MODE_STANDALONE) {
            if (g_safety_state.estop_active) {
                safety_isr_fault();
            }
            for (uint8_t i = 0; i < JOINT_COUNT; i++) {
                joint_push_tx(i, 0, CTRL_MODE_TORQUE);
            }
            ReportFrame_t frame;
            report_frame_build(&frame, g_leg_status, g_arm_status);
            report_fifo_write(&frame);
            HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
            return;
        }
        /* STANDALONE 非急停: 降级继续执行本地控制 */
    }

    {
        JointStatus_t tmp;
        int can1_count = 0, can2_count = 0;
        while (can1_count < 2) {
            tmp.joint_id = 0;  /* ★ v1.6.8fix2: 重置, 防止无帧时残留旧值导致 break 失效 */
            can_motor_receive_status(&hcan1, &tmp);
            if (tmp.joint_id == 0) break;
            /* 架构评审 #2: motor_id → 统一 idx (CAN1 仅腿 idx 0,1) */
            uint8_t jidx = joint_idx_from_motor_id(tmp.joint_id);
            if (jidx < 2) {
                *joint_status_ptr(jidx) = tmp;
            }
            can1_count++;
        }
        /* ★ v1.6.8fix2: 增加到 8 次, 确保 4 个电机帧都能被处理 (防止 FIFO 积压) */
        while (can2_count < 8) {
            tmp.joint_id = 0;  /* ★ 重置, 防止无帧时残留旧值导致 break 失效 */
            can_motor_receive_status(&hcan2, &tmp);
            if (tmp.joint_id == 0) break;
            /* 架构评审 #2: motor_id → 统一 idx (CAN2 仅臂 idx 2~5) */
            uint8_t jidx = joint_idx_from_motor_id(tmp.joint_id);
            if (jidx >= 2 && jidx < JOINT_COUNT) {
                *joint_status_ptr(jidx) = tmp;
            }
            can2_count++;
        }

        if (can1_count > 0) {
            g_can1_timeout_cnt = 0;
            SAFETY_LOCK();
            g_safety_state.fault_code &= ~FAULT_CAN1_TIMEOUT;
            SAFETY_UNLOCK();
        } else {
            if (g_can1_timeout_cnt < CAN_TIMEOUT_MS) {
                g_can1_timeout_cnt++;
            } else {
                SAFETY_LOCK();
                g_safety_state.fault_code |= FAULT_CAN1_TIMEOUT;
                SAFETY_UNLOCK();
            }
        }

        if (can2_count > 0) {
            g_can2_timeout_cnt = 0;
            SAFETY_LOCK();
            g_safety_state.fault_code &= ~FAULT_CAN2_TIMEOUT;
            SAFETY_UNLOCK();
        } else {
            if (g_can2_timeout_cnt < CAN_TIMEOUT_MS) {
                g_can2_timeout_cnt++;
            } else {
                SAFETY_LOCK();
                g_safety_state.fault_code |= FAULT_CAN2_TIMEOUT;
                SAFETY_UNLOCK();
            }
        }
    }

    safety_check_motor_limits(g_leg_status, g_arm_status);

    if (!safety_is_system_safe()) {
        /* v1.6.4: 与上方第一处安全分支一致: 仅急停或 HOST 模式故障才强制零力矩返回;
         * STANDALONE 模式下警告级故障降级继续执行本地控制 */
        if (g_safety_state.estop_active || g_comm_mode != COMM_MODE_STANDALONE) {
            if (g_safety_state.estop_active) {
                safety_isr_fault();
            }
            for (uint8_t i = 0; i < JOINT_COUNT; i++) {
                joint_push_tx(i, 0, CTRL_MODE_TORQUE);
            }
            ReportFrame_t frame;
            report_frame_build(&frame, g_leg_status, g_arm_status);
            report_fifo_write(&frame);
            HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
            return;
        }
        /* STANDALONE 非急停: 降级继续执行本地控制 */
    }

    JointCommand_t active_cmd[6];
    if (g_comm_mode == COMM_MODE_HOST) {
        if (!safe_cmd_read(active_cmd)) {
            /* v1.6.3+fix: CRC失败或无新帧: fallback到本地命令(零力矩自由转动),
             * 而非g_active_command默认{0}→POSITION模式target=0(kp=10闭环拉回0→手动转动过流Error) */
            local_cmd_generate(active_cmd);
        }
        /* 保存有效命令到 g_active_command 供下次超时使用 */
        {
            extern JointCommand_t g_active_command[6];
            memcpy(g_active_command, active_cmd, sizeof(active_cmd));
        }
    } else {
        local_cmd_generate(active_cmd);   /* STANDALONE: 本地步态/零扭矩控制 */
    }

    /* ====== 零 IMU 漂移免疫包 §A + §B (1kHz, 每关节) ======
     *  §A: 温度表 + 零负载窗口 在线力矩零漂修正
     *  §B: ESO3 抗漂移 (默认 eso_enable=0 不运行, 保 legacy ABO 兼容)
     *  必须放在 ABO/控制计算之前, 以便 tau_meas_nm 已经去漂. */
    {
        const float dt_s = 0.001f;   /* 1 ms */
        for (uint8_t i = 0; i < JOINT_COUNT; i++) {
            joint_drift_immune_update1kHz(i, dt_s);
        }
        /* §C-boost: 1kHz 递减 boost timer (100Hz 触发, 1kHz 衰减) */
        joint_eso_boost_tick_1kHz();

        /* v2.0: 算法故障正式接入安全状态机 (ESO发散/雅可比奇异/ZUPT失效)
         *   1kHz, 紧随 drift_immune 更新之后, 即时消费 eso_div_flag 锁存事件。
         *   WARN 级 + 保持定时器, 不拉低 EN, 不锁死机器人。 */
        safety_check_algorithm_faults();
    }

    /* ====== v1.1 ABO Observer: 每关节偏置估计 + HPF 提取人体力矩 + 助力叠加 ======
     *  放在"命令读取之后、控制计算之前"，就地修改 active_cmd[].syn_target
     *  RK3506 10ms 下发 assist_gain / hpf_alpha / bias_leak 参数 */
    for (int i = 0; i < 2; i++) {
        abo_update_one(i, &g_leg_status[i], &active_cmd[i]);
    }
    for (int i = 0; i < 4; i++) {
        abo_update_one(i + 2, &g_arm_status[i], &active_cmd[i + 2]);
    }

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
            /* ★ v1.6.8fix2: ZERO_TORQUE 模式直接发零力矩, 跳过 rate_limit/compensate
             * 否则 rate_limit 从当前力矩渐变到 0, 期间电机持续受力矩 → 狂转 */
            if (mode_get_local_mode() == 0) {
                push_can_tx(0, joint_id, 0, CTRL_MODE_TORQUE);
            } else {
                target = torque_rate_limit(g_leg_status[i].torque, target,
                                          rate_limit ? rate_limit : 5000, CONTROL_PERIOD_LEG);
                target = torque_compensate(target, g_leg_status[i].velocity,
                                           g_leg_status[i].position, (uint8_t)i);
                if (tlimit > 0) {
                    if (target > tlimit) target = tlimit;
                    if (target < -tlimit) target = -tlimit;
                }
                push_can_tx(0, joint_id, target, CTRL_MODE_TORQUE);
            }
        } else if (mode == CTRL_MODE_POSITION) {
            int32_t vlimit = pp->velocity_limit;
            target = trapezoidal_interpolate(g_leg_status[i].position, target,
                                             vlimit ? vlimit : max_vel, CONTROL_PERIOD_LEG);
            push_can_tx(0, joint_id, target, CTRL_MODE_POSITION);
        }
    }

    /* 臂关节 1ms 控制 (与腿关节、状态接收周期一致, 避免采样频率混叠)
     * ★ v1.6.8fix: 臂部碰撞检测 — 力矩突变 > 阈值 → 瞬间零力矩失能
     * v1.6.9: 阈值从硬编码 800 改为 SAFETY_COLLISION_TORQUE_DELTA(8000), 与 safety.h 一致
     *         模式切换时重置 arm_tau_prev, 避免 ZERO→GAIT 切换时残留 0 导致误触发 */
    {
        static int32_t arm_tau_prev[4] = {0};
        static uint8_t prev_local_mode = 0xFF;
        uint8_t cur_mode = mode_get_local_mode();
        if (cur_mode != prev_local_mode) {
            /* 模式切换: 重置 prev 为当前力矩, 使本帧 dtau=0, 防止误触发碰撞 */
            prev_local_mode = cur_mode;
            for (int i = 0; i < 4; i++) {
                arm_tau_prev[i] = g_arm_status[i].torque;
            }
        }
        for (int i = 0; i < 4; i++) {
            int32_t tau = g_arm_status[i].torque;
            int32_t dtau = tau - arm_tau_prev[i];
            arm_tau_prev[i] = tau;
            int32_t dtau_abs = (dtau >= 0) ? dtau : -dtau;
            if (dtau_abs > SAFETY_COLLISION_TORQUE_DELTA && cur_mode != 0) {  /* 透明模式跳过 */
                push_can_tx(1, g_arm_status[i].joint_id, 0, CTRL_MODE_TORQUE);
                rs01_mit_disable(&g_can2_handle, g_arm_status[i].joint_id);
                SAFETY_LOCK();
                g_safety_state.fault_code |= FAULT_COLLISION;
                SAFETY_UNLOCK();
            }
        }
    }
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
            /* ★ v1.6.8fix2: ZERO_TORQUE 模式直接发零力矩, 跳过 rate_limit/compensate */
            if (mode_get_local_mode() == 0) {
                push_can_tx(1, joint_id, 0, CTRL_MODE_TORQUE);
            } else {
                target = torque_rate_limit(g_arm_status[i].torque, target,
                                          rate_limit ? rate_limit : 5000, CONTROL_PERIOD_ARM);
                target = torque_compensate(target, g_arm_status[i].velocity,
                                           g_arm_status[i].position, (uint8_t)(i + 2));
                if (tlimit > 0) {
                    if (target > tlimit) target = tlimit;
                    if (target < -tlimit) target = -tlimit;
                }
                push_can_tx(1, joint_id, target, CTRL_MODE_TORQUE);
            }
        } else if (mode == CTRL_MODE_POSITION) {
            int32_t vlimit = pp->velocity_limit;
            target = trapezoidal_interpolate(g_arm_status[i].position, target,
                                             vlimit ? vlimit : max_vel, CONTROL_PERIOD_ARM);
            push_can_tx(1, joint_id, target, CTRL_MODE_POSITION);
        }
    }

    /* v1.6.9: ABO 调试打印 (每秒 1 次), 验证助力链路 — P0 排查临时开启 */
    {
        static uint32_t abo_dbg_ts = 0;
        uint32_t now = HAL_GetTick();
        if (now - abo_dbg_ts >= 1000) {
            abo_dbg_ts = now;
            printf("[ABO] mode=%d gait_run=%d phi=%.2f global_en=%d\r\n",
                   mode_get_local_mode(), mode_get_gait_running(), g_gait_phase,
                   mode_get_abo_enabled());
            for (int i = 0; i < 6; i++) {
                if (g_abo_state[i].industrial_mode) {
                    printf("  J%d en=%d gain=%d load=%ld freeze=%d bias=%ld bp=%ld meas=%ld assist=%ld\r\n",
                           i, g_abo_state[i].enable, g_abo_state[i].assist_gain_q10,
                           (long)g_abo_state[i].load_est_q10, g_abo_state[i].load_freeze_cnt,
                           (long)g_abo_state[i].bias_est, (long)g_abo_state[i].bp_lpf_state,
                           (long)joint_status_ptr(i)->torque,
                           (long)g_abo_assist_torque[i]);
                } else {
                    printf("  J%d en=%d gain=%d alpha=%ld leak=%ld bias=%ld hpf=%ld meas=%ld assist=%ld\r\n",
                           i, g_abo_state[i].enable, g_abo_state[i].assist_gain_q10,
                           (long)g_abo_state[i].hpf_alpha_q16, (long)g_abo_state[i].bias_leak_q16,
                           (long)g_abo_state[i].bias_est, (long)g_abo_state[i].hpf_state,
                           (long)joint_status_ptr(i)->torque,
                           (long)g_abo_assist_torque[i]);
                }
            }
        }
    }

    ReportFrame_t frame;
    report_frame_build(&frame, g_leg_status, g_arm_status);
    report_fifo_write(&frame);

    HAL_GPIO_TogglePin(WDT_FEED_PORT, WDT_FEED_PIN);
}

/* ===== STANDALONE 模式本地控制接口实现 ===== */

/* v1.6.2: 旧接口包装, 转发到 mode_manager */
void local_set_mode(uint8_t mode)
{
    mode_request_switch((MainMode_e)mode);
}

uint8_t local_get_mode(void)
{
    return mode_get_local_mode();
}

void local_gait_start_stop(uint8_t start)
{
    mode_gait_start_stop(start);
}

uint8_t local_gait_is_running(void)
{
    return mode_get_gait_running();
}

void local_gait_reset_phase(void)
{
    g_local_left_hip = 0;
    g_local_right_hip = 0;
}

int32_t local_get_left_hip(void)
{
    return g_local_left_hip;
}

int32_t local_get_right_hip(void)
{
    return g_local_right_hip;
}

/* ====== ABO Observer 参数接口实现 ====== */

uint8_t abo_set_param(uint8_t joint_idx, uint16_t assist_gain_q10,
                       uint16_t hpf_alpha_q16, uint16_t bias_leak_q16,
                       uint8_t enable)
{
    if (joint_idx >= 6) return 0;

    ABOState_t *abo = &g_abo_state[joint_idx];

    /* 关中断保证原子性 (与 1ms ISR 并发安全) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    abo->assist_gain_q10 = assist_gain_q10;
    abo->hpf_alpha_q16   = hpf_alpha_q16;
    abo->bias_leak_q16   = bias_leak_q16;
    abo->enable           = enable;

    if (!primask) __enable_irq();

    return 1;
}

uint8_t abo_get_param(uint8_t joint_idx, ABOState_t *out)
{
    if (joint_idx >= 6 || out == NULL) return 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = g_abo_state[joint_idx];
    if (!primask) __enable_irq();

    return 1;
}

uint8_t abo_set_bias(uint8_t joint_idx, int32_t bias_est)
{
    if (joint_idx >= 6) return 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_abo_state[joint_idx].bias_est = bias_est;
    g_abo_state[joint_idx].hpf_state = 0;  /* 恢复偏置时清零 HPF 状态 */
    if (!primask) __enable_irq();

    return 1;
}

/* ====== lcd_status.c 需要的 ctrl_get_* 接口 (完整功能版实现) ====== */

static uint8_t g_selected_motor = 0;   /* LCD 显示选中的电机 (0~5) */

uint8_t ctrl_get_mode(void)
{
    /* 0=ZERO, 1=POS, 2=TORQ — 基于当前命令模式推断 */
    if (g_comm_mode == COMM_MODE_STANDALONE) {
        uint8_t m = mode_get_local_mode();
        if (m == 1) return 1;   /* GAIT=POS */
        if (m == 2) return 2;   /* ARM=TORQ */
        return 0;               /* ZERO */
    }
    /* HOST 模式: 取第 0 个关节的控制模式 */
    JointCommand_t tmp[6];
    if (safe_cmd_read(tmp)) {
        if (tmp[0].control_mode == CTRL_MODE_POSITION) return 1;
        if (tmp[0].control_mode == CTRL_MODE_TORQUE)   return 2;
    }
    return 0;
}

uint8_t ctrl_get_selected(void)       { return g_selected_motor; }
int32_t ctrl_get_target_pos(void)     { return g_leg_status[0].position; }   /* 用反馈代替目标 */
int32_t ctrl_get_target_torque(void)  { return g_leg_status[0].torque; }     /* 用反馈代替目标 */
uint8_t ctrl_get_pos_manual(void)     { return 0; }
