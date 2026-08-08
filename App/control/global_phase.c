/**
 * @file global_phase.c
 * @brief §D 相位漂移终结 — 运动学相位 + 任务状态机
 *
 * 模式识别规则树 (无 IMU, 仅编码器/力矩/§C 末端力):
 *   1. 双腿周期性摆动 (速度交替过零 + 步态运行标志)  → PHASE_GAIT
 *   2. 双腿同步屈伸 (同向速度 + 髋角大幅变化)         → PHASE_SQUAT
 *   3. 末端 |F| > 阈值 (§C 已解算)                    → PHASE_CARRY
 *   4. 否则                                            → PHASE_FREE
 *
 * 相位/导纳映射:
 *   GAIT  : 复用 g_gait_phase, 按相位调度刚度 (站立相高 K, 摆动相低 K)
 *   SQUAT : s = (hip - hip_min)/(hip_max - hip_min), 蹲底高阻尼/站立高刚度
 *   CARRY : 按负载质量调度 (重载高阻尼低刚度, 轻载反之)
 *   FREE  : K=0, B=small (透明)
 */
#include "global_phase.h"
#include "control_isr.h"   /* g_gait_phase, mode_get_gait_running */
#include "mode_manager.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#define MDEG_TO_RAD_PH   (1.7453292519943295e-05f)

/* 模式切换去抖: 候选需持续 N 拍 (100Hz × N ms) 才切换 */
#define MODE_HOLD_THRESH     5u     /* 50 ms 去抖, 避免抖动 */

/* 模式识别阈值 */
#define GAIT_VEL_THRESH_MDEG_S   20000.0f   /* 20°/s 视为腿在动 */
#define SQUAT_VEL_THRESH_MDEG_S  5000.0f    /* 5°/s 蹲起速度下限 */
#define SQUAT_RANGE_THRESH_RAD   0.15f      /* 髋角变化 > 8.6° 视为蹲起 */
#define CARRY_FORCE_THRESH_N     30.0f      /* 末端力 > 30N 视为搬运 */

/* 蹲起髋角默认范围 (现场标定覆盖): 0° ~ 90° */
#define HIP_MIN_RAD_DEFAULT   0.0f
#define HIP_MAX_RAD_DEFAULT   1.5707963f

TaskPhase_t g_task_phase;

/* P2: 相位→导纳缩放 默认配置表 (取代 map_phase_to_admittance 内的硬编码魔数)
 * 集中可调, 与原魔数值一一对应, 行为保持完全一致. */
const PhaseAdmConfig_t g_phase_adm_config = {
    1.2f,   /* leg_gait_stance   站立相 adm */
    0.6f,   /* leg_gait_swing    摆动相 adm */
    1.0f,   /* leg_squat_top     站立顶基础 adm */
    0.5f,   /* leg_squat_depth_k 蹲底增量系数 */
    0.3f,   /* leg_free          腿透明 adm */
    0.7f,   /* arm_carry_base    搬运基础 adm */
    0.2f,   /* arm_carry_load_k  搬运负载系数 */
    0.3f,   /* arm_free          臂透明 adm */
};

/* ===== 辅助 ===== */
static inline float clampf_ph(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float map_range(float v, float lo, float hi) {
    if (hi <= lo) return 0.0f;
    return clampf_ph((v - lo) / (hi - lo), 0.0f, 1.0f);
}

/* ===== 模式识别 (P1-4: 腿/臂正交拆分) =====
 *   detect_leg_mode: GAIT(步态运行+双腿交替) / SQUAT(双腿同向屈伸) / FREE(否则)
 *   detect_arm_mode: CARRY(任一臂末端力>30N) / FREE(否则)
 *   原 detect_mode 的 GAIT>SQUAT>CARRY>FREE 优先级拆为两条独立轨道,
 *   行走拿重物时腿=GAIT, 臂=CARRY, 互不覆盖. */
static PhaseMode_e detect_leg_mode(void)
{
    JointStatus_t *ls = joint_status_ptr(0);
    JointStatus_t *rs = joint_status_ptr(1);
    if (!ls || !rs) return PHASE_FREE;

    float vl = fabsf((float)ls->velocity);   /* mdeg/s */
    float vr = fabsf((float)rs->velocity);
    float ql = (float)ls->position * MDEG_TO_RAD_PH;
    float qr = (float)rs->position * MDEG_TO_RAD_PH;

    /* 1. GAIT: 步态运行标志 + 至少一条腿在动 + 左右腿速度不同步 (交替) */
    if (mode_get_gait_running() && (vl + vr > GAIT_VEL_THRESH_MDEG_S)) {
        /* 交替判据: 两腿速度符号相反 (一摆一撑) */
        float vls = (float)ls->velocity;
        float vrs = (float)rs->velocity;
        if ((vls >  SQUAT_VEL_THRESH_MDEG_S && vrs < -SQUAT_VEL_THRESH_MDEG_S) ||
            (vls < -SQUAT_VEL_THRESH_MDEG_S && vrs >  SQUAT_VEL_THRESH_MDEG_S)) {
            return PHASE_GAIT;
        }
    }

    /* 2. SQUAT: 双腿同向运动 + 髋角变化范围大 */
    {
        float vls = (float)ls->velocity;
        float vrs = (float)rs->velocity;
        uint8_t same_dir =
            ((vls >  SQUAT_VEL_THRESH_MDEG_S) && (vrs >  SQUAT_VEL_THRESH_MDEG_S)) ||
            ((vls < -SQUAT_VEL_THRESH_MDEG_S) && (vrs < -SQUAT_VEL_THRESH_MDEG_S));
        /* 髋角本身偏离中位也作为蹲起证据 */
        uint8_t hip_bent = (fabsf(ql) > SQUAT_RANGE_THRESH_RAD ||
                            fabsf(qr) > SQUAT_RANGE_THRESH_RAD);
        if (same_dir && hip_bent) return PHASE_SQUAT;
    }

    /* 3. FREE */
    return PHASE_FREE;
}

static PhaseMode_e detect_arm_mode(void)
{
    /* CARRY: 任一臂末端力 > 阈值 (§C 已解算) */
    const EndEffectorForce_t *fL = global_get_arm_force(0);
    const EndEffectorForce_t *fR = global_get_arm_force(1);
    float fL_n = fL ? sqrtf(fL->fx*fL->fx + fL->fy*fL->fy) : 0.0f;
    float fR_n = fR ? sqrtf(fR->fx*fR->fx + fR->fy*fR->fy) : 0.0f;
    if (fL_n > CARRY_FORCE_THRESH_N || fR_n > CARRY_FORCE_THRESH_N) {
        return PHASE_CARRY;
    }
    return PHASE_FREE;
}

/* ===== 模式切换去抖 (主/腿模式) ===== */
static PhaseMode_e mode_switch_debounce(PhaseMode_e detected)
{
    TaskPhase_t *tp = &g_task_phase;
    if (detected == tp->mode_candidate) {
        if (tp->mode_hold_cnt < 0xFFFFu) tp->mode_hold_cnt++;
        if (tp->mode_hold_cnt >= MODE_HOLD_THRESH && detected != tp->mode) {
            tp->mode_prev = tp->mode;
            tp->mode = detected;
        }
    } else {
        tp->mode_candidate = detected;
        tp->mode_hold_cnt = 0u;
    }
    return tp->mode;
}

/* P1-4: 臂模式切换去抖 (独立计数, 与腿模式互不干扰) */
static PhaseMode_e arm_mode_switch_debounce(PhaseMode_e detected)
{
    TaskPhase_t *tp = &g_task_phase;
    if (detected == tp->arm_mode_candidate) {
        if (tp->arm_mode_hold_cnt < 0xFFFFu) tp->arm_mode_hold_cnt++;
        if (tp->arm_mode_hold_cnt >= MODE_HOLD_THRESH && detected != tp->arm_mode) {
            tp->arm_mode = detected;
        }
    } else {
        tp->arm_mode_candidate = detected;
        tp->arm_mode_hold_cnt = 0u;
    }
    return tp->arm_mode;
}

/* ===== 搬运子状态机 (IDLE→GRASP→LIFT→CARRY→RELEASE) ===== */
static void carry_substate_update(void)
{
    TaskPhase_t *tp = &g_task_phase;
    const EndEffectorForce_t *fL = global_get_arm_force(0);
    const EndEffectorForce_t *fR = global_get_arm_force(1);
    float fmax = 0.0f;
    if (fL) { float n = sqrtf(fL->fx*fL->fx + fL->fy*fL->fy); if (n > fmax) fmax = n; }
    if (fR) { float n = sqrtf(fR->fx*fR->fx + fR->fy*fR->fy); if (n > fmax) fmax = n; }

    /* 阈值滞回: 进入 LIFT 需 > 40N, 退出到 IDLE 需 < 15N */
    switch (tp->carry_sub) {
    case CARRY_IDLE:
        if (fmax > 25.0f) tp->carry_sub = CARRY_GRASP;
        break;
    case CARRY_GRASP:
        if (fmax > 40.0f)      tp->carry_sub = CARRY_LIFT;
        else if (fmax < 15.0f) tp->carry_sub = CARRY_IDLE;
        break;
    case CARRY_LIFT:
        tp->carry_sub = CARRY_CARRY;   /* 稳定后直接转持续 */
        break;
    case CARRY_CARRY:
        if (fmax < 30.0f) tp->carry_sub = CARRY_RELEASE;
        break;
    case CARRY_RELEASE:
        if (fmax < 15.0f)      tp->carry_sub = CARRY_IDLE;
        else if (fmax > 40.0f) tp->carry_sub = CARRY_CARRY;
        break;
    default:
        tp->carry_sub = CARRY_IDLE;
        break;
    }
}

/* ===== 相位→导纳映射 (P1-4: 腿/臂正交; P2: 魔数改配表) =====
 * adm_scale: 1.0=默认; <1.0 降刚度/阻尼(透明); >1.0 增阻尼(稳)
 *   腿 idx 0,1 → 按 leg_mode (GAIT/SQUAT/FREE) 调度
 *   臂 idx 2,3,4,5 → 按 arm_mode (CARRY/FREE) 调度
 *   保留原 GAIT/SQUAT/CARRY/FREE 四种映射逻辑, 分别应用到对应关节组.
 *   P2: 各魔数改为读取 g_phase_adm_config, 逻辑完全一致. */
static void map_phase_to_admittance(void)
{
    TaskPhase_t *tp = &g_task_phase;
    const PhaseAdmConfig_t *cfg = &g_phase_adm_config;   /* P2: 配表 */
    float s = tp->phase_val;

    /* --- 腿 (idx 0,1): 按 leg_mode 调度 --- */
    float leg_scale;
    switch (tp->leg_mode) {
    case PHASE_GAIT:
        /* s∈[0,1): 0~0.5 站立相 (高刚度 K↑), 0.5~1.0 摆动相 (低刚度 K↓)
         *   站立相 adm=cfg.leg_gait_stance (稳), 摆动相 adm=cfg.leg_gait_swing (柔) */
        leg_scale = (s < 0.5f) ? cfg->leg_gait_stance : cfg->leg_gait_swing;
        break;
    case PHASE_SQUAT:
        /* s=0 站立顶 (高刚度), s=1 蹲底 (高阻尼防跌): adm = top + depth_k * s */
        leg_scale = cfg->leg_squat_top + cfg->leg_squat_depth_k * s;
        break;
    case PHASE_FREE:
    default:
        /* 透明: K=0, B=small → adm=cfg.leg_free (柔) */
        leg_scale = cfg->leg_free;
        break;
    }
    tp->adm_scale[0] = leg_scale;   /* L-Hip */
    tp->adm_scale[1] = leg_scale;   /* R-Hip */

    /* --- 臂 (idx 2,3,4,5): 按 arm_mode 调度 --- */
    float arm_scale;
    switch (tp->arm_mode) {
    case PHASE_CARRY: {
        /* 按负载调度: 重载高阻尼低刚度 (稳), 轻载低阻尼高刚度 (响应)
         *   臂: adm = cfg.arm_carry_base + cfg.arm_carry_load_k * load_factor (搬运稳为主) */
        const EndEffectorForce_t *fL = global_get_arm_force(0);
        const EndEffectorForce_t *fR = global_get_arm_force(1);
        float fmax = 0.0f;
        if (fL) { float n = sqrtf(fL->fx*fL->fx + fL->fy*fL->fy); if (n > fmax) fmax = n; }
        if (fR) { float n = sqrtf(fR->fx*fR->fx + fR->fy*fR->fy); if (n > fmax) fmax = n; }
        float load_factor = clampf_ph(fmax / 100.0f, 0.0f, 1.5f);
        arm_scale = cfg->arm_carry_base + cfg->arm_carry_load_k * load_factor;
        break;
    }
    case PHASE_FREE:
    default:
        /* 透明: K=0, B=small → adm=cfg.arm_free (柔) */
        arm_scale = cfg->arm_free;
        break;
    }
    tp->adm_scale[2] = arm_scale;   /* L-Shldr */
    tp->adm_scale[3] = arm_scale;   /* L-Elbow */
    tp->adm_scale[4] = arm_scale;   /* R-Shldr */
    tp->adm_scale[5] = arm_scale;   /* R-Elbow */
}

/* ===== §D 主入口 100Hz ===== */
void Global_PhaseScheduler_Update100Hz(void)
{
    TaskPhase_t *tp = &g_task_phase;

    /* 1. 模式识别 + 去抖 (P1-4: 腿/臂正交) */
    PhaseMode_e leg_detected = detect_leg_mode();
    PhaseMode_e arm_detected = detect_arm_mode();
    mode_switch_debounce(leg_detected);        /* 更新 tp->mode (主模式) */
    tp->leg_mode = tp->mode;                    /* 腿模式 = 主模式 (向后兼容) */
    arm_mode_switch_debounce(arm_detected);     /* 更新 tp->arm_mode (独立) */

    /* 2. 计算相位/子状态 (按腿模式驱动相位变量; 臂搬运子状态机独立) */
    switch (tp->leg_mode) {
    case PHASE_GAIT:
        /* 复用 control_isr.c 的 AO 步态相位 (0.0~1.0) */
        tp->phase_val = g_gait_phase;
        break;
    case PHASE_SQUAT: {
        /* 单调映射: 平均髋角 → [0,1] */
        JointStatus_t *ls = joint_status_ptr(0);
        JointStatus_t *rs = joint_status_ptr(1);
        if (ls && rs) {
            float qavg = 0.5f * ((float)ls->position + (float)rs->position) * MDEG_TO_RAD_PH;
            tp->phase_val = map_range(qavg, tp->hip_min_rad, tp->hip_max_rad);
        }
        break;
    }
    case PHASE_FREE:
    default:
        tp->phase_val = 0.0f;
        break;
    }

    /* 臂搬运子状态机 (独立于腿, 仅 arm_mode=CARRY 时推进) */
    if (tp->arm_mode == PHASE_CARRY) {
        carry_substate_update();
    }

    /* 3. 相位 → 导纳缩放 (腿按 leg_mode, 臂按 arm_mode) */
    map_phase_to_admittance();

    /* 4. 广播下发 adm_scale → JointUnitState.global_adm_scale */
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        JointUnitState_t *st = joint_runtime_ptr(i);
        if (st) st->global_adm_scale = tp->adm_scale[i];
    }
}

/* ===== 初始化 ===== */
void global_phase_init(void)
{
    memset(&g_task_phase, 0, sizeof(g_task_phase));
    g_task_phase.mode = PHASE_FREE;
    g_task_phase.mode_candidate = PHASE_FREE;
    g_task_phase.carry_sub = CARRY_IDLE;
    g_task_phase.hip_min_rad = HIP_MIN_RAD_DEFAULT;
    g_task_phase.hip_max_rad = HIP_MAX_RAD_DEFAULT;
    /* P1-4: 腿/臂正交模式初始化 */
    g_task_phase.leg_mode = PHASE_FREE;
    g_task_phase.arm_mode = PHASE_FREE;
    g_task_phase.arm_mode_candidate = PHASE_FREE;
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        g_task_phase.adm_scale[i] = 1.0f;
    }
    printf("[PHASE] §D scheduler ON: GAIT/SQUAT/CARRY/FREE dual-track (100Hz)\r\n");
}

const TaskPhase_t *global_phase_get(void) { return &g_task_phase; }
PhaseMode_e        global_phase_mode(void) { return g_task_phase.mode; }
