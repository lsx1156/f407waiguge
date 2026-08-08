/**
 * @file global_coordinator.c
 * @brief 全局协调单例实现 — 顶层协同
 *
 * §C  Global_LoadEstimator_Update100Hz
 *   输入:   6 关节的反馈 (q, tau_meas_clean)
 *   模型:   工业搬运准静态 (acc≈0, vel≈0)
 *           tau_meas - G(q) = J(q)^T · F_ext
 *   解算:   F_ext = (J^T)^+ · (tau_meas - G(q))   (2x1 伪逆 / 闭式 2-DOF)
 *   输出:   per-joint payload_tau = J^T_col(i) · F_ext
 *           → 调用 joint_set_payload_est(i, ...) 下发
 *
 * 简化与约束:
 *   - 无 IMU 无额外传感器. 纯编码器绝对角度 + 电流力矩.
 *   - 双臂独立 (各 2-DOF, 肩+肘 平面). 腿不参与外力估计, 重力偏置保留 ABO 吸收.
 *   - 闭式雅可比 + 伪逆, 100Hz, 轻量浮点, H7/RT1062/F407(FPU) 均能跑.
 *   - 若模型不准: ESO3 自适应 beta3 + 残差吸收兜底, 不爆炸.
 */
#include "global_coordinator.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ===== 常量 ===== */
#define MDEG_TO_RAD_GC     (1.7453292519943295e-05f)
#define MNM_TO_NM_GC       (0.001f)
#define NM_TO_MNM_GC       (1000.0f)

/* 机械臂尺寸参考 (保守估计, 现场标定后应覆盖)
 * L1: 上臂 ~ 0.30 m, L2: 前臂 ~ 0.30 m — 未用时作为 2D 平面参数 */
#define ARM_L1_M            0.30f
#define ARM_L2_M            0.30f

/* 负载突变检测: |ΔF| > 阈值 (牛顿)
 * 20kg 工件 ~ 200 N, 50N 是合理的抓取/放下判别下限 */
#define PAYLOAD_DELTA_F_N   50.0f
#define PAYLOAD_DEBOUNCE_MS 5      /* 100Hz × 5 = 50 ms 去抖 */

GlobalCoordinator_t g_global_coord;

/* ===== 辅助 ===== */
static inline float gc_clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ===== 单臂 2-DOF 重力模型 (准静态) =====
 * side: 0=左臂 (idx 2=肩, idx 3=肘)
 *       1=右臂 (idx 4=肩, idx 5=肘)
 * 输出: G_tau[0]=肩关节重力扭矩 (Nm), G_tau[1]=肘
 *
 * 参数化: 肩 MGL1, 肘 MGL2 — 来自 JointUnitState.params.mgl
 *         theta0 由每关节 zero-cal 覆盖 */
static void arm_gravity_model(uint8_t side, float G_tau[2])
{
    uint8_t j_shld = (side == 0) ? 2u : 4u;
    uint8_t j_elbw = j_shld + 1u;
    JointUnitState_t *st_s = joint_runtime_ptr(j_shld);
    JointUnitState_t *st_e = joint_runtime_ptr(j_elbw);
    if (!st_s || !st_e) { G_tau[0] = G_tau[1] = 0.0f; return; }

    float q_s = st_s->pos_rad + st_s->params.theta0_rad;  /* 肩绝对角 (rad) */
    float q_e = st_e->pos_rad + st_e->params.theta0_rad;  /* 肘绝对角 (rad) */
    float mgl_s = st_s->params.mgl;
    float mgl_e = st_e->params.mgl;

    /* 肩: 同时扛上臂 + 前臂重量投影
     *   G_s = mgl1*cos(q_s) + mgl2*cos(q_s + q_e)
     * 肘: 仅前臂
     *   G_e = mgl2*cos(q_s + q_e) */
    G_tau[0] = mgl_s * cosf(q_s) + mgl_e * cosf(q_s + q_e);
    G_tau[1] = mgl_e * cosf(q_s + q_e);
}

/* ===== 单臂 2-DOF 雅可比转置 J^T (平面, 末端力→关节力矩) =====
 * τ = J^T · [fx, fy]^T
 *   J^T = [  -L1·sq - L2·sqe   -L2·sqe ]   ← τ_s
 *         [   L1·cq + L2·cqe    L2·cqe ]   ← τ_e
 *         (注意: 不同 DH 定义符号有差异, 这里用常见的 q1=肩绝对角, q2=肘相对角)
 * 本处按「肘 q_e 为相对肩夹角」处理. 若当前 RS01 报告的是绝对角,
 * J 中 q_e 项需改为「绝对-肩」差; 先用保守近似: 肘相对 = pos_elb - pos_shld. */
static void arm_jacobian_T(uint8_t side, float q1_rad, float q2_rel_rad, float JT[2][2])
{
    (void)side;
    float sq  = sinf(q1_rad);
    float cq  = cosf(q1_rad);
    float sqe = sinf(q1_rad + q2_rel_rad);
    float cqe = cosf(q1_rad + q2_rel_rad);

    /* row 0: shoulder joint */
    JT[0][0] = -ARM_L1_M * sq  - ARM_L2_M * sqe;
    JT[0][1] =  ARM_L1_M * cq  + ARM_L2_M * cqe;
    /* row 1: elbow joint */
    JT[1][0] = -ARM_L2_M * sqe;
    JT[1][1] =  ARM_L2_M * cqe;
}

/* ===== (J^T)^+ 伪逆 (2x2 方阵可逆 → 真逆) =====
 * det = JT[0][0]·JT[1][1] - JT[0][1]·JT[1][0]
 * 如果奇异 (|det| < eps): 退化为对角阻尼近似 1/k·I */
static void pinv2x2(const float A[2][2], float Ai[2][2])
{
    float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    const float eps = 1e-5f;
    if (fabsf(det) < eps) {
        /* 奇异: 退化为 对角阻尼伪逆, 避免 0 除 */
        Ai[0][0] = 1.0f; Ai[0][1] = 0.0f;
        Ai[1][0] = 0.0f; Ai[1][1] = 1.0f;
        return;
    }
    float inv_det = 1.0f / det;
    Ai[0][0] =  A[1][1] * inv_det;
    Ai[0][1] = -A[0][1] * inv_det;
    Ai[1][0] = -A[1][0] * inv_det;
    Ai[1][1] =  A[0][0] * inv_det;
}

/* ===== 单臂完整解算 (2-DOF) =====
 * side: 0=L, 1=R. 填充 f_ext[side] 和 joint_payload_torque[肩/肘]. */
static void arm_load_solve(uint8_t side)
{
    uint8_t j_shld = (side == 0) ? 2u : 4u;
    uint8_t j_elbw = j_shld + 1u;

    JointStatus_t *fb_s = joint_status_ptr(j_shld);
    JointStatus_t *fb_e = joint_status_ptr(j_elbw);
    JointUnitState_t *st_s = joint_runtime_ptr(j_shld);
    JointUnitState_t *st_e = joint_runtime_ptr(j_elbw);
    if (!fb_s || !fb_e || !st_s || !st_e) return;

    /* 1. 关节空间重力模型 G(q) */
    float G_tau[2];
    arm_gravity_model(side, G_tau);

    /* 2. 取去零漂后的 tau_meas (来自 §A) */
    float tau_s = st_s->tau_meas_nm;
    float tau_e = st_e->tau_meas_nm;

    /* 3. tau_meas - G(q) → 外力产生的等效扭矩 τ_ext */
    float tau_ext[2];
    tau_ext[0] = tau_s - G_tau[0];
    tau_ext[1] = tau_e - G_tau[1];

    /* 4. 读取关节角 → 计算 J^T → 检测奇异 → 求 (J^T)^+ → 解 F_ext */
    float q1 = st_s->pos_rad + st_s->params.theta0_rad;
    float q2 = (st_e->pos_rad + st_e->params.theta0_rad) - q1;   /* 肘相对角 (近似) */

    float JT[2][2];
    arm_jacobian_T(side, q1, q2, JT);

    EndEffectorForce_t *f = &g_global_coord.f_ext_arm[side];

    /* P0-3: 雅可比奇异兜底. 2x2 行列式 |det| < 阈值时雅可比退化,
     *   旧实现 pinv2x2 退化为单位阵 → 把 tau_ext 直接当 F_ext 输出, 负载野跳.
     *   修复: 奇异时不计算新 F_ext, 保持上一拍有效值, 不下发 payload, 仅递增计数. */
    float det = JT[0][0]*JT[1][1] - JT[0][1]*JT[1][0];
    const float JACOBIAN_SINGULAR_THRESH = 1e-3f;
    if (fabsf(det) < JACOBIAN_SINGULAR_THRESH) {
        g_global_coord.jacobian_singular[side] = 1u;
        if (g_global_coord.singular_cnt[side] < 0xFFFFFFFFu) {
            g_global_coord.singular_cnt[side]++;
        }
        /* 回退到上一拍有效 F_ext; joint_payload_torque 保持上一拍值 (不写新值) */
        *f = g_global_coord.f_last_valid[side];
        return;   /* 不调 joint_set_payload_est, 不触发 §C-boost */
    }
    g_global_coord.jacobian_singular[side] = 0u;

    float JT_inv[2][2];
    pinv2x2(JT, JT_inv);   /* det 已确认非奇异, 内部 eps 不会触发 */

    f->fx    = JT_inv[0][0] * tau_ext[0] + JT_inv[0][1] * tau_ext[1];
    f->fy    = JT_inv[1][0] * tau_ext[0] + JT_inv[1][1] * tau_ext[1];
    f->m_ext = 0.0f;

    /* 5. 物理合理钳位: 工业搬运单手末端力 ≤ 300N (约 30kg) */
    f->fx = gc_clampf(f->fx, -300.0f, 300.0f);
    f->fy = gc_clampf(f->fy, -300.0f, 300.0f);

    /* 6. 回映射 J^T·F_ext → 各关节 payload 扭矩
     *   这一步和步骤3 数值上应当 ≈ tau_ext, 但用 F_ext 钳位后重新回算更安全 */
    float pld_s = JT[0][0]*f->fx + JT[0][1]*f->fy;
    float pld_e = JT[1][0]*f->fx + JT[1][1]*f->fy;

    /* 7. 写入全局 joint_payload_torque[], 同时下发给单关节实例 */
    g_global_coord.joint_payload_torque[j_shld] = pld_s;
    g_global_coord.joint_payload_torque[j_elbw] = pld_e;
    joint_set_payload_est(j_shld, pld_s);
    joint_set_payload_est(j_elbw, pld_e);

    /* 保存本拍有效 F_ext, 供下一拍奇异时回退 */
    g_global_coord.f_last_valid[side] = *f;

    /* 8. 负载突变检测 (抓取/放下) → 触发 §C-boost */
    float fnorm = sqrtf(f->fx * f->fx + f->fy * f->fy);
    float dfn   = fabsf(fnorm - g_global_coord.f_prev_norm[side]);
    g_global_coord.f_prev_norm[side] = fnorm;
    if (dfn > PAYLOAD_DELTA_F_N) {
        g_global_coord.payload_change_cnt++;
        if (g_global_coord.payload_change_cnt >= PAYLOAD_DEBOUNCE_MS) {
            g_global_coord.payload_change_flag = 1u;
            g_global_coord.load_change_timer  = 500u;  /* 500 ms (100Hz 递减 10/拍) */
            /* §C-boost: 触发所有关节 ESO 快速收敛 (β3×3 + 遗忘旧 z3)
             *   持续 300ms, 1kHz 递减; 抓取/放下后负载阶跃, 旧 z3 失效需重收敛 */
            for (uint8_t i = 0; i < JOINT_COUNT; i++) {
                joint_trigger_eso_boost(i, 300u);
            }
        }
    }
}

/* ===== 腿部负载近似 (准静态, 由 ABO 偏置主承担) =====
 * 工业场景, 搬运时双腿主要抗地面反力. 我们用「双侧 tau_meas 与重力偏置之差」
 * 近似作 payload estimate, 避免雅可比; 若不准 ESO3 自适应兜底.
 *   payload ≈ (tau_meas - bias_est) / G_ratio
 * 保守近似: 直接写入 payload_est 的 20% 前馈, 不强行剥离全部. */
static void leg_payload_approx(void)
{
    for (uint8_t i = 0; i < 2; i++) {
        JointUnitState_t *st = joint_runtime_ptr(i);
        if (!st) continue;

        float tau_meas_nm = st->tau_meas_nm;
        float payload;
        if (st->eso_enable) {
            /* P0-4: ESO3 已剥离已知动力学, z3 含人力+未建模扰动;
             *   负载 ≈ tau_meas - z3 (即已建模部分含负载) */
            payload = tau_meas_nm - st->eso3.z3;
        } else {
            /* legacy: 20% 剥离 ABO bias (原逻辑) */
            ABOState_t *abo = joint_abo_ptr(i);
            if (!abo) continue;
            float bias_nm = (float)abo->bias_est * MNM_TO_NM_GC;
            payload = 0.2f * (tau_meas_nm - bias_nm);   /* 仅轻量剥离, 保守 */
        }

        payload = gc_clampf(payload, -40.0f, 40.0f);  /* 髋单关节 ≤ 40 N·m */
        g_global_coord.joint_payload_torque[i] = payload;
        joint_set_payload_est(i, payload);
    }
}

/* ===== 工况识别 (粗粒度, 为 §D 相位任务状态机占位) =====
 * 判据: 双腿速度 < 阈值 → 站立/搬运; 否则行走.
 * 手臂末端 |F| > 50N → 搬运.
 * 肩/肘接近 90° 弯 → 蹲起. */
static void op_mode_detect(void)
{
    JointStatus_t *ls = joint_status_ptr(0);
    JointStatus_t *rs = joint_status_ptr(1);
    if (!ls || !rs) return;

    float vleg = fabsf((float)ls->velocity) + fabsf((float)rs->velocity);  /* mdeg/s */
    uint8_t static_leg = (vleg < 5000.0f);  /* < 5°/s 视为静态 */

    float fnorm_L = sqrtf(g_global_coord.f_ext_arm[0].fx*g_global_coord.f_ext_arm[0].fx +
                          g_global_coord.f_ext_arm[0].fy*g_global_coord.f_ext_arm[0].fy);
    float fnorm_R = sqrtf(g_global_coord.f_ext_arm[1].fx*g_global_coord.f_ext_arm[1].fx +
                          g_global_coord.f_ext_arm[1].fy*g_global_coord.f_ext_arm[1].fy);
    uint8_t carrying = (fnorm_L > 50.0f || fnorm_R > 50.0f);

    if (!static_leg)             g_global_coord.op_mode = 3u;   /* 行走 */
    else if (carrying)           g_global_coord.op_mode = 2u;   /* 搬运 */
    else                         g_global_coord.op_mode = 1u;   /* 站立 */
}

/* ===== §C  顶层入口 100Hz ===== */
void Global_LoadEstimator_Update100Hz(void)
{
#if DRIFT_IMMUNE_ENABLE
    /* 双臂 (左臂 idx 2,3 / 右臂 idx 4,5) */
    arm_load_solve(0);
    arm_load_solve(1);

    /* 腿部近似 */
    leg_payload_approx();

    /* 工况识别 (占位, §D 会细化) */
    op_mode_detect();

    /* P1-3: §C load_change_timer 单位统一为 ms. 100Hz 每拍 10ms, 故减 10.
     *   500ms / 10 = 50 拍 × 10ms = 500ms, 与赋值 500u 语义一致.
     *   (eso_boost_timer 已是 ms、1kHz 递减 1, 不动) */
    if (g_global_coord.load_change_timer >= 10u) {
        g_global_coord.load_change_timer -= 10u;
    } else {
        g_global_coord.load_change_timer = 0u;
    }
    if (g_global_coord.load_change_timer == 0u) {
        g_global_coord.payload_change_flag = 0u;
    }
#endif /* DRIFT_IMMUNE_ENABLE */
}

/* ===== 初始化 ===== */
void global_coordinator_init(void)
{
    memset(&g_global_coord, 0, sizeof(g_global_coord));
    g_global_coord.op_mode = 0u;
#if DRIFT_IMMUNE_ENABLE
    printf("[GC] Global coordinator ON: §C full-body load estimator (100Hz, no-IMU)\r\n");
#else
    printf("[GC] Global coordinator: drift-immune disabled\r\n");
#endif
}

const EndEffectorForce_t *global_get_arm_force(uint8_t side)
{
    if (side > 1u) return NULL;
    return &g_global_coord.f_ext_arm[side];
}

float global_get_payload_torque(uint8_t joint_idx)
{
    if (joint_idx >= JOINT_COUNT) return 0.0f;
    return g_global_coord.joint_payload_torque[joint_idx];
}
