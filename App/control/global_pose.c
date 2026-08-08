/**
 * @file global_pose.c
 * @brief §E 基座/全身位姿漂移免疫 — 纯运动学 ZUPT (无 IMU)
 *
 * 数据流:
 *   关节反馈 (q, vel, tau) → 接触检测 → ZUPT 锁足 → 基座位姿解算
 *   → 重力向量 (基座系) → 广播 (供 §B Gravity_Comp 修正)
 *
 * 简化 (当前硬件: 仅髋关节, 单连杆腿):
 *   足端 FK (相对髋): foot_x = L·sin(q), foot_z = -L·cos(q)
 *   双足锁定: base_yaw = atan2(foot_R - foot_L); base_pitch/roll 由髋角推
 *   单足锁定: 基座绕该足旋转, yaw 由髋角差分推 (会微漂, ZUPT 每步校准)
 */
#include "global_pose.h"
#include "global_coordinator.h"   /* P1-5: g_global_coord.joint_payload_torque (腿接触阈值自适应) */
#include <string.h>
#include <math.h>
#include <stdio.h>

#define MDEG_TO_RAD_PS   (1.7453292519943295e-05f)
#define MNM_TO_NM_PS     (0.001f)

/* v2.0: ZUPT 持续失效判定阈值 — 双足离地超过此时长才报 WARN, 避免正常摆动误报 */
#define ZUPT_FAIL_TIMEOUT_MS  2000u

GlobalPose_t g_global_pose;

/* v2.0: ZUPT 失效检测的文件静态状态 (100Hz 维护, 1kHz safety_check_algorithm_faults 只读)
 *   s_zupt_invalid_start_ms: 本轮持续离地的起始时刻 (0=未在计时)
 *   s_zupt_ever_valid:       引导守卫, 首次有效前不计时, 避免上电未站立误报 */
static uint32_t s_zupt_invalid_start_ms = 0u;
static uint8_t  s_zupt_ever_valid = 0u;

/* ===== 辅助 ===== */
static inline float clampf_ps(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ===== 单连杆腿足端正运动学 (相对髋关节, 二维平面) =====
 * q: 髋角 (rad), 0=腿竖直向下; 正方向=屈髋 (腿前摆)
 * 返回: foot_x (前向, m), foot_z (垂直, 向下为负) */
static void foot_fk_single(float q_rad, float L_m, float *fx, float *fz)
{
    *fx =  L_m * sinf(q_rad);
    *fz = -L_m * cosf(q_rad);
}

/* ===== 接触检测 (无力传感器) =====
 * 判据: |vel| < 阈值 (静止) 且 |tau_clean| > 阈值 (承重) 且 电流不饱和
 *   站立相: 腿承重 → 力矩大; 摆动相: 腿离地 → 力矩小、速度大 */
static uint8_t detect_stance(uint8_t leg_idx)
{
    JointStatus_t      *fb = joint_status_ptr(leg_idx);
    JointUnitState_t   *st = joint_runtime_ptr(leg_idx);
    if (!fb || !st) return 0;

    float vel = fabsf((float)fb->velocity * MDEG_TO_RAD_PS);
    /* 用去零漂后的 tau (来自 §A), 更准 */
    float tau = fabsf(st->tau_meas_nm);

    GlobalPose_t *gp = &g_global_pose;
    /* P1-5: 接触力矩阈值自适应. 承重越大所需触发力矩越高, 避免重载站立时
     *   阈值过低误判 / 轻载摆动时阈值过高漏判.
     *   thr = 0.5Nm (基值) + 0.02 × |关节负载力矩| (§C 输出, 腿 idx 0,1).
     *   joint_payload_torque 来自 §C global_coordinator.leg_payload_approx. */
    float thr = 0.5f + 0.02f * fabsf(g_global_coord.joint_payload_torque[leg_idx]);
    return (vel < gp->thr_contact_vel_rad_s) &&
           (tau > thr);
    /* saturation 占位: RobStride 01 当前反馈不带电流饱和度, 暂不判 */
}

/* ===== §E 主入口 100Hz ===== */
void Global_Pose_Estimator_Update100Hz(void)
{
    GlobalPose_t *gp = &g_global_pose;

    /* 1. 双腿接触检测 */
    uint8_t left_contact  = detect_stance(0);
    uint8_t right_contact = detect_stance(1);

    /* ⚠️ 固定基座/站立不动假设 (P0-5):
     *   当前 foot_world 直接用相对髋的 FK (假设髋≈基座原点).
     *   仅适用于: 固定基座设备 / 穿戴者站立不动 / 平地缓慢行走(短时).
     *   不适用于: AGV 移动底盘 + 大范围移动. 后者需引入 base_pose 迭代:
     *     foot_world = base_pose + R(base_yaw) · FK(q)
     *   待 P2 阶段实现 full odometry. */
    /* 2. ZUPT: 上升沿 (刚接触) → 锁定该足世界位姿
     *   P0-5 守卫: 仅基座静止时允许更新锁足点, 避免移动中锁错点.
     *     基座静止判据: 冷启动 (base_pose 未有效, 允许首次锁定 bootstrap)
     *                   或 双足锁定 (drift_bounded=1, 基座完全约束).
     *     单足锁定 (valid && !drift_bounded) 时基座绕单足旋转 → 跳过, 保持
     *     pose_locked=0, 避免在移动中用 FK 相对髋的假设锁错世界位姿. */
    uint8_t base_stationary = (!gp->base_pose.valid) || gp->base_pose.drift_bounded;
    if (base_stationary) {
        if (left_contact && !gp->foot[0].contact_prev) {
            JointStatus_t *fb = joint_status_ptr(0);
            if (fb) {
                float q = (float)fb->position * MDEG_TO_RAD_PS;
                float fx, fz;
                foot_fk_single(q, gp->leg_length_m, &fx, &fz);
                /* 锁定时: 足端世界位姿 = 当前基座估计 + FK 偏移
                 * 简化: 假设基座在原点附近, 足世界 ≈ FK (相对基座) */
                gp->foot[0].foot_x_world = fx;
                gp->foot[0].foot_z_world = fz;
                gp->foot[0].pose_locked = 1u;
            }
        }
        if (right_contact && !gp->foot[1].contact_prev) {
            JointStatus_t *fb = joint_status_ptr(1);
            if (fb) {
                float q = (float)fb->position * MDEG_TO_RAD_PS;
                float fx, fz;
                foot_fk_single(q, gp->leg_length_m, &fx, &fz);
                gp->foot[1].foot_x_world = fx;
                gp->foot[1].foot_z_world = fz;
                gp->foot[1].pose_locked = 1u;
            }
        }
    }
    /* 离地时清除锁定 (避免用旧锁点) */
    if (!left_contact)  gp->foot[0].pose_locked = 0u;
    if (!right_contact) gp->foot[1].pose_locked = 0u;

    gp->foot[0].contact_prev = left_contact;
    gp->foot[1].contact_prev = right_contact;

    /* 3. 基座位姿解算 */
    JointStatus_t *ls = joint_status_ptr(0);
    JointStatus_t *rs = joint_status_ptr(1);
    float qL = ls ? (float)ls->position * MDEG_TO_RAD_PS : 0.0f;
    float qR = rs ? (float)rs->position * MDEG_TO_RAD_PS : 0.0f;

    if (gp->foot[0].pose_locked && gp->foot[1].pose_locked) {
        /* 双足锁定: 基座完全约束, 无漂移
         *   base_pitch ≈ 平均髋角 (双腿都屈 → 躯干前倾)
         *   base_roll  ≈ 髋角差 / 2 (单侧屈 → 侧倾)
         *   base_yaw   由双足横向偏移决定 (此处单连杆无横向, yaw 保持) */
        gp->base_pose.pitch_rad = 0.5f * (qL + qR);
        gp->base_pose.roll_rad  = 0.5f * (qL - qR);
        /* yaw 不更新 (双足都在 sagittal 平面, 无 yaw 信息) */
        gp->base_pose.valid        = 1u;
        gp->base_pose.drift_bounded = 1u;
    } else if (gp->foot[0].pose_locked || gp->foot[1].pose_locked) {
        /* 单足锁定: 基座绕该足旋转
         *   pitch/roll 由编码器链约束 (相对锁足), yaw 微漂 */
        uint8_t stance = gp->foot[0].pose_locked ? 0u : 1u;
        float q_stance = (stance == 0u) ? qL : qR;
        /* 基座俯仰 ≈ 支撑腿髋角 (支撑腿竖直 → 躯干竖直) */
        gp->base_pose.pitch_rad = q_stance;
        gp->base_pose.roll_rad  = 0.5f * (qL - qR);
        gp->base_pose.valid        = 1u;
        gp->base_pose.drift_bounded = 0u;   /* 单足: yaw 漂移未约束 */
    } else {
        /* 双足离地 (飞行相/搬运悬空): 纯运动学前向传播
         *   短时 (<1s) 漂移可忍, 不更新 pitch/roll (保持上一拍) */
        gp->base_pose.valid        = 0u;
        gp->base_pose.drift_bounded = 0u;
    }

    /* 4. 重力向量 (基座系) 广播
     *   直立: g = [0, 0, -1]
     *   基座俯仰 pitch → g_y = sin(pitch), g_z = -cos(pitch)
     *   基座侧倾 roll  → g_x = sin(roll) (小角近似, 忽略耦合) */
    float pitch = gp->base_pose.pitch_rad;
    float roll  = gp->base_pose.roll_rad;
    gp->gravity_base.gx =  sinf(roll);
    gp->gravity_base.gy =  sinf(pitch) * cosf(roll);
    gp->gravity_base.gz = -cosf(pitch) * cosf(roll);

    /* 钳位: 单位向量模长 ≈ 1 */
    float norm = sqrtf(gp->gravity_base.gx * gp->gravity_base.gx +
                       gp->gravity_base.gy * gp->gravity_base.gy +
                       gp->gravity_base.gz * gp->gravity_base.gz);
    if (norm > 1e-6f) {
        gp->gravity_base.gx /= norm;
        gp->gravity_base.gy /= norm;
        gp->gravity_base.gz /= norm;
    }

    /* v2.0: ZUPT 持续失效检测 (双足离地超过 2s → WARN 故障)
     *   首次有效前不计时(上电引导), 避免未站立时误报;
     *   一旦曾有效, 后续持续离地>2s 才判失效, 站立恢复即清除。 */
    if (gp->base_pose.valid) {
        s_zupt_ever_valid = 1u;
        s_zupt_invalid_start_ms = 0u;
        gp->zupt_fail_flag = 0u;
    } else if (s_zupt_ever_valid) {
        if (s_zupt_invalid_start_ms == 0u) s_zupt_invalid_start_ms = HAL_GetTick();
        if ((HAL_GetTick() - s_zupt_invalid_start_ms) > ZUPT_FAIL_TIMEOUT_MS) {
            gp->zupt_fail_flag = 1u;
        }
    }
}

/* ===== 初始化 ===== */
void global_pose_init(void)
{
    memset(&g_global_pose, 0, sizeof(g_global_pose));
    /* 接触检测默认阈值 (现场标定覆盖) */
    g_global_pose.thr_contact_vel_rad_s = 0.05f;   /* ~2.9 °/s 视为静止 */
    g_global_pose.thr_contact_tau_nm    = 1.0f;    /* 1 Nm 视为承重 */
    g_global_pose.thr_contact_sat       = 0.5f;
    g_global_pose.leg_length_m          = 0.70f;   /* 髋→足 ~70cm (现场标定) */
    /* 直立重力向量 */
    g_global_pose.gravity_base.gz = -1.0f;
    g_global_pose.base_pose.valid = 0u;
    /* v2.0: ZUPT 失效标志显式清零 (memset 已清, 此行注释更清晰) + 重置引导守卫 */
    g_global_pose.zupt_fail_flag = 0u;
    s_zupt_ever_valid = 0u;
    s_zupt_invalid_start_ms = 0u;
    printf("[POSE] §E ZUPT pose estimator ON: contact-detect + dual-foot lock (100Hz)\r\n");
}

const BasePose_t   *global_pose_get_base(void)     { return &g_global_pose.base_pose; }
const GravityVec_t *global_pose_get_gravity(void)  { return &g_global_pose.gravity_base; }
uint8_t             global_pose_drift_bounded(void){ return g_global_pose.base_pose.drift_bounded; }
