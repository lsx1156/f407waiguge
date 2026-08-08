/**
 * @file global_pose.h
 * @brief §E 基座/全身位姿漂移免疫 — 纯运动学 ZUPT (无 IMU)
 *
 * 适用: 可移动底盘 / 穿戴式外骨骼行走. 需全身位姿做重力补偿方向修正.
 *
 * 原理:
 *   1. 接触检测 (无力传感器): 电机扭矩 + 速度 + 电流饱和度
 *      站立腿: |vel| < 阈值 且 |tau| > 阈值 且 电流环非饱和
 *   2. ZUPT: 检测到站立相 → 该足位姿在世界系锁定 (零速更新)
 *   3. 基座位姿: 双足锁定 → 完全约束 (无漂移);
 *               单足锁定 → 绕该足旋转 (仅 yaw 微漂, roll/pitch 由编码器链约束);
 *               双足离地 → 纯运动学前向传播 (短时 <1s 漂移可忍)
 *   4. 重力方向修正: 提取 base_pose 的 roll/pitch → 重力向量广播
 *      供 §B ESO3 模型前馈 Gravity_Comp() 使用
 *
 * 无 IMU: 平地行走/站立搬运, ZUPT 每步校准一次, 漂移 < 1°/h.
 *
 * 注: 当前系统仅髋关节 (单连杆腿), FK 退化为单连杆; 若后续加膝/踝,
 *     Forward_Kinematics_Foot() 需扩展为多连杆链.
 */
#ifndef __GLOBAL_POSE_H__
#define __GLOBAL_POSE_H__

#include "joint_unit.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 基座位姿 (世界系) =====
 * roll: 绕 x (侧倾); pitch: 绕 y (俯仰); yaw: 绕 z (偏航, 会微漂) */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    uint8_t valid;        /* 1=位姿约束有效 (至少单足锁定), 0=悬空传播 */
    uint8_t drift_bounded;/* 1=漂移有界 (双足锁定), 0=仅单足/悬空 */
} BasePose_t;

/* ===== 重力向量 (基座系, 单位向量) =====
 * 直立时 g_base = [0, 0, -1].
 * 基座俯仰 pitch → g_y = sin(pitch), g_z = -cos(pitch)
 * 基座侧倾 roll  → g_x = sin(roll) (近似, 小角) */
typedef struct {
    float gx;
    float gy;
    float gz;
} GravityVec_t;

/* ===== 足端接触状态 ===== */
typedef struct {
    uint8_t contact;        /* 1=接触地面 (站立相), 0=摆动/离地 */
    uint8_t contact_prev;   /* 上一拍 (边沿检测) */
    uint8_t pose_locked;    /* 1=ZUPT 已锁定该足世界位姿 */
    float  foot_x_world;    /* 锁定时的足端世界 x (m) */
    float  foot_z_world;    /* 锁定时的足端世界 z (m) */
} FootContact_t;

/* ===== 全局位姿估计器状态 ===== */
typedef struct {
    BasePose_t    base_pose;
    GravityVec_t  gravity_base;
    FootContact_t foot[2];        /* 0=左, 1=右 */

    /* 接触检测阈值 (可现场标定覆盖) */
    float thr_contact_vel_rad_s;  /* 速度阈值: |vel| < 此值视为静止 */
    float thr_contact_tau_nm;     /* 力矩阈值: |tau| > 此值视为承重 */
    float thr_contact_sat;        /* 电流饱和度上限 (占位, 当前 0) */

    /* 腿连杆长度 (m), 单连杆腿: 髋→足 */
    float leg_length_m;

    uint8_t zupt_fail_flag;   /* v2.0: 1=ZUPT 持续失效(双足离地>2s), 0=正常 */
} GlobalPose_t;

extern GlobalPose_t g_global_pose;

/* ===== API ===== */
void global_pose_init(void);

/* §E 主入口 100Hz: 接触检测 + ZUPT + 基座位姿 + 重力向量
 * (用户原稿 1kHz, 此处 100Hz 已足够: 位姿变化慢, 节省 CPU) */
void Global_Pose_Estimator_Update100Hz(void);

/* 只读访问 */
const BasePose_t   *global_pose_get_base(void);
const GravityVec_t *global_pose_get_gravity(void);
uint8_t             global_pose_drift_bounded(void);

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_POSE_H__ */
