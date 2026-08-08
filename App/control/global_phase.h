/**
 * @file global_phase.h
 * @brief §D 相位漂移终结 — 运动学相位变量 + 任务状态机双轨制
 *
 * 核心思想:
 *   工业外骨骼 80% 工况非周期 (搬运/装配/蹲起/推拉).
 *   强行用 AO 锁相位是漂移/对抗主因.
 *
 *   行走  → PHASE_GAIT  : AO 锁相 (唯一强周期工况), 输出 φ∈[0,1)
 *   蹲起  → PHASE_SQUAT : 髋角单调映射 s∈[0,1], 零漂移
 *   搬运  → PHASE_CARRY : 离散任务状态机 IDLE→GRASP→LIFT→CARRY→RELEASE, 零积分
 *   自由  → PHASE_FREE  : 纯导纳/阻抗, K/B 固定或按负载调度
 *
 * 输出: 每关节 adm_scale (复合 K/B 缩放因子), 下发 JointUnitState.global_adm_scale
 *       供控制环 (数据流待定) 调度导纳/阻抗.
 *
 * 无 IMU: 模式识别仅用关节角度/速度/力矩 + §C 末端外力.
 */
#ifndef __GLOBAL_PHASE_H__
#define __GLOBAL_PHASE_H__

#include "joint_unit.h"
#include "global_coordinator.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 相位模式 ===== */
typedef enum {
    PHASE_GAIT  = 0,   /* 行走/跑步: AO 锁相 */
    PHASE_SQUAT = 1,   /* 蹲起/负重站立: 髋角单调映射 */
    PHASE_CARRY = 2,   /* 上肢搬运/推拉: 任务状态机 */
    PHASE_FREE  = 3,   /* 自由移动/微调: 透明导纳 */
} PhaseMode_e;

/* ===== 搬运任务子状态机 (PHASE_CARRY 内部) ===== */
typedef enum {
    CARRY_IDLE    = 0,   /* 空闲, 末端无力 */
    CARRY_GRASP   = 1,   /* 抓取过渡: 力矩上升 */
    CARRY_LIFT    = 2,   /* 提起: 末端力 > 阈值且上抬 */
    CARRY_CARRY   = 3,   /* 持续搬运: 力稳定 */
    CARRY_RELEASE = 4,   /* 放下过渡: 力矩下降 */
} CarrySubState_e;

/* ===== 任务相位结构 ===== */
typedef struct {
    PhaseMode_e      mode;                 /* 当前主模式 (= leg_mode, 向后兼容现有读取者) */
    CarrySubState_e  carry_sub;             /* PHASE_CARRY 子状态 */
    float            phase_val;             /* 归一化相位 [0,1], GAIT/SQUAT 用, CARRY/FREE=0 */
    float            adm_scale[JOINT_COUNT];/* 每关节导纳/阻抗缩放, 下发 JointUnitState */

    /* 模式切换去抖 (主/腿) */
    PhaseMode_e      mode_candidate;        /* 候选模式 */
    uint16_t         mode_hold_cnt;         /* 候选持续计数 (100Hz × N) */
    PhaseMode_e      mode_prev;             /* 上一拍模式 (边沿检测) */

    /* 蹲起相位缓存的髋角上下限 (现场标定覆盖) */
    float            hip_min_rad;
    float            hip_max_rad;

    /* P1-4: 腿/臂正交模式 (行走拿重物时臂应独立 CARRY, 不被腿 GAIT 覆盖)
     *   leg_mode ∈ {GAIT, SQUAT, FREE}  → 决定腿(idx 0,1) 的 adm_scale
     *   arm_mode ∈ {CARRY, FREE}        → 决定臂(idx 2,3,4,5) 的 adm_scale
     *   mode 字段保留为 "主模式" = leg_mode, 向后兼容. */
    PhaseMode_e      leg_mode;             /* 腿模式 */
    PhaseMode_e      arm_mode;             /* 臂模式 */
    PhaseMode_e      arm_mode_candidate;   /* 臂模式去抖候选 */
    uint16_t         arm_mode_hold_cnt;    /* 臂模式去抖计数 (100Hz × N) */
} TaskPhase_t;

extern TaskPhase_t g_task_phase;

/* P2: 相位→导纳缩放 配置表 (取代硬编码魔数) */
typedef struct {
    float leg_gait_stance;   /* 站立相 adm (默认 1.2) */
    float leg_gait_swing;    /* 摆动相 adm (默认 0.6) */
    float leg_squat_top;     /* 站立顶基础 adm (默认 1.0) */
    float leg_squat_depth_k; /* 蹲底增量系数 (默认 0.5, adm=top+k*s) */
    float leg_free;          /* 透明 adm (默认 0.3) */
    float arm_carry_base;    /* 搬运基础 adm (默认 0.7) */
    float arm_carry_load_k;  /* 搬运负载系数 (默认 0.2, adm=base+k*load) */
    float arm_free;          /* 透明 adm (默认 0.3) */
} PhaseAdmConfig_t;
extern const PhaseAdmConfig_t g_phase_adm_config;

/* ===== API ===== */
void  global_phase_init(void);

/* §D 主入口 100Hz: 模式识别 + 相位/导纳计算 + 下发 adm_scale */
void  Global_PhaseScheduler_Update100Hz(void);

/* 只读访问 */
const TaskPhase_t *global_phase_get(void);
PhaseMode_e        global_phase_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_PHASE_H__ */
