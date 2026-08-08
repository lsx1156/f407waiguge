/**
 * @file global_coordinator.h
 * @brief 全局协调单例 — 顶层协同 (架构评审 §C + 未来 AO/相位)
 *
 * 硬约束: 单芯片、无 IMU、无外挂 MCU。
 *
 * 现已落地:
 *   §C  Global_LoadEstimator_Update100Hz
 *        全身运动学 + 多关节力矩耦合解算「末端外力/负载」
 *        下发 payload_torque 给每关节 → §B ESO3 模型前馈剥离
 *
 * 未来占位:
 *   §D  关节空间相位变量 + 任务状态机双轨制 (解决 AO 相位漂移)
 *   §E  ZUPT + 足底力矩接触检测 + 闭链约束 (基座里程计漂移免疫)
 */
#ifndef __GLOBAL_COORDINATOR_H__
#define __GLOBAL_COORDINATOR_H__

#include "joint_unit.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 末端外力 (3 维平面近似, 工业工况低速) =====
 * 单位: N (力), N·m (力矩). 目前左臂/右臂单独估计, 下肢独立估计重力偏置. */
typedef struct {
    float fx;           /* x 向力 (N) */
    float fy;           /* y 向力 (N) */
    float m_ext;        /* 末端外力矩 (N·m), 通常 0 (手抓自由抓握) */
} EndEffectorForce_t;

/* ===== 全局协调器状态 =====
 * 顶层单例; 不与 wire-format 协议耦合. */
typedef struct {
    EndEffectorForce_t f_ext_arm[2];   /* 0=左臂, 1=右臂 */
    float              joint_payload_torque[JOINT_COUNT];  /* 对每关节的 payload 扭矩 (Nm)
                                                            * 最终下发给 JointUnit payload_est */

    /* 突变检测 (抓取/放下 识别) */
    float              f_prev_norm[2];      /* 上一拍 |F| */
    uint8_t            payload_change_flag; /* 1=检测到突变, 0=稳定 */
    uint16_t           payload_change_cnt;  /* 去抖计数 */
    uint16_t           load_change_timer;   /* §C 突变持续窗口 (ms, 100Hz 递减 10/拍), 0=稳定 */

    /* 工业工况识别: 站立/搬运/行走/蹲起 */
    uint8_t            op_mode;             /* 0=未知, 1=站立, 2=搬运, 3=行走, 4=蹲起 */

    /* P0-3: 雅可比奇异兜底
     *   奇异时不计算新 F_ext, 保持上一拍有效值 (f_last_valid), 不下发 payload,
     *   仅递增奇异计数. 避免奇异退化为单位阵把 tau_ext 直接当 F_ext 输出. */
    EndEffectorForce_t f_last_valid[2];     /* 上一拍有效末端力 (奇异时回退) */
    uint8_t            jacobian_singular[2];/* 1=本拍雅可比奇异, 0=正常 */
    uint32_t           singular_cnt[2];     /* 累计奇异次数 (调试/上报) */
} GlobalCoordinator_t;

extern GlobalCoordinator_t g_global_coord;

/* ===== API ===== */
void global_coordinator_init(void);

/* §C  主循环 100Hz 调用 (与 industrial_load_estimator 同时钟域)
 * 输入: 6 关节反馈 (位置 + 去零漂后的力矩)
 * 副作用: 写入 g_global_coord.joint_payload_torque[]
 *          调用 joint_set_payload_est(i, tau_payload[i]) 下发给分布式关节 */
void Global_LoadEstimator_Update100Hz(void);

/* 只读访问: 获取末端外力估计 (调试/上传) */
const EndEffectorForce_t *global_get_arm_force(uint8_t side);  /* side 0=L, 1=R */
float                    global_get_payload_torque(uint8_t joint_idx);

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_COORDINATOR_H__ */
