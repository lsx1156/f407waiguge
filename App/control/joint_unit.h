/**
 * @file joint_unit.h
 * @brief JointUnit 标准化关节接口 (架构评审 #2)
 *
 * 统一封装 6 个关节的:
 *   - 身份:     CAN bus / motor_id / 关节类型 / 名称
 *   - 硬件映射: EN 使能引脚 (拉低 = 切断驱动级)
 *   - 软限位:   位置上下限 (mdeg)
 *   - 运行时状态引用: status / ABO / assist_torque
 *
 * 消除 control_isr.c / tasks.c 中散落的隐式知识:
 *   i<2 ? g_leg_status[i] : g_arm_status[i-2]
 *   push_can_tx(0, MOTOR_LEG_0_ID, ...)
 *   HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, ...)
 *   idx = tmp.joint_id - MOTOR_ARM_0_ID
 *
 * 配置部分编译期固定; 运行时状态通过指针引用 control_isr.c 中的既有全局数组,
 * 零内存迁移, 不破坏既有访问路径 (g_leg_status / g_arm_status / g_abo_state 仍可直用).
 */
#ifndef __JOINT_UNIT_H__
#define __JOINT_UNIT_H__

#include "contract.h"
#include "bsp_config.h"
#include "stm32f4xx_hal.h"
#include "control_isr.h"   /* CanTxEntry_t, can_tx_fifo_write, g_leg/g_arm_status, g_abo_state */

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 关节类型 ===== */
typedef enum {
    JOINT_TYPE_LEG_HIP      = 0,   /* 下肢髋关节 (CAN1, CyberGear) */
    JOINT_TYPE_ARM_SHOULDER = 1,   /* 上肢肩关节 (CAN2, RS01) */
    JOINT_TYPE_ARM_ELBOW    = 2,   /* 上肢肘关节 (CAN2, RS01) */
} JointType_e;

/* ===== CAN 总线选择 ===== */
typedef enum {
    JOINT_BUS_CAN1 = 0,            /* CyberGear 下肢 */
    JOINT_BUS_CAN2 = 1,            /* RS01 上肢 */
} JointBus_e;

/* ===== 零 IMU 漂移免疫包：编译开关 ===== */
#ifndef DRIFT_IMMUNE_ENABLE
#define DRIFT_IMMUNE_ENABLE     1   /* 1=开启, 0=关闭 (保留原有 ABO 路径) */
#endif

/* ====== 力矩零漂补偿 (§A, 分布式单关节内部) ======
 * 仅用 RobStride 01 原生 ABS 编码器 + 温度 + 电流反馈，无 IMU。 */
#define TEMP_TABLE_SIZE         21   /* -20~80°C, 每 5°C 一个点 */
#define TEMP_TABLE_STEP_C       5
#define TEMP_TABLE_MIN_C        (-20)

typedef struct {
    /* 出厂烧录：不同温度下的电流零漂补偿值 (Nm) */
    float    temp_coeff[TEMP_TABLE_SIZE];

    /* 实时中间量 */
    float    current_zero_thermal;   /* 温度补偿后的零点 (Nm) */
    float    current_zero_est;       /* 在线估计的零点 (Nm, EMA) */

    /* 零负载窗口检测 */
    uint16_t zero_load_window_cnt;   /* 连续「疑似零负载」计数 (ms) */
    uint8_t  zero_calib_valid;       /* 在线标定是否可信 */

    /* 阈值 (Nm / rad/s, 可按关节类型调参) */
    float    thr_tau_zero_load;      /* 零负载力矩阈值 (Nm) */
    float    thr_vel_zero_load;      /* 零负载速度阈值 (rad/s) */
} TorqueDriftComp_t;

/* ====== 3 阶 ESO 抗漂移 (§B, 分布式单关节核心) ======
 * 模型前馈剥离已知动力学 + 自适应遗忘 beta3 + 无激励冻结。
 * 状态量 float: STM32F407 带 VFPv4-SP，1kHz 单精度浮点可承载。 */
typedef struct {
    float z1;    /* 位置估计 (rad) */
    float z2;    /* 速度估计 (rad/s) */
    float z3;    /* 总扰动估计 (Nm) → 输出 tau_human */

    float vel_prev;      /* 上一拍速度 (rad/s)，用于加速度微分 */
    float payload_est;   /* 全局层下发的负载扭矩前馈 (Nm) */
    float tau_model_prev;/* 上一拍模型前馈 (Nm)，调试用 */
} ESO3State_t;

/* ====== 单关节静态参数 (ESO / 动力学) ======
 * 未来可由参数热加载 JSON 覆盖，此处给出 RobStride 01 40Nm 关节默认值。 */
typedef struct {
    float J_rotor;           /* 转子 + 连杆折算惯量 (kg·m²) */
    float friction_viscous;  /* 粘性摩擦 (Nm·s/rad) */
    float friction_coulomb;  /* 库仑摩擦 (Nm) */
    float torque_const;      /* 力矩常数 (Nm/A) */
    float gear_ratio;        /* 减速比 */

    /* ESO 3 阶带宽参数 (β1, β2, β3) */
    float eso_beta[3];

    /* 重力臂 (mgL, Nm) — MGL*cos(θ) 做重力前馈
     * 肩/髋：正值；肘：0 或小值由零位标定后启用 */
    float mgl;

    /* §E 重力力矩臂向量 (Nm/g, 即 kg·m) — grav_arm · g_base * 9.81 得重力扭矩
     * 出厂标定 (步骤4) 烧录; 默认全 0 → 退化为位置依赖 mgl*cos(q) 形式 */
    float grav_arm[3];

    /* 机械零位偏移 (rad) — 出厂标定 θ0，使 G(pos)=MGL*cos(pos + θ0) 对齐 */
    float theta0_rad;

    /* §D 导纳/阻抗基值 (被 global_adm_scale 实时缩放) */
    float adm_M_base;       /* 惯量 (kg·m²) */
    float adm_B_base;       /* 阻尼 (Nm·s/rad) */
    float adm_K_base;       /* 刚度 (Nm/rad) */

    /* 无激励检测阈值 */
    float thr_vel_noexc;     /* rad/s */
    float thr_tau_noexc;     /* Nm */
    float thr_res_noexc;     /* Nm */

    uint16_t param_version;  /* 参数版本号, 每次热加载自增 (出厂默认 1) */
} JointUnitParams_t;

/* ====== 单关节运行时状态 (§A + §B) ======
 * 每关节一个实例，纯 C，零全局变量依赖 (与架构评审一致)。 */
typedef struct {
    TorqueDriftComp_t drift;
    ESO3State_t       eso3;
    JointUnitParams_t params;

    /* 输入/输出镜像 (与既有 status/abo/assist_torque 指针解耦) */
    float pos_rad;         /* 最新反馈位置 (rad) */
    float vel_rad_s;       /* 最新反馈速度 (rad/s) */
    float tau_meas_nm;     /* 去零漂后的反馈力矩 (Nm) */
    float tau_human_nm;    /* ESO 输出人力估计 (Nm) */
    float tau_cmd_nm;      /* 导纳输出的目标力矩 (Nm, 可选) */

    /* §D 导纳控制器状态 (1kHz) */
    float adm_pos;         /* 导纳位置积分 (rad) */
    float adm_vel;         /* 导纳速度 (rad/s) */

    /* §E 基座系重力向量指针 → global_pose.gravity_base (float[3], 归一化)
     *   NULL → 退化为直立假设; 非 NULL → 送入 tau_model_forward 重力项 */
    const float *gravity_base_ptr;

    /* 故障/状态标志 */
    uint8_t eso_enable;    /* 1=ESO3 启用, 0=保留 legacy ABO */
    uint8_t eso_div_flag;  /* ESO 发散监测: 1=触发 */

    /* §C-boost: 负载突变 (抓取/放下) 时触发, ESO3 暂时增大 beta3 + 遗忘旧 z3 */
    uint8_t  eso_boost_mode;    /* 1=boost 激活 */
    uint16_t eso_boost_timer;   /* boost 剩余时间 (ms), 1kHz 递减; 0=结束 */

    /* §D: 全局相位调度器下发的导纳/阻抗缩放 (复合 K/B 因子)
     *   adm_scale=1.0 → 默认; <1.0 → 降刚度/阻尼 (透明); >1.0 → 增阻尼 (稳) */
    float global_adm_scale;
} JointUnitState_t;

/* 全局单关节实例数组 (每关节一个) */
extern JointUnitState_t g_joint_state[JOINT_COUNT];

/* ===== 关节单元 =====
 * 配置字段编译期固定; status/abo/assist_torque 指针引用既有全局,
 * 经 const JointUnit_t 仍可修改其指向内容 (指针本身不变). */
typedef struct {
    /* ---- 身份与硬件映射 (const) ---- */
    JointBus_e      bus;
    uint32_t        motor_id;        /* CAN motor ID */
    JointType_e     type;
    const char     *name;
    GPIO_TypeDef   *en_port;         /* 使能引脚 (拉低 = 切断驱动) */
    uint16_t        en_pin;
    uint16_t        control_period;  /* 控制周期 (ms) */
    int32_t         soft_limit_min;  /* 软限位下限 (mdeg) */
    int32_t         soft_limit_max;  /* 软限位上限 (mdeg) */

    /* ---- 运行时状态指针 (引用既有全局, 可变内容) ---- */
    JointStatus_t  *status;          /* -> g_leg_status / g_arm_status */
    ABOState_t     *abo;             /* -> g_abo_state[idx] */
    int32_t        *assist_torque;   /* -> g_abo_assist_torque[idx] */

    /* ---- 新增: 指向零 IMU 漂移状态 (非 wire-format) ---- */
    JointUnitState_t *state;         /* -> g_joint_state[idx] */
} JointUnit_t;

/* ===== 关节总数 (2 腿 + 4 臂) ===== */
#define JOINT_COUNT         MOTOR_COUNT_TOTAL

/* 配置表 (唯一身份/硬件映射来源) */
extern const JointUnit_t g_joint[JOINT_COUNT];

/* ===== API ===== */
void                  joint_unit_init(void);

const JointUnit_t    *joint_get(uint8_t idx);          /* 越界返回 NULL */
uint8_t               joint_count(void);

/* 便捷访问器 (越界返回 NULL) */
JointStatus_t        *joint_status_ptr(uint8_t idx);
ABOState_t           *joint_abo_ptr(uint8_t idx);
int32_t              *joint_assist_ptr(uint8_t idx);
JointUnitState_t    *joint_runtime_ptr(uint8_t idx);   /* 新增 */
CAN_HandleTypeDef    *joint_bus_handle(uint8_t idx);
JointType_e           joint_type(uint8_t idx);
uint8_t               joint_is_leg(uint8_t idx);       /* idx 0,1 → 1 */
uint8_t               joint_is_arm(uint8_t idx);       /* idx 2~5 → 1 */

/* 反向查找: motor_id → joint idx (未找到返回 0xFF) */
uint8_t               joint_idx_from_motor_id(uint32_t motor_id);

/* 硬件操作 */
void                  joint_push_tx(uint8_t idx, int32_t target, uint8_t mode);  /* 入 CAN TX FIFO */
void                  joint_disable_hw(uint8_t idx);                              /* 拉低 EN pin */

/* ===== 零 IMU 漂移免疫包 公共 API (1kHz ISR 调用) ===== */
#if DRIFT_IMMUNE_ENABLE

/* ===== v2.0 补丁3: 1kHz 回放日志 (环形缓冲 + CSV 导出) ===== */
#ifndef JOINT_LOG_ENABLE
#define JOINT_LOG_ENABLE    1   /* 1=开启回放缓冲(占用RAM), 0=关闭省RAM */
#endif
#ifndef JOINT_LOG_BUF_SIZE
#define JOINT_LOG_BUF_SIZE  256 /* 环形缓冲帧数; 6关节/ms → 约42ms历史(可调大) */
#endif

/* 单帧回放记录 (1kHz 每关节一帧) */
typedef struct {
    uint8_t  joint_idx;   /* 0~5 */
    uint8_t  phase;       /* 占位 (可接 global_phase_mode), 默认0 */
    float    pos_rad;
    float    vel_rad_s;
    float    tau_meas;    /* 去零漂后反馈力矩 (Nm) */
    float    tau_human;   /* ESO 输出人力估计 (Nm) */
    float    tau_cmd;     /* 导纳输出目标力矩 (Nm) */
    float    adm_scale;   /* global_adm_scale */
    uint32_t ts_ms;       /* HAL_GetTick() 时间戳 */
} joint_unit_log_frame_t;

/* 串口导出环形缓冲为 CSV (调试, 按需调用, 例如收到上位机指令或故障时) */
void joint_log_dump_csv(void);

/* 1kHz 周期内对单关节的完整漂移免疫更新
 *   1. TorqueDrift_Update1kHz        (§A)
 *   2. JointUnit_ESO3_Update1kHz     (§B)
 * 调用时机: 在 abo_update_one 之前或作为其替代 (eso_enable 标志控制) */
void joint_drift_immune_update1kHz(uint8_t idx, float dt_s);

/* 供全局协调器下发 payload 扭矩前馈 (§C) */
void joint_set_payload_est(uint8_t idx, float payload_torque_nm);

/* §C-boost: 触发某关节 ESO boost (负载突变时快速收敛 z3)
 *   duration_ms: boost 持续时间, 1kHz 递减 */
void joint_trigger_eso_boost(uint8_t idx, uint16_t duration_ms);

/* §C-boost: 1kHz 递减 boost timer (在 ISR 内调用) */
void joint_eso_boost_tick_1kHz(void);

/* §E: 绑定基座系重力向量指针 (上电一次, g_base_ptr 指向 float[3]) */
void joint_bind_gravity_vector(uint8_t idx, const float *g_base_ptr);

/* 获取 ESO3 人力估计 (Nm), 负值表示反方向 */
float joint_get_human_torque_nm(uint8_t idx);

/* P2-3: 参数热加载校验. 校验 new_params 物理合理性后原子写入关节 idx,
 *        param_version 自增. 返回 1=成功, 0=校验失败/idx越界.
 *        JSON/上位机解析得到参数后应调用本函数, 由本函数统一做边界校验. */
uint8_t joint_unit_params_hotload(uint8_t idx, const JointUnitParams_t *new_params);

#endif /* DRIFT_IMMUNE_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __JOINT_UNIT_H__ */
