/**
 * @file joint_unit.c
 * @brief JointUnit 标准化关节接口实现 (架构评审 #2)
 *
 * 零 IMU 漂移免疫包 (§A + §B) 也落在本文件：
 *   §A  TorqueDrift_Update1kHz    : 温度表插值 + 零负载窗口在线标定
 *   §B  JointUnit_ESO3_Update1kHz : 3 阶 ESO + 模型前馈 + 自适应 beta3 + 无激励冻结
 *
 * 配置表 g_joint[] 编译期固定, 引用 control_isr.c 既有全局:
 *   g_leg_status[2] / g_arm_status[4] / g_abo_state[6] / g_abo_assist_torque[6]
 * 零内存迁移, 既有全局仍可独立访问, 本表为新增的统一入口.
 */
#include "joint_unit.h"
#include "main.h"       /* hcan1 / hcan2 */
#include "safety.h"     /* ARM_SHOULDER/ELBOW 软限位 */
#include "bsp_config.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ===== 单位转换常量 (与 control_isr.c 保持一致) ===== */
#define MDEG_TO_RAD_F        (1.7453292519943295e-05f)   /* mdeg → rad, π/(180*1000) */
#define RAD_TO_MDEG_F        (57295.77951308232f)        /* rad → mdeg        */
#define MNM_TO_NM_F          (0.001f)                     /* mNm → Nm           */
#define NM_TO_MNM_F          (1000.0f)                    /* Nm → mNm           */

/* ===== 全局单关节运行时状态数组 (零 IMU 漂移免疫包) ===== */
JointUnitState_t g_joint_state[JOINT_COUNT];

#if JOINT_LOG_ENABLE
/* v2.0 补丁3: 1kHz 环形回放缓冲 (单缓冲, 6关节交错入队) */
static joint_unit_log_frame_t g_log_buf[JOINT_LOG_BUF_SIZE];
static volatile uint16_t g_log_head = 0u;

static void joint_log_push(uint8_t idx, const JointUnitState_t *st)
{
    joint_unit_log_frame_t *f = &g_log_buf[g_log_head];
    f->joint_idx = idx;
    f->phase     = 0u;                 /* 占位 (保持 joint_unit 解耦, 不引 global_phase) */
    f->pos_rad   = st->pos_rad;
    f->vel_rad_s = st->vel_rad_s;
    f->tau_meas  = st->tau_meas_nm;
    f->tau_human = st->tau_human_nm;
    f->tau_cmd   = st->tau_cmd_nm;
    f->adm_scale = st->global_adm_scale;
    f->ts_ms     = HAL_GetTick();
    g_log_head = (uint16_t)((g_log_head + 1u) % JOINT_LOG_BUF_SIZE);
}
#endif

/* ===== 默认出厂温度零漂表 (TEMP_TABLE_SIZE = 21, -20~80°C / 5°C step)
 * 冷启动先用出厂表; 每关节热箱标定后可通过 eeprom_params 热加载覆盖.
 * 默认值全 0 (表示"未出厂标定", 仅靠在线窗口校准) */
static const float g_default_temp_coeff[TEMP_TABLE_SIZE] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

/* ===== 默认 RobStride 01 40Nm 关节参数 (待 JSON 热加载覆盖) =====
 * 保守估计, 仅用作启用 ESO3 前的默认值. */
static void joint_params_set_default(JointUnitParams_t *p, JointType_e type)
{
    p->J_rotor          = 0.00012f;    /* 转子 + 连杆折算惯量, 保守值 (kg·m²) */
    p->friction_viscous = 0.008f;      /* 粘性摩擦 (Nm·s/rad) */
    p->friction_coulomb = 0.45f;       /* 库仑摩擦 (Nm) */
    p->torque_const     = 0.18f;       /* 力矩常数 (Nm/A) */
    p->gear_ratio       = 54.0f;       /* 减速比 */

    /* ESO 带宽: β1=1200, β2=80000, β3=5e5 (连续域, 此处直接用 1kHz 欧拉离散) */
    p->eso_beta[0] = 1200.0f;
    p->eso_beta[1] = 80000.0f;
    p->eso_beta[2] = 500000.0f;

    /* 重力前馈 + 零位 (出厂 θ₀=0, 需现场标定后再覆盖启用)
     *   髋: 0 (步态时下肢重力由 ABO 偏置吸收)
     *   肩: 保守 6.0 Nm (对应 g_gravity_mgl=6963 mNm ≈ 7 N·m)
     *   肘: 保守 1.5 Nm (对应 g_gravity_mgl=1638 mNm ≈ 1.6 N·m) */
    if (type == JOINT_TYPE_LEG_HIP) {
        p->mgl        = 0.0f;
    } else if (type == JOINT_TYPE_ARM_SHOULDER) {
        p->mgl        = 6.0f;
    } else {
        p->mgl        = 1.5f;
    }
    p->theta0_rad     = 0.0f;

    /* §E 重力力矩臂向量: 默认全 0 → 退化为位置依赖 mgl*cos(q) (待出厂标定步骤4 烧录) */
    p->grav_arm[0]    = 0.0f;
    p->grav_arm[1]    = 0.0f;
    p->grav_arm[2]    = 0.0f;

    /* §D 导纳/阻抗基值 (被 global_adm_scale 实时缩放)
     *   M=0.02, B=0.5, K=2.0: 40Nm 关节保守值, 自然频率 ~1.6Hz, 阻尼比 ~1.25 (过阻尼稳) */
    p->adm_M_base     = 0.02f;
    p->adm_B_base     = 0.5f;
    p->adm_K_base     = 2.0f;

    /* 无激励检测阈值 */
    p->thr_vel_noexc  = 1e-3f;         /* 1 mrad/s ≈ 0.057 °/s */
    p->thr_tau_noexc  = 0.05f;         /* 50 mNm */
    p->thr_res_noexc  = 0.1f;          /* 100 mNm */

    /* P2-3: 参数版本号 (出厂默认 1, 每次热加载自增) */
    p->param_version  = 1u;
}

/* ===== 关节配置表 (唯一身份/硬件映射来源) =====
 * idx: 0=L-Hip 1=R-Hip 2=L-Shldr 3=L-Elbow 4=R-Shldr 5=R-Elbow
 *   腿: CAN1 / CyberGear / EN_LEG_L,R
 *   臂: CAN2 / RS01     / EN_ARM_1~4
 * 软限位: 腿用步态目标钳位范围; 肩/肘沿用 safety.h 定义 */
const JointUnit_t g_joint[JOINT_COUNT] = {
    { JOINT_BUS_CAN1, MOTOR_LEG_0_ID,     JOINT_TYPE_LEG_HIP,      "L-Hip",
      EN_LEG_L_PORT, EN_LEG_L_PIN, CONTROL_PERIOD_LEG,
      LOCAL_HIP_POS_MIN_MDEG, LOCAL_HIP_POS_MAX_MDEG,
      &g_leg_status[0], &g_abo_state[0], &g_abo_assist_torque[0],
      &g_joint_state[0] },

    { JOINT_BUS_CAN1, MOTOR_LEG_1_ID,     JOINT_TYPE_LEG_HIP,      "R-Hip",
      EN_LEG_R_PORT, EN_LEG_R_PIN, CONTROL_PERIOD_LEG,
      LOCAL_HIP_POS_MIN_MDEG, LOCAL_HIP_POS_MAX_MDEG,
      &g_leg_status[1], &g_abo_state[1], &g_abo_assist_torque[1],
      &g_joint_state[1] },

    { JOINT_BUS_CAN2, MOTOR_ARM_0_ID,     JOINT_TYPE_ARM_SHOULDER, "L-Shldr",
      EN_ARM_1_PORT, EN_ARM_1_PIN, CONTROL_PERIOD_ARM,
      ARM_SHOULDER_POS_MIN, ARM_SHOULDER_POS_MAX,
      &g_arm_status[0], &g_abo_state[2], &g_abo_assist_torque[2],
      &g_joint_state[2] },

    { JOINT_BUS_CAN2, MOTOR_ARM_0_ID + 1, JOINT_TYPE_ARM_ELBOW,    "L-Elbow",
      EN_ARM_2_PORT, EN_ARM_2_PIN, CONTROL_PERIOD_ARM,
      ARM_ELBOW_POS_MIN, ARM_ELBOW_POS_MAX,
      &g_arm_status[1], &g_abo_state[3], &g_abo_assist_torque[3],
      &g_joint_state[3] },

    { JOINT_BUS_CAN2, MOTOR_ARM_0_ID + 2, JOINT_TYPE_ARM_SHOULDER, "R-Shldr",
      EN_ARM_3_PORT, EN_ARM_3_PIN, CONTROL_PERIOD_ARM,
      ARM_SHOULDER_POS_MIN, ARM_SHOULDER_POS_MAX,
      &g_arm_status[2], &g_abo_state[4], &g_abo_assist_torque[4],
      &g_joint_state[4] },

    { JOINT_BUS_CAN2, MOTOR_ARM_0_ID + 3, JOINT_TYPE_ARM_ELBOW,    "R-Elbow",
      EN_ARM_4_PORT, EN_ARM_4_PIN, CONTROL_PERIOD_ARM,
      ARM_ELBOW_POS_MIN, ARM_ELBOW_POS_MAX,
      &g_arm_status[3], &g_abo_state[5], &g_abo_assist_torque[5],
      &g_joint_state[5] },
};

/* ===== 辅助: 钳位 ===== */
static inline float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* P1-1a: 分段线性近似 tanh, 替代 tanhf 节省 ~80 周期/调用
 *   |x|<2  线性斜率 0.5 钳位 [-1,1]
 *   |x|>=2 饱和 ±1
 *   tanh(2)≈0.964, 0.5x 在 x=2 给 1.0, 误差可接受 (β3 自适应已含钳位兜底) */
static inline float tanh_approx(float x) {
    if (x > 2.0f)  return 1.0f;
    if (x < -2.0f) return -1.0f;
    return 0.5f * x;
}

/* P2: 256 点正弦查表 + 线性插值, 替代 cosf/sinf 节省 ~150 周期/关节
 *   g_fast_sin_table[i] = sinf(2π·i/256), i∈[0,255], 覆盖 [0,2π) 一个周期
 *   编译期常量, 写死数值 (6 位小数) */
#define FAST_TWO_PI  6.2831853f
static const float g_fast_sin_table[256] = {
    0.000000f, 0.024541f, 0.049068f, 0.073565f, 0.098017f, 0.122411f, 0.146730f, 0.170962f,
    0.195090f, 0.219101f, 0.242980f, 0.266713f, 0.290285f, 0.313682f, 0.336890f, 0.359895f,
    0.382683f, 0.405241f, 0.427555f, 0.449611f, 0.471397f, 0.492898f, 0.514103f, 0.534998f,
    0.555570f, 0.575808f, 0.595699f, 0.615231f, 0.634393f, 0.653173f, 0.671559f, 0.689541f,
    0.707107f, 0.724247f, 0.740951f, 0.757209f, 0.773010f, 0.788346f, 0.803208f, 0.817585f,
    0.831470f, 0.844853f, 0.857729f, 0.870087f, 0.881921f, 0.893224f, 0.903989f, 0.914210f,
    0.923880f, 0.932993f, 0.941544f, 0.949528f, 0.956940f, 0.963776f, 0.970031f, 0.975702f,
    0.980785f, 0.985278f, 0.989177f, 0.992479f, 0.995185f, 0.997290f, 0.998795f, 0.999699f,
    1.000000f, 0.999699f, 0.998795f, 0.997290f, 0.995185f, 0.992479f, 0.989177f, 0.985278f,
    0.980785f, 0.975702f, 0.970031f, 0.963776f, 0.956940f, 0.949528f, 0.941544f, 0.932993f,
    0.923880f, 0.914210f, 0.903989f, 0.893224f, 0.881921f, 0.870087f, 0.857729f, 0.844853f,
    0.831470f, 0.817585f, 0.803208f, 0.788346f, 0.773010f, 0.757209f, 0.740951f, 0.724247f,
    0.707107f, 0.689541f, 0.671559f, 0.653173f, 0.634393f, 0.615231f, 0.595699f, 0.575808f,
    0.555570f, 0.534998f, 0.514103f, 0.492898f, 0.471397f, 0.449611f, 0.427555f, 0.405241f,
    0.382683f, 0.359895f, 0.336890f, 0.313682f, 0.290285f, 0.266713f, 0.242980f, 0.219101f,
    0.195090f, 0.170962f, 0.146730f, 0.122411f, 0.098017f, 0.073565f, 0.049068f, 0.024541f,
    0.000000f, -0.024541f, -0.049068f, -0.073565f, -0.098017f, -0.122411f, -0.146730f, -0.170962f,
    -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
    -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f, -0.514103f, -0.534998f,
    -0.555570f, -0.575808f, -0.595699f, -0.615231f, -0.634393f, -0.653173f, -0.671559f, -0.689541f,
    -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
    -0.831470f, -0.844853f, -0.857729f, -0.870087f, -0.881921f, -0.893224f, -0.903989f, -0.914210f,
    -0.923880f, -0.932993f, -0.941544f, -0.949528f, -0.956940f, -0.963776f, -0.970031f, -0.975702f,
    -0.980785f, -0.985278f, -0.989177f, -0.992479f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
    -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992479f, -0.989177f, -0.985278f,
    -0.980785f, -0.975702f, -0.970031f, -0.963776f, -0.956940f, -0.949528f, -0.941544f, -0.932993f,
    -0.923880f, -0.914210f, -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844853f,
    -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f, -0.740951f, -0.724247f,
    -0.707107f, -0.689541f, -0.671559f, -0.653173f, -0.634393f, -0.615231f, -0.595699f, -0.575808f,
    -0.555570f, -0.534998f, -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
    -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f, -0.242980f, -0.219101f,
    -0.195090f, -0.170962f, -0.146730f, -0.122411f, -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

/* 快速正弦: 归一化到 [0,2π) 后 256 点查表 + 线性插值
 *   归一化用乘 1/(2π) + 整数截断 (避免 fmodf/while 迭代)
 *   (int32_t) 向零截断丢弃整数周期; 负角残差为负, 补 1.0 落到 [0,1) */
static inline float fast_sinf(float x)
{
    const float INV_TWO_PI = 0.15915494f;       /* 1/(2π) */
    float k_float = x * INV_TWO_PI;             /* x/(2π) */
    int32_t ki = (int32_t)k_float;              /* 整数周期 (向零截断) */
    float frac_k = k_float - (float)ki;         /* 周期内位置 ∈(-1,1) */
    if (frac_k < 0.0f) frac_k += 1.0f;          /* 负角修正 → [0,1) */

    float idx = frac_k * 256.0f;                /* 表索引 ∈[0,256) */
    uint32_t i = (uint32_t)idx & 0xFFu;         /* 整数下标, 回绕 */
    float frac = idx - (float)(uint32_t)idx;    /* 小数部分 (idx≥0 故截断=向下取整) */
    float s0 = g_fast_sin_table[i];
    float s1 = g_fast_sin_table[(i + 1u) & 0xFFu];
    return s0 + frac * (s1 - s0);               /* 线性插值 */
}

/* 快速余弦: cos(x) = sin(x + π/2) */
static inline float fast_cosf(float x)
{
    return fast_sinf(x + 1.5707963f);
}

/* ===== 温度表线性插值 =====
 * temp_c / TEMP_TABLE_STEP_C = 整数 index, frac = 小数部分
 * table[-20°C] at index 0, table[80°C] at index 20 */
static float temp_table_linterp(const float table[TEMP_TABLE_SIZE], float temp_c)
{
    float fidx = (temp_c - (float)TEMP_TABLE_MIN_C) / (float)TEMP_TABLE_STEP_C;
    if (fidx <= 0.0f) return table[0];
    if (fidx >= (float)(TEMP_TABLE_SIZE - 1)) return table[TEMP_TABLE_SIZE - 1];
    int i = (int)fidx;
    float frac = fidx - (float)i;
    return table[i] + frac * (table[i + 1] - table[i]);
}

/* ===== §A  力矩零漂补偿 (1kHz) =====
 * 输入: 原始 torque_raw (Nm, 电机反馈), motor_temp_c (°C, RobStride 上报),
 *       vel_rad_s (编码器速度, rad/s), saturation (0~1 估算电流环饱和度)
 * 输出: 去零漂后的 tau_clean (Nm) → 写入 st->tau_meas_nm */
static void TorqueDrift_Update1kHz(JointUnitState_t *st,
                                   float torque_raw_nm,
                                   float motor_temp_c,
                                   float vel_rad_s,
                                   float saturation)
{
    TorqueDriftComp_t *dc = &st->drift;

    /* 1. 温度补偿查表 (出厂表) */
    dc->current_zero_thermal = temp_table_linterp(dc->temp_coeff, motor_temp_c);

    /* 2. 零负载窗口检测
     *   |tau - thermal_zero| < 阈值
     *   且 |vel|                < 阈值
     *   且 饱和度 低 (避免闭环积分残留力矩误判零负载) */
    float tau_thermal_corr = torque_raw_nm - dc->current_zero_thermal;
    uint8_t likely_zero_load =
        (fabsf(tau_thermal_corr) < dc->thr_tau_zero_load) &&
        (fabsf(vel_rad_s)          < dc->thr_vel_zero_load) &&
        (saturation                < 0.1f);

    if (likely_zero_load) {
        /* 指数滑动平均, alpha=0.001 → 时间常数 ~1s 等效 1kHz */
        dc->current_zero_est = 0.999f * dc->current_zero_est + 0.001f * torque_raw_nm;
        if (dc->zero_load_window_cnt < 0xFFFFu) dc->zero_load_window_cnt++;
        /* 5s 连续确认 => 可信 */
        if (dc->zero_load_window_cnt > 5000u) dc->zero_calib_valid = 1u;
    } else {
        dc->zero_load_window_cnt = 0;
    }

    /* 3. 输出去零漂力矩
     *   零载标定可信 → 用 zero_est; 否则退回到出厂温度表 */
    float zero = dc->zero_calib_valid ? dc->current_zero_est : dc->current_zero_thermal;
    st->tau_meas_nm = torque_raw_nm - zero;
}

/* ===== §E  重力补偿 (基座系向量投影) =====
 * 输入: g_base[3] = global_pose.gravity_base (归一化, 直立=[0,0,-1])
 *   - grav_arm 已标定 (范数>ε): tau_g = grav_arm · g_base * 9.81 (Nm)
 *   - grav_arm 未标定 (默认 0): 退化为位置依赖形式
 *       tau_g = mgl * (-gz*cos(q) + gx*sin(q)), q = pos + θ0
 *       直立 gz=-1 → mgl*cos(q), 与旧代码完全一致 (行为不变) */
static float Gravity_Comp_Vector(const JointUnitState_t *st, const float g_base[3])
{
    const JointUnitParams_t *p = &st->params;
    float ga_norm = sqrtf(p->grav_arm[0]*p->grav_arm[0] +
                          p->grav_arm[1]*p->grav_arm[1] +
                          p->grav_arm[2]*p->grav_arm[2]);
    if (ga_norm > 1e-5f) {
        /* 已标定: 向量投影 (grav_arm 单位 Nm/g, ×9.81 → Nm) */
        return (p->grav_arm[0]*g_base[0] +
                p->grav_arm[1]*g_base[1] +
                p->grav_arm[2]*g_base[2]) * 9.81f;
    }
    /* 未标定: 位置依赖 + g_base 修正 (直立退化为旧 mgl*cos) */
    float q = st->pos_rad + p->theta0_rad;
    return p->mgl * (-g_base[2]*fast_cosf(q) + g_base[0]*fast_sinf(q));
}

/* ===== §B  模型前馈 (已知动力学剥离) ===== */
static float tau_model_forward(JointUnitState_t *st, float acc_rad_s2)
{
    const JointUnitParams_t *p = &st->params;

    /* 摩擦: B*vel + C*sgn(vel)
     * sgn(0)=0 避免低速抖动. 摩擦参数需要逐关节标定. */
    float vel = st->vel_rad_s;
    float fric = p->friction_viscous * vel;
    if (vel > 1e-4f)       fric += p->friction_coulomb;
    else if (vel < -1e-4f) fric -= p->friction_coulomb;

    /* §E 重力前馈: 基座系向量投影 (有 gravity_base_ptr) / 直立退化 (无) */
    float grav;
    if (st->gravity_base_ptr) {
        grav = Gravity_Comp_Vector(st, st->gravity_base_ptr);
    } else {
        grav = p->mgl * fast_cosf(st->pos_rad + p->theta0_rad);   /* 兜底: 直立假设 */
    }

    /* 惯量项: J*acc */
    float inertia = p->J_rotor * acc_rad_s2;

    /* 全局 payload 前馈 (§C 全局协调器 100Hz 下发) */
    float payload = st->eso3.payload_est;

    return inertia + fric + grav + payload;
}

/* ===== §D  导纳/阻抗控制器 (1kHz) =====
 * M*ddx + B*dx + K*x = tau_human  → 解出 adm_pos/adm_vel/tau_cmd
 * M/B/K 被 global_adm_scale (§D 100Hz 下发) 实时缩放 */
static void Admittance_Update1kHz(JointUnitState_t *st, float dt_s)
{
    const JointUnitParams_t *p = &st->params;
    float s = st->global_adm_scale;
    if (s < 0.05f) s = 0.05f;   /* 钳位下限, 避免退化除零 */

    float M = p->adm_M_base * s;
    float B = p->adm_B_base * s;
    float K = p->adm_K_base * s;

    /* 半隐式欧拉: 先更新速度再更新位置 (稳定性优于显式) */
    float tau_h = st->tau_human_nm;
    st->adm_vel += (tau_h - B * st->adm_vel - K * st->adm_pos) / M * dt_s;
    st->adm_pos += st->adm_vel * dt_s;

    /* 目标力矩 = 阻抗律输出 (K*x + B*dx) */
    st->tau_cmd_nm = K * st->adm_pos + B * st->adm_vel;
}

/* ===== §B  3 阶 ESO 抗漂移 (1kHz) ===== */
static void JointUnit_ESO3_Update1kHz(JointUnitState_t *st, float tau_cmd_nm, float dt_s)
{
    ESO3State_t *es = &st->eso3;
    const JointUnitParams_t *p = &st->params;

    /* 1. 加速度微分 (速度差分, 1kHz 下噪声可接受) */
    float acc = (st->vel_rad_s - es->vel_prev) / dt_s;
    es->vel_prev = st->vel_rad_s;

    /* 2. 精确模型前馈 (剥离已知动力学) */
    float tau_model = tau_model_forward(st, acc);
    es->tau_model_prev = tau_model;

    /* 3. ESO 仅观测未建模残差 */
    float tau_residual = st->tau_meas_nm - tau_model;

    /* 4. 标准 3 阶 ESO 欧拉离散化
     *   ė = C z - y
     *   ż₁ = z₂ - β₁ e
     *   ż₂ = z₃ + Kτ u - β₂ e     (Kτ = kt/J)
     *   ż₃ = -β₃_adapt e
     * 注释: "人力估计" tau_human 直接取 z3 (残差域的总扰动) */
    float e = es->z1 - st->pos_rad;
    float b0 = p->torque_const / p->J_rotor;   /* 输入增益 = kt / J */

    es->z1 += dt_s * (es->z2 - p->eso_beta[0] * e);
    es->z2 += dt_s * (es->z3 + b0 * tau_cmd_nm - p->eso_beta[1] * e);

    /* 5. 自适应 β3
     *   corr = z3 * (τ_res - z3)
     *   同号 → z3 与残差变化同向, 收敛 / 跟踪需要 → beta3 增大
     *   异号 → z3 可能积分漂移 → beta3 减小, 抑制风车
     *   用 tanh 软化, 增益 0.5~1.5 × 基值 beta3_base */
    float residual_change = tau_residual - es->z3;
    float correlation = es->z3 * residual_change;
    float beta3_base = p->eso_beta[2];
    /* P1-1a: tanh_approx 替代 tanhf (分段线性, 误差由后续 clampf_local 兜底) */
    float beta3_adapt = beta3_base * (1.0f + 0.5f * tanh_approx(correlation * 10.0f));
    beta3_adapt = clampf_local(beta3_adapt, 0.1f * beta3_base, 5.0f * beta3_base);

    /* 5b. §C-boost: 负载突变 (抓取/放下) 时
     *   - beta3 × 3 (快速跟踪新负载下的扰动)
     *   - z3 向 0 衰减 (遗忘旧负载下的扰动估计)
     *   - 旁路无激励冻结 (突变期需要持续积分收敛) */
    uint8_t boost_active = st->eso_boost_mode && (st->eso_boost_timer > 0u);
    if (boost_active) {
        beta3_adapt *= 3.0f;
        /* 遗忘因子: 每拍把 z3 拉向 0 约 5% (时间常数 ~20ms @1kHz) */
        es->z3 *= 0.95f;
    }

    /* 6. 无激励检测 → 冻结 z3 积分
     *   无激励时 z3 积分噪声累积是漂移主因之一. beta3=0 等价于冻结.
     *   boost 期间旁路冻结 (突变期需要持续收敛). */
    uint8_t no_excitation =
        (fabsf(st->vel_rad_s)   < p->thr_vel_noexc) &&
        (fabsf(tau_cmd_nm)      < p->thr_tau_noexc) &&
        (fabsf(residual_change) < p->thr_res_noexc);

    if (!no_excitation || boost_active) {
        es->z3 += dt_s * (-beta3_adapt * e);
    } /* else 冻结 z3 (等价 beta3=0) */

    /* 7. 发散监测 (|z3| 超过合理范围), 置位标志位供上层故障状态机使用
     *    v2.0: 改为纯锁存事件 — 仅置位, 不在此清除。由 safety_check_algorithm_faults()
     *    (1kHz, joint_drift_immune_update1kHz 之后调用) 消费后清0, 并经保持定时器
     *    上报 FAULT_ESO_DIVERGE。避免降级(eso_enable=0)后 ESO 停止、z3 冻结>60
     *    导致滞回清除永不到达而永久锁死。 */
    if (fabsf(es->z3) > 60.0f) {  /* RobStride 01 40Nm, 允许 60 Nm 异常余量 */
        st->eso_div_flag = 1u;
    }

    /* 8. 输出人力估计 (仅含 人力 + 真正未建模扰动) */
    st->tau_human_nm = es->z3;
}

/* ===== 公共 API: 1kHz 完整漂移免疫更新 ===== */
#if DRIFT_IMMUNE_ENABLE
void joint_drift_immune_update1kHz(uint8_t idx, float dt_s)
{
    const JointUnit_t *j = joint_get(idx);
    if (!j || !j->state) return;
    JointUnitState_t *st = j->state;
    JointStatus_t  *fb = j->status;

    /* 单位转换 (mdeg → rad, mNm → Nm; 温度 0.1°C → °C) */
    st->pos_rad   = (float)fb->position * MDEG_TO_RAD_F;
    st->vel_rad_s = (float)fb->velocity * MDEG_TO_RAD_F;
    float tau_raw = (float)fb->torque   * MNM_TO_NM_F;
    float temp_c  = (float)((int16_t)fb->temperature) * 0.1f;
    /* P0-2: fb->temperature 异常值 (如 INT16_MIN → -3276.8°C) 会导致 temp_c 爆表,
     *       进而使温度表插值越界/零漂补偿异常. 钳位到传感器合理量程 [-40,100]°C. */
    temp_c = clampf_local(temp_c, -40.0f, 100.0f);

    /* §A  力矩零漂补偿 */
    TorqueDrift_Update1kHz(st, tau_raw, temp_c, st->vel_rad_s, 0.0f /*saturation 估算占位*/);

    /* §B  ESO3 + §D 导纳 仅在启用时运行 (默认关, 保 legacy ABO 兼容)
     * eso_enable 在 joint_unit_init 后可由上位机/配置包开启.
     * 数据流: §A tau_meas → §B ESO3 tau_human → §D 导纳 tau_cmd → 反喂 ESO3 u */
    if (st->eso_enable) {
        /* 1. ESO3 更新 (产出 tau_human_nm) */
        JointUnit_ESO3_Update1kHz(st, st->tau_cmd_nm, dt_s);
        /* 2. §D 导纳: 用 tau_human 解算 adm_pos/adm_vel/tau_cmd (被 global_adm_scale 缩放) */
        Admittance_Update1kHz(st, dt_s);
        /* 3. 下一拍 ESO3 的 u 项用新 tau_cmd (本拍已写入 st->tau_cmd_nm) */
    }

    /* v2.0: ESO3 发散 → 自动降级 + 复位 ESO 状态
     *   eso_div_flag 作为锁存事件保留置位, 由 safety_check_algorithm_faults()
     *   (1kHz, 本函数之后调用) 消费后清除, 并经保持定时器上报 FAULT_ESO_DIVERGE。
     *   复位 z3=0 避免重启用残留发散值; 降级后回 legacy ABO+PID, 不锁死机器人。 */
    if (st->eso_div_flag) {
        st->eso_enable = 0u;            /* 自动降级回 legacy */
        st->eso3.z1      = st->pos_rad;
        st->eso3.z2      = st->vel_rad_s;
        st->eso3.z3      = 0.0f;
        st->eso3.vel_prev = st->vel_rad_s;
    }

#if JOINT_LOG_ENABLE
    joint_log_push(idx, st);   /* 1kHz 回放入队 (6关节/ms 交错) */
#endif
}

void joint_set_payload_est(uint8_t idx, float payload_torque_nm)
{
    JointUnitState_t *st = joint_runtime_ptr(idx);
    if (!st) return;
    st->eso3.payload_est = payload_torque_nm;
}

void joint_trigger_eso_boost(uint8_t idx, uint16_t duration_ms)
{
    JointUnitState_t *st = joint_runtime_ptr(idx);
    if (!st) return;
    st->eso_boost_mode = 1u;
    /* 取较大值: 已在 boost 时不缩短剩余时间 */
    if (duration_ms > st->eso_boost_timer) st->eso_boost_timer = duration_ms;
}

void joint_eso_boost_tick_1kHz(void)
{
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        JointUnitState_t *st = &g_joint_state[i];
        if (st->eso_boost_timer > 0u) {
            st->eso_boost_timer--;
            if (st->eso_boost_timer == 0u) st->eso_boost_mode = 0u;
        }
    }
}

void joint_bind_gravity_vector(uint8_t idx, const float *g_base_ptr)
{
    JointUnitState_t *st = joint_runtime_ptr(idx);
    if (!st) return;
    st->gravity_base_ptr = g_base_ptr;   /* 上电一次绑定, 1kHz 只读 */
}

float joint_get_human_torque_nm(uint8_t idx)
{
    JointUnitState_t *st = joint_runtime_ptr(idx);
    if (!st) return 0.0f;
    return st->tau_human_nm;
}

/* P2-3: 参数热加载校验 + 原子写入
 *   1. idx 越界 / 空指针 → 返回 0
 *   2. 物理合理性范围校验 (任一失败返回 0)
 *   3. 关中断原子拷贝 (参考 control_isr.c abo_set_param 写法, 与 1kHz ISR 并发安全),
 *      param_version 自增 (若为 0 则置 1, 保证版本号非零).
 *   返回 1 = 成功. */
uint8_t joint_unit_params_hotload(uint8_t idx, const JointUnitParams_t *new_params)
{
    /* 1. 基础校验: idx 越界 / 空指针 */
    if (idx >= JOINT_COUNT || new_params == NULL) return 0u;

    /* 2. 物理合理性范围校验 (任一失败返回 0) */
    if (!(new_params->J_rotor > 0.0f && new_params->J_rotor < 1.0f))         return 0u;
    if (!(new_params->gear_ratio > 0.0f))                                    return 0u;
    if (!(new_params->torque_const > 0.0f))                                  return 0u;
    if (!(new_params->friction_viscous >= 0.0f))                             return 0u;
    if (!(new_params->friction_coulomb >= 0.0f))                             return 0u;
    if (!(new_params->eso_beta[0] > 0.0f && new_params->eso_beta[0] < 1e7f)) return 0u;
    if (!(new_params->eso_beta[1] > 0.0f && new_params->eso_beta[1] < 1e7f)) return 0u;
    if (!(new_params->eso_beta[2] > 0.0f && new_params->eso_beta[2] < 1e7f)) return 0u;
    if (!(new_params->adm_M_base > 0.0f))                                    return 0u;
    if (!(new_params->adm_B_base >= 0.0f))                                   return 0u;
    if (!(new_params->adm_K_base >= 0.0f))                                   return 0u;
    if (!(new_params->thr_vel_noexc > 0.0f))                                 return 0u;
    if (!(new_params->thr_tau_noexc > 0.0f))                                 return 0u;
    if (!(new_params->thr_res_noexc > 0.0f))                                 return 0u;

    /* 3. 关中断原子拷贝 (与 1kHz ISR 并发安全) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(&g_joint_state[idx].params, new_params, sizeof(JointUnitParams_t));
    /* param_version 自增; 若为 0 (首次/越界恢复) 置 1, 保证版本号非零 */
    if (g_joint_state[idx].params.param_version == 0u) {
        g_joint_state[idx].params.param_version = 1u;
    } else {
        g_joint_state[idx].params.param_version++;
    }
    if (!primask) __enable_irq();

    return 1u;
}
#endif /* DRIFT_IMMUNE_ENABLE */

/* ===== 初始化 ===== */
void joint_unit_init(void)
{
    /* 1. 初始化每个关节的漂移参数 + 默认物理参数 */
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        JointUnitState_t *st = &g_joint_state[i];
        memset(st, 0, sizeof(*st));
        const JointUnit_t *j = &g_joint[i];

        /* §A 温度系数拷贝出厂默认 + 阈值配置 */
        memcpy(st->drift.temp_coeff, g_default_temp_coeff, sizeof(g_default_temp_coeff));
        switch (j->type) {
        case JOINT_TYPE_LEG_HIP:
            /* 髋关节负载大, 阈值放宽 */
            st->drift.thr_tau_zero_load = 0.10f;   /* 100 mNm */
            st->drift.thr_vel_zero_load = 0.02f;   /* ~1.15 °/s */
            break;
        case JOINT_TYPE_ARM_SHOULDER:
        case JOINT_TYPE_ARM_ELBOW:
        default:
            st->drift.thr_tau_zero_load = 0.05f;   /* 50 mNm */
            st->drift.thr_vel_zero_load = 0.01f;   /* ~0.57 °/s */
            break;
        }

        /* §B 默认物理参数 (由关节类型决定) */
        joint_params_set_default(&st->params, j->type);

        /* 默认: ESO3 关闭, 系统继续走 legacy ABO (向后兼容)
         * 可在 runtime 由用户/配置包置 1 启用 ESO3 路径. */
        st->eso_enable = 0u;
    }

    /* 2. 启动摘要打印 (零 IMU 漂移包摘要) */
    printf("[JOINT] %u joints registered:", (unsigned)JOINT_COUNT);
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        printf(" [%u]%s(b%d,id0x%02lX,ESO=%u)", (unsigned)i, g_joint[i].name,
               (unsigned)g_joint[i].bus, (unsigned long)g_joint[i].motor_id,
               (unsigned)g_joint_state[i].eso_enable);
    }
    printf("\r\n");
#if DRIFT_IMMUNE_ENABLE
    printf("[DRIFT] Immune package ON: §A torque zero-drift + §B ESO3 adaptive beta3\r\n");
#else
    printf("[DRIFT] Immune package OFF (DRIFT_IMMUNE_ENABLE=0)\r\n");
#endif
}

/* ===== 基础访问器实现 ===== */
const JointUnit_t *joint_get(uint8_t idx)
{
    if (idx >= JOINT_COUNT) return NULL;
    return &g_joint[idx];
}

uint8_t joint_count(void)
{
    return JOINT_COUNT;
}

JointStatus_t *joint_status_ptr(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? j->status : NULL;
}

ABOState_t *joint_abo_ptr(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? j->abo : NULL;
}

int32_t *joint_assist_ptr(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? j->assist_torque : NULL;
}

JointUnitState_t *joint_runtime_ptr(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? j->state : NULL;
}

CAN_HandleTypeDef *joint_bus_handle(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    if (!j) return NULL;
    return (j->bus == JOINT_BUS_CAN1) ? &hcan1 : &hcan2;
}

JointType_e joint_type(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? j->type : JOINT_TYPE_LEG_HIP;
}

uint8_t joint_is_leg(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? (j->type == JOINT_TYPE_LEG_HIP) : 0;
}

uint8_t joint_is_arm(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    return j ? (j->type != JOINT_TYPE_LEG_HIP) : 0;
}

uint8_t joint_idx_from_motor_id(uint32_t motor_id)
{
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        if (g_joint[i].motor_id == motor_id) return i;
    }
    return 0xFF;
}

void joint_push_tx(uint8_t idx, int32_t target, uint8_t mode)
{
    const JointUnit_t *j = joint_get(idx);
    if (!j) return;
    CanTxEntry_t e;
    e.bus       = (uint8_t)j->bus;
    e.motor_id  = (uint8_t)j->motor_id;
    e.target    = target;
    e.mode      = mode;
    can_tx_fifo_write(&e);
}

void joint_disable_hw(uint8_t idx)
{
    const JointUnit_t *j = joint_get(idx);
    if (!j) return;
    HAL_GPIO_WritePin(j->en_port, j->en_pin, GPIO_PIN_RESET);
}

#if JOINT_LOG_ENABLE
/* v2.0 补丁3: 串口导出环形缓冲为 CSV (调试用, 按需调用) */
void joint_log_dump_csv(void)
{
    /* 从最旧帧(head 即下一写入位置=最旧)开始顺序导出 */
    printf("[JOINT_LOG] csv begin, %u frames\r\n", (unsigned)JOINT_LOG_BUF_SIZE);
    printf("ts_ms,joint,phase,pos_rad,vel_rad_s,tau_meas,tau_human,tau_cmd,adm_scale\r\n");
    uint16_t idx = g_log_head;
    for (uint16_t n = 0u; n < JOINT_LOG_BUF_SIZE; n++) {
        const joint_unit_log_frame_t *f = &g_log_buf[idx];
        printf("%lu,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
               (unsigned long)f->ts_ms, (unsigned)f->joint_idx, (unsigned)f->phase,
               (double)f->pos_rad, (double)f->vel_rad_s, (double)f->tau_meas,
               (double)f->tau_human, (double)f->tau_cmd, (double)f->adm_scale);
        idx = (uint16_t)((idx + 1u) % JOINT_LOG_BUF_SIZE);
    }
    printf("[JOINT_LOG] csv end\r\n");
}
#else
void joint_log_dump_csv(void) { /* JOINT_LOG_ENABLE=0: 空实现 */ }
#endif
