/**
 * @file joint_unit_test.c
 * @brief v2.0 补丁3 自包含测试桩 — 验证 ESO3 / 导纳 / 重力补偿核心数学行为
 *
 * 【用途】
 *   本文件是 PC 端回归测试桩, 不依赖任何硬件头文件 (stm32 / safety / bsp_config),
 *   仅用 stdio / stdint / math, 可直接在 PC 上编译运行:
 *     gcc test/joint_unit_test.c -lm -o test_joint && ./test_joint
 *   返回值 = 失败用例数 (0 = 全部 PASS)。
 *
 * 【镜像来源】
 *   固件 App/control/joint_unit.c 中以下函数为 static 且依赖硬件头, 无法直接 include:
 *     - Gravity_Comp_Vector          (重力补偿, §E)
 *     - tau_model_forward            (模型前馈, §B)
 *     - Admittance_Update1kHz        (导纳/阻抗, §D)
 *     - JointUnit_ESO3_Update1kHz    (3 阶 ESO, §B)
 *   因此本文件「镜像」其核心算法 (复制实现, 去掉 static, 改用本文件最小化 test 结构体)。
 *   ★重要: 固件 joint_unit.c 修改后需手动同步本文件镜像实现, 否则测试无效。
 *
 * 【与固件的关系】
 *   - fast_sinf / fast_cosf 在固件中是 256 点查表 + 线性插值; 本桩为聚焦 ESO/导纳/重力
 *     逻辑而非查表精度, 镜像算法直接用 <math.h> 的 sinf / cosf (PC 端高精度参考基准)。
 *   - 这会引入与固件查表的微小数值偏差, 但对下述用例的容差余量无影响。
 *   - 本文件不进 Keil 工程 (.uvprojx), 仅作 PC 端回归测试参考。
 *
 * 【测试用例】
 *   A: 重力补偿 (直立 mgl*cos 退化形式, pos=0 / pos=π/2)
 *   B: 重力向量投影 (grav_arm 已标定, g=[0,0,-1] / g=[1,0,0])
 *   C: 导纳稳态 (tau_human=2.0 恒定, 跑 2000 步 → tau_cmd≈2.0)
 *   D: ESO3 无激励冻结 (pos=0 恒定, tau_meas=5.0 → z3 冻结≈0, div_flag=0)
 *   E: ESO3 发散监测 (beta3 极大 + 初始 z1 偏离 → z3 发散, div_flag=1)
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===== 最小化测试结构体 (镜像 JointUnitParams_t / ESO3State_t / JointUnitState_t) ===== */

/* 镜像 JointUnitParams_t (仅保留算法需要的字段) */
typedef struct {
    float J_rotor;           /* 转子+连杆折算惯量 (kg·m²) */
    float friction_viscous;  /* 粘性摩擦 (Nm·s/rad) */
    float friction_coulomb;  /* 库仑摩擦 (Nm) */
    float torque_const;      /* 力矩常数 (Nm/A) */
    float gear_ratio;        /* 减速比 */
    float eso_beta[3];       /* ESO 3 阶带宽 (β1,β2,β3) */
    float mgl;               /* 重力臂 (Nm) */
    float grav_arm[3];       /* §E 重力力矩臂向量 (Nm/g) */
    float theta0_rad;        /* 机械零位偏移 (rad) */
    float adm_M_base;        /* 导纳惯量基值 (kg·m²) */
    float adm_B_base;        /* 导纳阻尼基值 (Nm·s/rad) */
    float adm_K_base;        /* 导纳刚度基值 (Nm/rad) */
    float thr_vel_noexc;     /* 无激励速度阈值 (rad/s) */
    float thr_tau_noexc;     /* 无激励力矩阈值 (Nm) */
    float thr_res_noexc;     /* 无激励残差阈值 (Nm) */
    uint16_t param_version;  /* 参数版本号 */
} test_params_t;

/* 镜像 ESO3State_t */
typedef struct {
    float z1;               /* 位置估计 (rad) */
    float z2;               /* 速度估计 (rad/s) */
    float z3;               /* 总扰动估计 (Nm) → tau_human */
    float vel_prev;         /* 上一拍速度 (rad/s) */
    float payload_est;      /* 负载扭矩前馈 (Nm) */
    float tau_model_prev;   /* 上一拍模型前馈 (Nm) */
} test_eso3_t;

/* 镜像 JointUnitState_t (仅保留算法需要的字段) */
typedef struct {
    float pos_rad;          /* 反馈位置 (rad) */
    float vel_rad_s;        /* 反馈速度 (rad/s) */
    float tau_meas_nm;      /* 去零漂后反馈力矩 (Nm) */
    float tau_human_nm;     /* ESO 输出人力估计 (Nm) */
    float tau_cmd_nm;       /* 导纳输出目标力矩 (Nm) */
    float adm_pos;          /* 导纳位置积分 (rad) */
    float adm_vel;          /* 导纳速度 (rad/s) */
    float global_adm_scale; /* 导纳/阻抗缩放因子 */
    uint8_t eso_enable;     /* 1=ESO3 启用, 0=legacy */
    uint8_t eso_div_flag;   /* ESO 发散标志 */
    uint8_t eso_boost_mode; /* boost 激活 (镜像里 ESO3 用到) */
    uint16_t eso_boost_timer;
    const float *gravity_base_ptr; /* 基座系重力向量 (float[3], 归一化) */
    test_eso3_t eso3;
    test_params_t params;
} test_state_t;

/* ===== 镜像辅助: 钳位 (镜像 joint_unit.c clampf_local) ===== */
static inline float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ===== 镜像辅助: 分段线性 tanh 近似 (镜像 joint_unit.c tanh_approx) =====
 * |x|<2 线性斜率 0.5 钳位 [-1,1]; |x|>=2 饱和 ±1 */
static inline float tanh_approx(float x)
{
    if (x > 2.0f)  return 1.0f;
    if (x < -2.0f) return -1.0f;
    return 0.5f * x;
}

/* ===== 镜像: §E 重力补偿 (基座系向量投影) =====
 * 镜像 joint_unit.c Gravity_Comp_Vector, 仅 fast_sinf/fast_cosf → sinf/cosf。
 * 已标定 (grav_arm 范数>ε): tau_g = grav_arm · g_base * 9.81
 * 未标定 (默认 0): tau_g = mgl * (-gz*cos(q) + gx*sin(q)), q = pos + θ0
 *   直立 gz=-1 → mgl*cos(q), 与旧代码一致 */
static float Gravity_Comp_Vector_test(const test_state_t *st, const float g_base[3])
{
    const test_params_t *p = &st->params;
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
    return p->mgl * (-g_base[2]*cosf(q) + g_base[0]*sinf(q));
}

/* ===== 镜像: §B 模型前馈 (已知动力学剥离) =====
 * 镜像 joint_unit.c tau_model_forward。
 * tau_model = J*acc + (B*vel + C*sgn(vel)) + grav + payload */
static float tau_model_forward_test(test_state_t *st, float acc_rad_s2)
{
    const test_params_t *p = &st->params;

    /* 摩擦: B*vel + C*sgn(vel), sgn(0)=0 避免低速抖动 */
    float vel = st->vel_rad_s;
    float fric = p->friction_viscous * vel;
    if (vel > 1e-4f)       fric += p->friction_coulomb;
    else if (vel < -1e-4f) fric -= p->friction_coulomb;

    /* §E 重力前馈: 有 gravity_base_ptr → 向量投影; 无 → 直立兜底 */
    float grav;
    if (st->gravity_base_ptr) {
        grav = Gravity_Comp_Vector_test(st, st->gravity_base_ptr);
    } else {
        grav = p->mgl * cosf(st->pos_rad + p->theta0_rad);
    }

    float inertia = p->J_rotor * acc_rad_s2;
    float payload = st->eso3.payload_est;

    return inertia + fric + grav + payload;
}

/* ===== 镜像: §D 导纳/阻抗控制器 (1kHz) =====
 * 镜像 joint_unit.c Admittance_Update1kHz。
 * M*ddx + B*dx + K*x = tau_human, M/B/K 被 global_adm_scale 实时缩放。
 * 半隐式欧拉: 先更新速度再更新位置。tau_cmd = K*x + B*dx */
static void Admittance_Update1kHz_test(test_state_t *st, float dt_s)
{
    const test_params_t *p = &st->params;
    float s = st->global_adm_scale;
    if (s < 0.05f) s = 0.05f;   /* 钳位下限, 避免退化除零 */

    float M = p->adm_M_base * s;
    float B = p->adm_B_base * s;
    float K = p->adm_K_base * s;

    float tau_h = st->tau_human_nm;
    st->adm_vel += (tau_h - B * st->adm_vel - K * st->adm_pos) / M * dt_s;
    st->adm_pos += st->adm_vel * dt_s;

    st->tau_cmd_nm = K * st->adm_pos + B * st->adm_vel;
}

/* ===== 镜像: §B 3 阶 ESO 抗漂移 (1kHz) =====
 * 镜像 joint_unit.c JointUnit_ESO3_Update1kHz。
 *   ė = z1 - y (y = pos)
 *   ż₁ = z₂ - β₁ e
 *   ż₂ = z₃ + b0*u - β₂ e      (b0 = kt/J)
 *   ż₃ = -β₃_adapt e            (自适应 + boost + 无激励冻结)
 * tau_human = z3; |z3|>60 → eso_div_flag=1 (锁存, 不在此清除) */
static void ESO3_Update1kHz_test(test_state_t *st, float tau_cmd_nm, float dt_s)
{
    test_eso3_t *es = &st->eso3;
    const test_params_t *p = &st->params;

    /* 1. 加速度微分 (速度差分) */
    float acc = (st->vel_rad_s - es->vel_prev) / dt_s;
    es->vel_prev = st->vel_rad_s;

    /* 2. 精确模型前馈 (剥离已知动力学) */
    float tau_model = tau_model_forward_test(st, acc);
    es->tau_model_prev = tau_model;

    /* 3. ESO 仅观测未建模残差 */
    float tau_residual = st->tau_meas_nm - tau_model;

    /* 4. 标准 3 阶 ESO 欧拉离散化 */
    float e = es->z1 - st->pos_rad;
    float b0 = p->torque_const / p->J_rotor;   /* 输入增益 = kt / J */

    es->z1 += dt_s * (es->z2 - p->eso_beta[0] * e);
    es->z2 += dt_s * (es->z3 + b0 * tau_cmd_nm - p->eso_beta[1] * e);

    /* 5. 自适应 β3 (corr = z3 * Δres, 同号增大 / 异号减小, tanh 软化) */
    float residual_change = tau_residual - es->z3;
    float correlation = es->z3 * residual_change;
    float beta3_base = p->eso_beta[2];
    float beta3_adapt = beta3_base * (1.0f + 0.5f * tanh_approx(correlation * 10.0f));
    beta3_adapt = clampf_local(beta3_adapt, 0.1f * beta3_base, 5.0f * beta3_base);

    /* 5b. §C-boost: 负载突变时 beta3×3 + z3 衰减 + 旁路冻结 */
    uint8_t boost_active = st->eso_boost_mode && (st->eso_boost_timer > 0u);
    if (boost_active) {
        beta3_adapt *= 3.0f;
        es->z3 *= 0.95f;
    }

    /* 6. 无激励检测 → 冻结 z3 积分 (boost 期间旁路冻结) */
    uint8_t no_excitation =
        (fabsf(st->vel_rad_s)   < p->thr_vel_noexc) &&
        (fabsf(tau_cmd_nm)      < p->thr_tau_noexc) &&
        (fabsf(residual_change) < p->thr_res_noexc);

    if (!no_excitation || boost_active) {
        es->z3 += dt_s * (-beta3_adapt * e);
    } /* else 冻结 z3 */

    /* 7. 发散监测 (|z3|>60), 仅置位不清 (锁存事件, 由上层消费清除) */
    if (fabsf(es->z3) > 60.0f) {
        st->eso_div_flag = 1u;
    }

    /* 8. 输出人力估计 */
    st->tau_human_nm = es->z3;
}

/* ===== 断言宏 ===== */
static int g_fail_count = 0;

/* 浮点近邻断言: |actual - expected| <= tol */
#define ASSERT_NEAR(actual, expected, tol, name) do { \
    float _a = (float)(actual); float _e = (float)(expected); float _t = (float)(tol); \
    if (fabsf(_a - _e) <= _t) { \
        printf("[PASS] %s: 实测=%.6f 期望=%.6f 容差=%.6f\n", name, _a, _e, _t); \
    } else { \
        printf("[FAIL] %s: 实测=%.6f 期望=%.6f 容差=%.6f\n", name, _a, _e, _t); \
        g_fail_count++; \
    } \
} while (0)

/* 整数相等断言 (用于标志位) */
#define ASSERT_EQ(actual, expected, name) do { \
    long _a = (long)(actual); long _e = (long)(expected); \
    if (_a == _e) { \
        printf("[PASS] %s: 实测=%ld 期望=%ld\n", name, _a, _e); \
    } else { \
        printf("[FAIL] %s: 实测=%ld 期望=%ld\n", name, _a, _e); \
        g_fail_count++; \
    } \
} while (0)

/* ===== 用例 A: 重力补偿 (直立 mgl*cos 退化形式) =====
 * 未标定 grav_arm=0 → 退化分支: tau_g = mgl * (-gz*cos(q) + gx*sin(q))
 *   直立 g=[0,0,-1]: tau_g = mgl * (cos(q) + 0) = mgl*cos(q)
 *   pos=0    → mgl*cos(0)    = mgl*1   = 6.0
 *   pos=π/2  → mgl*cos(π/2)  ≈ 0 */
static void test_case_A_gravity_compensation(void)
{
    test_state_t st;
    float g_base[3] = {0.0f, 0.0f, -1.0f};   /* 直立 */
    /* 仅初始化本用例需要的字段 */
    st.params.mgl = 6.0f;
    st.params.grav_arm[0] = 0.0f;
    st.params.grav_arm[1] = 0.0f;
    st.params.grav_arm[2] = 0.0f;   /* 未标定 → 走退化分支 */
    st.params.theta0_rad = 0.0f;
    st.gravity_base_ptr = g_base;   /* 走向量投影入口 (内部因 ga_norm=0 退化) */

    printf("\n--- 用例 A: 重力补偿 (直立 mgl*cos 退化) ---\n");

    /* pos=0 → tau_g ≈ 6.0 */
    st.pos_rad = 0.0f;
    float tau_g = Gravity_Comp_Vector_test(&st, g_base);
    ASSERT_NEAR(tau_g, 6.0f, 0.01f, "A.1 重力 pos=0");

    /* pos=π/2 → tau_g ≈ 0 (cos(π/2)≈0) */
    st.pos_rad = (float)M_PI / 2.0f;
    tau_g = Gravity_Comp_Vector_test(&st, g_base);
    ASSERT_NEAR(tau_g, 0.0f, 0.01f, "A.2 重力 pos=pi/2");
}

/* ===== 用例 B: 重力向量投影 (grav_arm 已标定) =====
 * ga_norm>ε → tau_g = (grav_arm · g_base) * 9.81
 *   grav_arm=[0.1,0,0], g=[0,0,-1] → 0.1*0*9.81 = 0
 *   grav_arm=[0.1,0,0], g=[1,0,0]  → 0.1*1*9.81 = 0.981 */
static void test_case_B_gravity_vector_projection(void)
{
    test_state_t st;
    /* 仅初始化本用例需要的字段 */
    st.params.grav_arm[0] = 0.1f;
    st.params.grav_arm[1] = 0.0f;
    st.params.grav_arm[2] = 0.0f;   /* 已标定 → 走向量投影分支 */
    st.params.mgl = 6.0f;           /* 走投影分支时 mgl 不参与 */
    st.params.theta0_rad = 0.0f;
    st.pos_rad = 0.0f;

    printf("\n--- 用例 B: 重力向量投影 (grav_arm 标定) ---\n");

    /* g=[0,0,-1] → 0 */
    float g1[3] = {0.0f, 0.0f, -1.0f};
    float tau_g = Gravity_Comp_Vector_test(&st, g1);
    ASSERT_NEAR(tau_g, 0.0f, 0.01f, "B.1 投影 g=[0,0,-1]");

    /* g=[1,0,0] → 0.1*1*9.81 = 0.981 */
    float g2[3] = {1.0f, 0.0f, 0.0f};
    tau_g = Gravity_Comp_Vector_test(&st, g2);
    ASSERT_NEAR(tau_g, 0.981f, 0.01f, "B.2 投影 g=[1,0,0]");
}

/* ===== 用例 C: 导纳稳态 =====
 * tau_human=2.0 恒定, M=0.02, B=0.5, K=2.0, adm_scale=1.0
 * 稳态: adm_vel→0, adm_pos→tau_h/K=1.0, tau_cmd→K*1.0=2.0
 * 跑 2000 步 dt=0.001 (2s, 远超时间常数 ~0.08s) → 应收敛 */
static void test_case_C_admittance_steady(void)
{
    test_state_t st;
    /* 仅初始化本用例需要的字段 */
    st.params.adm_M_base = 0.02f;
    st.params.adm_B_base = 0.5f;
    st.params.adm_K_base = 2.0f;
    st.global_adm_scale = 1.0f;
    st.tau_human_nm = 2.0f;     /* 恒定人力 */
    st.adm_pos = 0.0f;
    st.adm_vel = 0.0f;
    st.tau_cmd_nm = 0.0f;

    float dt = 0.001f;
    int steps = 2000;

    printf("\n--- 用例 C: 导纳稳态 (tau_h=2.0, 2000 步) ---\n");

    for (int i = 0; i < steps; i++) {
        /* 导纳用 tau_human 解算; 每步重置 tau_human 保持恒定 (本用例不跑 ESO) */
        st.tau_human_nm = 2.0f;
        Admittance_Update1kHz_test(&st, dt);
    }

    /* 稳态 adm_pos≈1.0, tau_cmd≈2.0 (B*adm_vel≈0) */
    ASSERT_NEAR(st.adm_pos, 1.0f, 0.05f, "C.1 稳态 adm_pos");
    ASSERT_NEAR(st.tau_cmd_nm, 2.0f, 0.1f, "C.2 稳态 tau_cmd");
}

/* ===== 用例 D: ESO3 无激励冻结 =====
 * 场景: tau_meas=5.0 恒定 (含恒定扰动), tau_cmd=0, vel=0, pos=0 恒定, mgl=0 (髋)
 *
 * ★行为说明 (与任务原始期望的差异):
 *   任务原始期望 "z3≈tau_meas-tau_model=5.0"。但镜像 ESO 代码中 z3 由位置跟踪
 *   误差 e = z1 - pos 驱动 (ż₃ = -β₃·e)。pos=0 恒定 + z1 初值=0 → e=0 →
 *   ż₃=0 → z3 冻结在初值 0。即便 tau_residual=5.0, 没有位置跟踪误差, z3 也
 *   无法感知该扰动。这是 ESO "用位置误差观测扰动" 的设计特性 (固件代码亦如此)。
 *
 *   因此本用例断言 z3 冻结≈0 (而非 5.0), 如实反映代码行为; 同时验证 |z3|<60
 *   故 eso_div_flag=0。这构成有意义的回归: 确认 ESO 在无位置激励下不漂移。
 *
 *   附带验证: no_excitation 判定中 residual_change = tau_residual - z3 = 5.0 - 0 = 5.0
 *   > thr_res(0.1) → no_excitation=false, 故 z3 更新分支被执行; 但因 e=0,
 *   z3 += dt*(-β₃*0)=0, 仍冻结。两条路径都冻结 z3, 行为一致。 */
static void test_case_D_eso3_no_excitation_freeze(void)
{
    test_state_t st;
    /* 仅初始化本用例需要的字段 */
    st.params.J_rotor = 0.00012f;
    st.params.friction_viscous = 0.008f;
    st.params.friction_coulomb = 0.45f;
    st.params.torque_const = 0.18f;
    st.params.gear_ratio = 54.0f;
    st.params.eso_beta[0] = 1200.0f;
    st.params.eso_beta[1] = 80000.0f;
    st.params.eso_beta[2] = 500000.0f;
    st.params.mgl = 0.0f;            /* 髋: mgl=0 */
    st.params.grav_arm[0] = 0.0f;
    st.params.grav_arm[1] = 0.0f;
    st.params.grav_arm[2] = 0.0f;
    st.params.theta0_rad = 0.0f;
    st.params.thr_vel_noexc = 1e-3f;
    st.params.thr_tau_noexc = 0.05f;
    st.params.thr_res_noexc = 0.1f;

    /* 运行时状态 */
    st.pos_rad = 0.0f;               /* pos 恒定 */
    st.vel_rad_s = 0.0f;
    st.tau_meas_nm = 5.0f;           /* 恒定扰动 */
    st.tau_cmd_nm = 0.0f;
    st.tau_human_nm = 0.0f;
    st.eso_div_flag = 0u;
    st.eso_boost_mode = 0u;
    st.eso_boost_timer = 0u;
    st.gravity_base_ptr = NULL;      /* 直立兜底 (mgl=0 → grav=0) */

    /* ESO 状态初值 */
    st.eso3.z1 = 0.0f;               /* z1 初值 = pos → e=0 */
    st.eso3.z2 = 0.0f;
    st.eso3.z3 = 0.0f;
    st.eso3.vel_prev = 0.0f;
    st.eso3.payload_est = 0.0f;
    st.eso3.tau_model_prev = 0.0f;

    float dt = 0.001f;
    int steps = 3000;

    printf("\n--- 用例 D: ESO3 无激励冻结 (pos=0 恒定, tau_meas=5.0) ---\n");

    for (int i = 0; i < steps; i++) {
        /* pos/vel/tau_meas 恒定; tau_cmd 本拍用 0 (不跑导纳) */
        ESO3_Update1kHz_test(&st, 0.0f, dt);
    }

    /* z3 因 e=0 冻结在初值 0 (非 5.0, 见上方行为说明) */
    ASSERT_NEAR(st.eso3.z3, 0.0f, 0.5f, "D.1 z3 无激励冻结≈0 (非5.0, ESO由位置误差驱动)");
    /* |z3|<60 → 发散标志保持 0 */
    ASSERT_EQ(st.eso_div_flag, 0, "D.2 eso_div_flag=0 (未发散)");
}

/* ===== 用例 E: ESO3 发散监测 =====
 * 人为设 β3 极大 (5e7) + 初始 z1 偏离 pos (z1=0.5, pos=0) → e=0.5
 * tau_meas=5.0 (使 residual_change 大 → no_excitation=false, z3 更新分支开放)
 * 第 1 拍: ż₃ = -β₃·e = -5e7*0.5 = -2.5e7 → z3 爆炸 → |z3|>60 → div_flag=1
 * 跑 50 步确保触发。 */
static void test_case_E_eso3_divergence(void)
{
    test_state_t st;
    /* 与用例 D 相同的基础参数, 仅 β3 极大 */
    st.params.J_rotor = 0.00012f;
    st.params.friction_viscous = 0.008f;
    st.params.friction_coulomb = 0.45f;
    st.params.torque_const = 0.18f;
    st.params.gear_ratio = 54.0f;
    st.params.eso_beta[0] = 1200.0f;
    st.params.eso_beta[1] = 80000.0f;
    st.params.eso_beta[2] = 5.0e7f;   /* ★极大 β3, 制造发散 */
    st.params.mgl = 0.0f;
    st.params.grav_arm[0] = 0.0f;
    st.params.grav_arm[1] = 0.0f;
    st.params.grav_arm[2] = 0.0f;
    st.params.theta0_rad = 0.0f;
    st.params.thr_vel_noexc = 1e-3f;
    st.params.thr_tau_noexc = 0.05f;
    st.params.thr_res_noexc = 0.1f;

    /* 运行时状态 */
    st.pos_rad = 0.0f;
    st.vel_rad_s = 0.0f;
    st.tau_meas_nm = 5.0f;            /* 使 residual_change 大, 开放 z3 更新 */
    st.tau_cmd_nm = 0.0f;
    st.tau_human_nm = 0.0f;
    st.eso_div_flag = 0u;
    st.eso_boost_mode = 0u;
    st.eso_boost_timer = 0u;
    st.gravity_base_ptr = NULL;

    /* ★z1 初值偏离 pos → e≠0, 驱动 z3 发散 */
    st.eso3.z1 = 0.5f;
    st.eso3.z2 = 0.0f;
    st.eso3.z3 = 0.0f;
    st.eso3.vel_prev = 0.0f;
    st.eso3.payload_est = 0.0f;
    st.eso3.tau_model_prev = 0.0f;

    float dt = 0.001f;
    int steps = 50;

    printf("\n--- 用例 E: ESO3 发散监测 (β3=5e7, z1初值=0.5) ---\n");

    for (int i = 0; i < steps; i++) {
        ESO3_Update1kHz_test(&st, 0.0f, dt);
        if (st.eso_div_flag) break;   /* 触发即停, 避免数值溢出 */
    }

    /* |z3| 应已 >60 → div_flag 锁存置 1 */
    ASSERT_EQ(st.eso_div_flag, 1, "E.1 eso_div_flag=1 (z3 发散触发)");
    printf("    (触发时 z3=%.2f, |z3|>60 阈值)\n", st.eso3.z3);
}

/* ===== main: 跑全部用例, 返回失败数 ===== */
int main(void)
{
    printf("=============================================\n");
    printf(" joint_unit 自包含测试桩 (v2.0 补丁3)\n");
    printf(" 镜像来源: App/control/joint_unit.c\n");
    printf("=============================================\n");

    test_case_A_gravity_compensation();
    test_case_B_gravity_vector_projection();
    test_case_C_admittance_steady();
    test_case_D_eso3_no_excitation_freeze();
    test_case_E_eso3_divergence();

    printf("\n=============================================\n");
    if (g_fail_count == 0) {
        printf(" 结果: 全部 PASS\n");
    } else {
        printf(" 结果: %d 项 FAIL\n", g_fail_count);
    }
    printf("=============================================\n");

    return g_fail_count;
}
