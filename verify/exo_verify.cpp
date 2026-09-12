// exo_verify.cpp — 外骨骼 1+2+3 助力算法参考实现 + 十万次蒙特卡洛验证台
//
//  1 = 摩擦补偿(速度前馈)   2 = 重力补偿(mgl*cos(θ-θ0), θ0 上电自标定)   3 = ESO3 扰动观测放大
//  被控对象/传感用真实硬件参数(手册): 减速比 7.75、kt、库仑摩擦 1.9~2.8N·m、
//  14bit 编码器量化、速度滤波、CAN 延迟、电机力矩环滞后; 人体输入取 de Leva 1996 + 关节力矩文献范围。
//
//  验证目标: 在随机参数域 + 边界工况下, 逐条检查不变量, 统计失败率与失败分类。
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>
#include <string>
#include <omp.h>

static const double DT = 1e-3;          // 1 kHz 控制周期
static const double G_ACC = 9.81;

/* ---------------- 控制器参数(将被随机化/扫描) ---------------- */
struct CtrlParams {
    double mgl;          // 重力矩幅值 N·m (可由臂质量×质心估出)
    double theta0;       // 重力零位 rad (上电自由下垂自动标定值)
    double fc_coul;      // 库仑摩擦 N·m
    double fc_visc;      // 粘性摩擦 N·m·s/rad
    double k_fric;       // 摩擦补偿速度前馈增益(1.0=全补偿)
    double G_assist;     // 人力放大倍数
    double w0;           // ESO 带宽 rad/s
    double tlimit;       // 放大助力限幅 N·m (人体放大, 安全上限)
    double ff_limit;     // 重力/摩擦前馈限幅 N·m (须够补满重力, 取电机峰值)
    double total_limit;  // 总输出限幅 N·m (电机峰值/安全上限)
    double deadzone;     // 速度死区 rad/s (摩擦补偿用)
    double fric_lpf_a;   // 摩擦补偿输出 LPF 系数(0~1)
    double assist_lpf_a; // 放大输出 LPF 系数(0~1)
    double hp_alpha;     // 残差高通系数(去慢变模型误差, 只放动态人力)
    double J_hat;        // 惯量估计(ESO 用)
};

/* ---------------- 控制器状态 ---------------- */
struct CtrlState {
    double z1, z2, z3;   // ESO 三阶状态
    double vel_prev;
    double fric_state;   // 摩擦补偿 LPF 状态
    double assist_state; // 放大 LPF 状态
    double hp_state;     // 残差高通状态(抑制慢变模型误差)
    double tau_cmd_prev; // 上一拍总指令(ESO 已知输入, 必须含放大项! 否则自反馈)
};

static inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===== 1+2+3 控制器单拍(1ms): 返回本拍下发力矩(N·m) ===== */
static double ctrl_step(const CtrlParams *p, CtrlState *s,
                        double theta, double omega, double dt)
{
    /* --- 1. 摩擦补偿 (速度前馈, 稳定前馈, 不构成反馈) --- */
    double w = omega;
    double sgn = (w > 1e-4) ? 1.0 : (w < -1e-4 ? -1.0 : 0.0);
    double fric_raw = p->k_fric * (p->fc_coul * sgn + p->fc_visc * w);
    if (fabs(w) < p->deadzone) fric_raw = 0.0;      // 死区: 静止不补偿(防抖动/自激)
    s->fric_state += p->fric_lpf_a * (fric_raw - s->fric_state);

    /* --- 2. 重力补偿 (位置前馈, 依赖 θ0 自标定) --- */
    double tau_grav = p->mgl * cos(theta - p->theta0);

    /* --- 3. ESO3: 观测真外部扰动(人力 + 模型误差) ---
     * 关键: 已知输入必须是"上一拍下发的总指令"(含重力/摩擦/放大全部),
     *       若漏掉放大项, 放大输出会被当成外部扰动 → 自反馈发散(已由验证器抓到)。 */
    double b0 = 1.0 / p->J_hat;
    double e = s->z1 - theta;
    double b1 = 3.0 * p->w0, b2 = 3.0 * p->w0 * p->w0, b3 = p->w0 * p->w0 * p->w0;
    s->z1 += dt * (s->z2 - b1 * e);
    s->z2 += dt * (s->z3 + b0 * s->tau_cmd_prev - b2 * e);
    s->z3 += dt * (-b3 * e);
    /* ESO 数值防护: 高带宽+延迟下会发散 → 检测到非有限/超界即复位, 防止 0*inf=NaN 污染输出 */
    if (!std::isfinite(s->z1) || !std::isfinite(s->z2) || !std::isfinite(s->z3) ||
        fabs(s->z3) > 1e4 || fabs(s->z2) > 1e4) { s->z1 = theta; s->z2 = omega; s->z3 = 0.0; }
    double tau_ext = p->J_hat * s->z3;              // 外部扰动估计(人力 + 标定残差)

    /* 只放大动态分量: 高通滤掉慢变的模型误差/重力标定残差, 避免把 θ0 误差放大成漂移 */
    s->hp_state += p->hp_alpha * (tau_ext - s->hp_state);
    double tau_human_dyn = tau_ext - s->hp_state;

    double amp_raw = p->G_assist * tau_human_dyn;
    s->assist_state += p->assist_lpf_a * (amp_raw - s->assist_state);

    /* --- 合成 + 分离限幅 ---
     * 关键: 重力/摩擦前馈必须能提供完整重力矩(肩 12.6 N·m), 不能被"助力限幅(6N·m)"卡住,
     *       否则重力只补一半 → 臂坠落(已由单轨迹 dump 抓到)。 */
    double tau_ff = clampd(tau_grav + s->fric_state, -p->ff_limit, p->ff_limit);
    double tau_a  = clampd(s->assist_state, -p->tlimit, p->tlimit);
    double tau_out = clampd(tau_ff + tau_a, -p->total_limit, p->total_limit);
    s->tau_cmd_prev = tau_out;
    return tau_out;
}

/* ---------------- 被控对象 + 传感(真实参数) ---------------- */
struct PlantParams {
    double J;            // 臂+负载惯量 kg·m²
    double mgl_true;     // 真实重力矩 N·m
    double theta0_true;  // 真实重力零位 rad
    double fc_true;      // 真实库仑摩擦 N·m
    double bv_true;      // 真实粘性摩擦 N·m·s/rad
    double tau_loop;     // 电机力矩环时间常数 s
    double delay_ms;     // CAN 反馈延迟
    double sigma_v;      // 测速噪声 rad/s
    double vq;           // 速度量化 rad/s
    double wmax;         // 关节速度上限(安全边界)
};

/* 人体输入模式 */
struct HumanInput {
    int    mode;         // 0=正弦摆荡 1=含二次谐波 2=步态样屈快伸慢 3=摆荡+直流偏置 4=零(自激测试)
    double amp_pos;      // 意图摆荡幅值 rad (肢体摆幅)
    double freq;         // Hz
    double t_on, t_off;  // s
    double Kh;           // 人体关节刚度 N·m/rad
    double Bh;           // 人体关节阻尼 N·m·s/rad
    double tau_max;      // 人体最大力矩 N·m
};

struct TrialResult {
    int    nan_flag;
    int    run_flag;        // 失控(|ω|>wmax 或 |θ|>10)
    int    clamp_flag;      // 超限幅
    int    selfex_flag;     // 零人力不收敛
    int    grav_flag;       // 重力补偿失效(|θ_final-θ_hang|>阈值)
    int    dir_flag;        // 助力方向错
    double max_w, max_th, max_tau;
    double amp_meas;        // 实测放大倍数
    double fail_code;       // 失败时记录的主因参数
};

/* 单次试验 */
static TrialResult run_trial(const CtrlParams *cp, const PlantParams *pp,
                             const HumanInput *hi, double th0_init, double w0_init,
                             std::mt19937_64 *rng)
{
    TightLoop:;
    TrialResult r; memset(&r, 0, sizeof(r));
    std::normal_distribution<double> nd(0.0, pp->sigma_v);

    CtrlState cs; memset(&cs, 0, sizeof(cs));
    cs.z1 = th0_init; cs.z2 = w0_init;

    double theta = th0_init, omega = w0_init, tau_act = 0.0;
    int n = 3000;                          // 3 s
    int dly = (int)(pp->delay_ms / (DT * 1000.0));
    if (dly < 0) dly = 0;
    std::vector<double> buf_v(dly + 1, 0.0), buf_th(dly + 1, th0_init);
    double th_hang = pp->theta0_true;      // 自由下垂平衡位(重力零位)
    double th_muscle = 0.0;                // 人体力矩(一阶肌肉模型)状态

    // 方向相关性统计
    double sum_wh = 0.0, sum_wa = 0.0;
    double max_w = 0.0, max_th = 0.0, max_tau = 0.0;

    for (int k = 0; k < n; k++) {
        double t = k * DT;
        // ---- 人体输入: 阻抗跟踪周期性意图轨迹(物理自洽: 力矩受刚度/阻尼与上限约束) ----
        double th_target = 0.0;
        double t_act = t - hi->t_on;
        if (hi->mode != 4 && t >= hi->t_on && t < hi->t_off) {
            double w = 2 * M_PI * hi->freq;
            double th_ref = 0.0, dth_ref = 0.0;
            switch (hi->mode) {
                case 0:   // 纯正弦摆荡
                    th_ref  = hi->amp_pos * sin(w * t_act);
                    dth_ref = hi->amp_pos * w * cos(w * t_act);
                    break;
                case 1:   // 含二次谐波(屈伸不对称)
                    th_ref  = hi->amp_pos * (0.75 * sin(w * t_act) + 0.25 * sin(2 * w * t_act));
                    dth_ref = hi->amp_pos * w * (0.75 * cos(w * t_act) + 0.5 * cos(2 * w * t_act));
                    break;
                case 2: { // 步态样: 屈曲快、伸展慢(周期性曲折)
                    double ph = fmod(hi->freq * t_act, 1.0);
                    if (ph < 0.4) { th_ref = hi->amp_pos * sin(M_PI * ph / 0.4);
                                    dth_ref = hi->amp_pos * M_PI / 0.4 * cos(M_PI * ph / 0.4) * hi->freq * 2 * M_PI / (2 * M_PI); }
                    else          { th_ref = -hi->amp_pos * sin(M_PI * (ph - 0.4) / 0.6);
                                    dth_ref = -hi->amp_pos * M_PI / 0.6 * cos(M_PI * (ph - 0.4) / 0.6) * hi->freq * 2 * M_PI / (2 * M_PI); }
                    dth_ref *= 1.0;
                    break; }
                default:  // 周期摆荡 + 直流偏置(一边抗重力一边摆动)
                    th_ref  = hi->amp_pos * (0.5 + 0.5 * sin(w * t_act));
                    dth_ref = hi->amp_pos * 0.5 * w * cos(w * t_act);
                    break;
            }
            // 人体关节阻抗: τ = Kh(θref−θ) + Bh(θ̇ref−θ̇), 并按人体最大力矩限幅
            th_target = clampd(hi->Kh * (th_ref - theta) + hi->Bh * (dth_ref - omega),
                               -hi->tau_max, hi->tau_max);
        }
        th_muscle += (th_target - th_muscle) * (DT / 0.05);   // 肢体动态上升 ~50ms
        double th = th_muscle;
        // ---- 传感: 量化 + 噪声 + 延迟 ----
        double v_q = round(omega / pp->vq) * pp->vq;
        double v_noisy = v_q + nd(*rng);
        double th_q = round(theta / (pp->vq * DT)) * (pp->vq * DT);
        double v_fb = buf_v.front(); buf_v.erase(buf_v.begin()); buf_v.push_back(v_noisy);
        double th_fb = buf_th.front(); buf_th.erase(buf_th.begin()); buf_th.push_back(th_q);

        // ---- 控制器 ----
        double tau_cmd = ctrl_step(cp, &cs, th_fb, v_fb, DT);

        // ---- 电机力矩环一阶滞后 (用精确指数离散, 避免 DT/tau>1 时显式欧拉数值自激) ----
        double a_loop = exp(-DT / pp->tau_loop);
        tau_act = tau_cmd + (tau_act - tau_cmd) * a_loop;

        // ---- 被控对象 ----
        double coul = (omega > 1e-6) ? pp->fc_true : (omega < -1e-6 ? -pp->fc_true : 0.0);
        double acc = (tau_act + th - pp->bv_true * omega - coul
                      - pp->mgl_true * cos(theta - pp->theta0_true)) / pp->J;
        omega += acc * DT;
        theta += omega * DT;

        if (!std::isfinite(theta) || !std::isfinite(omega) || !std::isfinite(tau_cmd)) { r.nan_flag = 1; break; }
        if (fabs(omega) > max_w) max_w = fabs(omega);
        if (fabs(theta) > max_th) max_th = fabs(theta);
        if (fabs(tau_cmd) > max_tau) max_tau = fabs(tau_cmd);
        if (fabs(tau_cmd) > cp->total_limit + 1e-6) r.clamp_flag = 1;
        sum_wh += th * omega; sum_wa += th * tau_cmd;
    }

    r.max_w = max_w; r.max_th = max_th; r.max_tau = max_tau;
    /* 稳定性判定只对"零人力"工况: 人推工况下关节被人体驱动变快不属于控制器缺陷 */
    if (hi->mode == 4 && (fabs(omega) > 0.5 || fabs(theta - th0_init) > 0.30)) r.run_flag = 1;
    /* 增益异常/发散的通用兜底: 任何工况下出现极端速度都算失控 */
    if (fabs(omega) > 60.0) r.run_flag = 1;
    // 零人力: 收敛性(末段速度应很小)
    if (hi->mode == 4 && fabs(omega) > 0.5) r.selfex_flag = 1;
    // 重力补偿: 无人力时臂应"零重力"停在释放位附近(不坠不漂)
    if (hi->mode == 4 && fabs(theta - th0_init) > 0.30) r.grav_flag = 1;
    // 方向: 人力与助力应同号相关
    if (hi->mode == 0 && hi->amp_pos > 0.05) {
        if (sum_wh * sum_wa < 0.0) r.dir_flag = 1;
    }
    return r;
}

/* ---------------- 蒙特卡洛 ---------------- */
int main(int argc, char **argv)
{
    long N = (argc > 1) ? atol(argv[1]) : 100000;
    unsigned seed = (argc > 2) ? (unsigned)atoi(argv[2]) : 12345;
    /* 分层开关: G_fixed(<0=随机/0=只1+2) grav_en fric_en w0_max delay_max */
    double g_G = (argc > 3) ? atof(argv[3]) : -1.0;
    int g_grav = (argc > 4) ? atoi(argv[4]) : 1;
    int g_fric = (argc > 5) ? atoi(argv[5]) : 1;
    double g_w0max = (argc > 6) ? atof(argv[6]) : 100.0;
    double g_dmax  = (argc > 7) ? atof(argv[7]) : 12.0;
    double g_th0err= (argc > 8) ? atof(argv[8]) : 0.26;   // θ0 标定误差 rad
    double g_mglerr= (argc > 9) ? atof(argv[9]) : 0.2;    // mgl 估计误差比例

    long n_nan = 0, n_run = 0, n_clamp = 0, n_self = 0, n_grav = 0, n_dir = 0, n_fail = 0;
    double worst_margin = 1e9;
    std::vector<double> fail_th0, fail_delay, fail_w0;

    double t_start = omp_get_wtime();
#pragma omp parallel
    {
        std::mt19937_64 rng(seed + omp_get_thread_num() * 7919);
        std::uniform_real_distribution<double> U(0.0, 1.0);
#pragma omp for reduction(+:n_nan,n_run,n_clamp,n_self,n_grav,n_dir,n_fail)
        for (long i = 0; i < N; i++) {
            CtrlParams cp; PlantParams pp; HumanInput hi;
            // ---- 随机化(真实范围) ----
            pp.J        = 0.05 + 0.25 * U(rng);
            pp.mgl_true = 5.0 + 15.0 * U(rng);
            pp.theta0_true = -0.6 + 1.2 * U(rng);
            pp.fc_true  = 1.5 + 1.5 * U(rng);
            pp.bv_true  = 0.02 + 0.18 * U(rng);
            pp.tau_loop = (0.3 + 1.2 * U(rng)) * 1e-3;
            pp.delay_ms = g_dmax * U(rng);
            pp.sigma_v  = 0.06 * U(rng);
            pp.vq       = 1.745e-5 * (1.0 + 9.0 * U(rng));   // 编码器量化
            pp.wmax     = 12.0;

            // 控制器: mgl/θ0 有估计误差(标定残差)
            cp.mgl    = g_grav ? (pp.mgl_true * (1.0 - g_mglerr + 2.0 * g_mglerr * U(rng))) : 0.0;
            cp.theta0 = pp.theta0_true + g_th0err * (2.0 * U(rng) - 1.0);
            cp.fc_coul = pp.fc_true;
            cp.fc_visc = pp.bv_true;
            cp.k_fric  = g_fric ? (0.7 * U(rng)) : 0.0;   // 保守: ≤0.7(避免过补偿粘性项→负阻尼)
            cp.G_assist= (g_G >= 0.0) ? g_G : (0.0 + 1.5 * U(rng));
            cp.w0      = 20.0 + (g_w0max - 20.0) * U(rng);
            cp.tlimit  = 6.0;                       // 放大助力上限(RS01 额定 6 N·m)
            cp.ff_limit= 17.0;                      // 前馈上限(RS01 峰值 17 N·m, 够补肩 12.6)
            cp.total_limit = 17.0;                  // 总输出上限(电机峰值)
            cp.deadzone= 0.15;
            cp.fric_lpf_a  = 0.15 + 0.5 * U(rng);
            cp.assist_lpf_a= 0.05 + 0.3 * U(rng);
            cp.hp_alpha    = 0.002 + 0.03 * U(rng);   // 高通(去慢变误差)
            cp.J_hat   = pp.J * (0.6 + 0.8 * U(rng));
            /* 设计约束: ESO 带宽必须与回路延迟匹配 (w0*delay<~0.5), 否则数值发散 */
            {
                double d_s = pp.delay_ms * 1e-3 + pp.tau_loop;   // 总延迟
                double w0_lim = 0.45 / d_s;
                if (cp.w0 > w0_lim) cp.w0 = w0_lim;
                if (cp.w0 < 5.0) cp.w0 = 5.0;
            }

            // 人体输入: 20% 静止(自激测试), 其余为【阻抗跟踪周期性摆荡/屈伸】
            double u = U(rng);
            if (u < 0.20) { hi.mode = 4; hi.amp_pos = 0; hi.t_on = 0; hi.t_off = 1; hi.freq = 0;
                            hi.Kh = 0; hi.Bh = 0; hi.tau_max = 0; }
            else {
                hi.mode    = (int)(U(rng) * 4);
                hi.amp_pos = 0.2 + 0.8 * U(rng);      // 肢体摆幅 0.2~1.0 rad (11~57°)
                hi.freq    = 0.5 + 2.0 * U(rng);      // 0.5~2.5 Hz (步行/摆臂频段)
                hi.t_on    = 0.2 * U(rng);
                hi.t_off   = 2.5;
                hi.Kh      = 20.0 + 40.0 * U(rng);    // 人体关节刚度
                hi.Bh      = 1.0 + 2.0 * U(rng);      // 人体关节阻尼
                hi.tau_max = 20.0 + 20.0 * U(rng);    // 人体最大力矩 20~40 N·m
            }
            double th0_i = -0.5 + 1.0 * U(rng);
            double w0_i  = -0.5 + 1.0 * U(rng);

            TrialResult r = run_trial(&cp, &pp, &hi, th0_i, w0_i, &rng);
            n_nan += r.nan_flag; n_run += r.run_flag; n_clamp += r.clamp_flag;
            n_self += r.selfex_flag; n_grav += r.grav_flag; n_dir += r.dir_flag;
            int f = r.nan_flag | r.run_flag | r.clamp_flag | r.selfex_flag | r.grav_flag | r.dir_flag;
            if (f) {
#pragma omp critical
                { fail_th0.push_back(cp.theta0 - pp.theta0_true); fail_delay.push_back(pp.delay_ms); fail_w0.push_back(cp.w0); }
                n_fail++;
            }
            // 安全裕度: 最快多少接近失控
            double m = pp.wmax / (r.max_w + 1e-9);
#pragma omp critical
            { if (m < worst_margin) worst_margin = m; }
        }
    }
    double tel = omp_get_wtime() - t_start;

    printf("==== 1+2+3 算法验证: %ld 次蒙特卡洛 (seed=%u, %d 线程) ====\n", N, seed, omp_get_max_threads());
    printf("配置: G_assist=%s, 重力补偿=%s, 摩擦补偿=%s, ESO带宽≤%.0f, 延迟≤%.0fms, θ0误差±%.1f°, mgl误差±%.0f%%\n",
           g_G >= 0.0 ? (g_G == 0.0 ? "关(只测1+2)" : "固定") : "随机0~1.5",
           g_grav ? "开" : "关", g_fric ? "开" : "关", g_w0max, g_dmax,
           g_th0err * 57.2958, g_mglerr * 100.0);
    printf("总耗时 %.2f s (%.0f 次/s)\n\n", tel, N / tel);
    printf("失败统计(次数/占比):\n");
    printf("  数值异常(NaN/Inf)      : %ld  (%.3f%%)\n", n_nan, 100.0*n_nan/N);
    printf("  失控(|ω|>12或|θ|>10)   : %ld  (%.3f%%)\n", n_run, 100.0*n_run/N);
    printf("  超限幅(>±6 N·m)        : %ld  (%.3f%%)\n", n_clamp, 100.0*n_clamp/N);
    printf("  零人力不收敛(自激)     : %ld  (%.3f%%)\n", n_self, 100.0*n_self/N);
    printf("  重力补偿失效(>0.30rad) : %ld  (%.3f%%)\n", n_grav, 100.0*n_grav/N);
    printf("  助力方向错             : %ld  (%.3f%%)\n", n_dir, 100.0*n_dir/N);
    printf("  ---- 任一失败          : %ld  (%.3f%%)\n", n_fail, 100.0*n_fail/N);
    printf("\n安全裕度: 最快工况 max|ω| = %.2f rad/s (上限 12) → 裕度 %.2fx\n",
           12.0 / worst_margin, worst_margin);
    if (!fail_th0.empty()) {
        double s=0,a=0,b=0;
        for (size_t i=0;i<fail_th0.size();i++){ s+=fabs(fail_th0[i]); a+=fail_delay[i]; b+=fail_w0[i]; }
        size_t m=fail_th0.size();
        printf("失败样本均值: |θ0误差| = %.3f rad (%.1f°), 延迟 = %.1f ms, ESO带宽 = %.0f rad/s\n",
               s/m, s/m*57.2958, a/m, b/m);
    }
    return n_fail > 0 ? 1 : 0;
}
