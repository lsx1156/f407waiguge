/**
 * @file mode_manager.c
 * @brief 模式管理器 - 单按键 (WKUP) 状态机 + 平滑模式切换
 *
 * ★ v1.6.8fix: 放弃 KEY0, 仅用 WKUP 单键控制
 * 交互设计:
 *   WKUP (PA0) 短按<500ms: ZERO ↔ GAIT 切换 (进入 GAIT 自动启动步态)
 *   WKUP 长按>3s:          强制回零力矩
 */

#include "mode_manager.h"
#include "./BSP/KEY/key.h"
#include "safety.h"
#include "bsp_config.h"
#include <stdio.h>
#include <string.h>

/* ===== 全局状态 ===== */
ModeManager_t g_mode_mgr = {
    .main_mode      = MODE_ZERO_TORQUE,
    .prev_mode      = MODE_ZERO_TORQUE,
    .gait_sub       = GAIT_SUB_FLAT,
    .arm_level      = ARM_ASSIST_NORM,
    .abo_global_en  = 1,
    .transitioning  = 0,
    .switch_ts      = 0,
};

/* 步态运行标志 (兼容旧接口) */
static volatile uint8_t s_gait_running = 0;
static volatile uint32_t s_gait_start_ms = 0;

/* ===== 按键状态机 (非阻塞) ===== */
#define SHORT_PRESS_MS   500   /* 短按阈值: <500ms */
#define LONG_PRESS_MS    3000  /* 长按阈值: >3s */
#define DEBOUNCE_MS      20

/* 内部标志, 由 scan_key 设置, mode_fsm_process 映射为事件 */
#define EVT_SHORT_FLAG  0x10
#define EVT_LONG_FLAG   0x20

typedef struct {
    uint8_t  state;        /* 0=idle, 1=debounce, 2=pressed, 3=long_fired */
    uint32_t press_ts;
} KeySm_t;

static KeySm_t s_wkup_sm;
/* ★ v1.6.8fix: s_key0_sm 废弃, 不再扫描 KEY0 */

/* 按键扫描, 返回事件 bitmask */
static uint8_t scan_key(KeySm_t *sm, uint8_t pressed, uint32_t now)
{
    uint8_t evt = EVT_NONE;

    switch (sm->state) {
    case 0: /* idle */
        if (pressed) {
            sm->state    = 1;
            sm->press_ts = now;
        }
        break;

    case 1: /* debounce */
        if (now - sm->press_ts >= DEBOUNCE_MS) {
            if (pressed) {
                sm->state    = 2;
                sm->press_ts = now;  /* 重新计时用于长按 */
            } else {
                sm->state = 0;  /* 抖动 */
            }
        }
        break;

    case 2: /* pressed, 等释放或长按 */
        if (!pressed) {
            /* 释放 → 短按 */
            sm->state = 0;
            evt |= EVT_SHORT_FLAG;
        } else if (now - sm->press_ts >= LONG_PRESS_MS) {
            sm->state = 3;
            evt |= EVT_LONG_FLAG;
        }
        break;

    case 3: /* long_fired, 等释放 */
        if (!pressed) {
            sm->state = 0;
        }
        break;
    }
    return evt;
}

/* ===== 模式切换平滑过渡 ===== */
static void mode_exit_cleanup(MainMode_e old_mode)
{
    /* 旧模式退出: 清零目标力矩/位置, 停止步态 */
    s_gait_running = 0;

    /* 重置 ABO 状态 (清零偏置累积, 防止残助力矩) */
    extern void abo_state_reset_all(void);
    abo_state_reset_all();

    /* 清零 ABO 助力力矩输出 */
    extern int32_t g_abo_assist_torque[6];
    for (int i = 0; i < 6; i++) {
        g_abo_assist_torque[i] = 0;
    }

    (void)old_mode;
}

static void mode_enter_init(MainMode_e new_mode)
{
    g_mode_mgr.transitioning = 1;
    g_mode_mgr.switch_ts    = HAL_GetTick();

    /* 新模式进入初始化: 所有模式进入时都先待机, 不自动启动 */
    s_gait_running = 0;

    if (new_mode == MODE_INDUSTRIAL) {
        /* ★ v1.7: 工业模式进入 — ABO 工业参数初始化 */
        extern ABOState_t g_abo_state[6];
        for (int i = 0; i < 6; i++) {
            g_abo_state[i].industrial_mode   = 1;
            g_abo_state[i].bias_leak_q16     = 10;     /* 极小 leak ≈ 60s */
            g_abo_state[i].hpf_alpha_q16     = 0;      /* 不再用单 HPF */
            g_abo_state[i].assist_gain_q10   = 1024;   /* 默认 1.0× */
            g_abo_state[i].load_freeze_cnt   = 0;
            g_abo_state[i].load_est_q10      = 0;
            g_abo_state[i].bp_lpf_state      = 0;
            g_abo_state[i].tau_prev          = 0;
            g_abo_state[i].enable            = 1;
        }
        g_mode_mgr.abo_global_en = 1;
        printf("[MODE] INDUSTRIAL enter (industrial ABO)\r\n");
    } else {
        /* 非工业模式: 清除工业标志 */
        extern ABOState_t g_abo_state[6];
        for (int i = 0; i < 6; i++) {
            g_abo_state[i].industrial_mode = 0;
        }
    }

    (void)new_mode;
}

/* ===== 公共 API ===== */

void mode_manager_init(void)
{
    g_mode_mgr.main_mode     = MODE_ZERO_TORQUE;
    g_mode_mgr.prev_mode     = MODE_ZERO_TORQUE;
    g_mode_mgr.gait_sub      = GAIT_SUB_FLAT;
    g_mode_mgr.arm_level     = ARM_ASSIST_OFF;
    g_mode_mgr.abo_global_en = 1;
    g_mode_mgr.transitioning = 0;
    s_gait_running            = 0;
    memset(&s_wkup_sm, 0, sizeof(s_wkup_sm));
}

void mode_fsm_process(void)
{
    uint32_t now = HAL_GetTick();

    /* ★ v1.6.8fix: 仅扫描 WKUP, 放弃 KEY0 */
    uint8_t wkup_pressed = (WK_UP == 1);

    /* Debug: 按键电平变化时打印 */
    static uint8_t prev_wkup = 0xFF;
    if (prev_wkup == 0xFF) { prev_wkup = wkup_pressed; }
    if (wkup_pressed != prev_wkup) {
        printf("[KEY] WKUP=%d st=%d\r\n", WK_UP, s_wkup_sm.state);
        prev_wkup = wkup_pressed;
    }

    uint8_t wkup_evt = scan_key(&s_wkup_sm, wkup_pressed, now);

    /* 映射为事件 */
    uint8_t evt = EVT_NONE;
    if (wkup_evt & EVT_SHORT_FLAG) evt |= EVT_WKUP_SHORT;
    if (wkup_evt & EVT_LONG_FLAG)  evt |= EVT_WKUP_LONG;

    if (evt != EVT_NONE) {
        printf("[KEY] evt=0x%02X mode=%d->?\r\n", evt, g_mode_mgr.main_mode);
        mode_handle_evt(evt);
    }

    /* 清除切换中标志 (500ms 后) */
    if (g_mode_mgr.transitioning &&
        now - g_mode_mgr.switch_ts > 500) {
        g_mode_mgr.transitioning = 0;
    }
}

void mode_handle_evt(uint8_t evt)
{
    /* HOST 模式下只响应 WKUP 长按 (切回 STANDALONE) */
    if (g_comm_mode != COMM_MODE_STANDALONE) {
        if (evt & EVT_WKUP_LONG) {
            extern volatile CommMode_e g_comm_mode;
            g_comm_mode = COMM_MODE_STANDALONE;
            mode_request_switch(MODE_ZERO_TORQUE);
            SAFETY_LOCK();
            g_safety_state.fault_code &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
            SAFETY_UNLOCK();
            printf("[MODE] HOST -> STANDALONE\r\n");
        }
        return;
    }

    /* === STANDALONE 模式: 仅 WKUP 单键控制 === */

    if (evt & EVT_WKUP_SHORT) {
        /* ★ v1.7: 短按模式循环
         * INDUSTRIAL_MODE=1: ZERO → GAIT → INDUSTRIAL → ZERO
         * INDUSTRIAL_MODE=0: ZERO → GAIT → ZERO (原医疗逻辑) */
        if (g_mode_mgr.main_mode == MODE_ZERO_TORQUE) {
            mode_request_switch(MODE_GAIT);
            s_gait_start_ms = HAL_GetTick();
            s_gait_running  = 1;
            printf("[MODE] ZERO -> GAIT (auto start)\r\n");
        } else if (g_mode_mgr.main_mode == MODE_GAIT) {
            s_gait_running = 0;
#if INDUSTRIAL_MODE
            mode_request_switch(MODE_INDUSTRIAL);
            printf("[MODE] GAIT -> INDUSTRIAL\r\n");
#else
            mode_request_switch(MODE_ZERO_TORQUE);
            printf("[MODE] GAIT -> ZERO\r\n");
#endif
        } else if (g_mode_mgr.main_mode == MODE_INDUSTRIAL) {
            mode_request_switch(MODE_ZERO_TORQUE);
            printf("[MODE] INDUSTRIAL -> ZERO\r\n");
        } else {
            mode_request_switch(MODE_ZERO_TORQUE);
            printf("[MODE] -> ZERO\r\n");
        }
    }

    if (evt & EVT_WKUP_LONG) {
        /* 长按: 强制回零力矩 + 停步态 */
        s_gait_running = 0;
        mode_request_switch(MODE_ZERO_TORQUE);
        printf("[MODE] WKUP long: Force ZERO_TORQUE\r\n");
    }
}

void mode_request_switch(MainMode_e new_mode)
{
    if (new_mode == g_mode_mgr.main_mode) return;

    /* 1. 旧模式退出清理 */
    mode_exit_cleanup(g_mode_mgr.main_mode);

    /* 2. 记录并切换 */
    g_mode_mgr.prev_mode = g_mode_mgr.main_mode;
    g_mode_mgr.main_mode = new_mode;

    /* 3. 新模式进入初始化 */
    mode_enter_init(new_mode);
}

/* ===== 查询接口 ===== */

MainMode_e mode_get_main(void)       { return g_mode_mgr.main_mode; }
GaitSubMode_e mode_get_gait_sub(void) { return g_mode_mgr.gait_sub; }
ArmAssistLevel_e mode_get_arm_level(void) { return g_mode_mgr.arm_level; }
uint8_t mode_get_abo_enabled(void)   { return g_mode_mgr.abo_global_en; }

const char *mode_get_main_name(void)
{
    static const char *names[] = {"ZERO", "GAIT", "ARM", "IND"};
    if (g_mode_mgr.main_mode < MODE_COUNT) return names[g_mode_mgr.main_mode];
    return "?";
}

const char *mode_get_gait_sub_name(void)
{
    static const char *names[] = {"FLAT", "UP", "DOWN", "STAIR"};
    if (g_mode_mgr.gait_sub < GAIT_SUB_COUNT) return names[g_mode_mgr.gait_sub];
    return "?";
}

const char *mode_get_arm_level_name(void)
{
    static const char *names[] = {"OFF", "0.5x", "1.0x", "1.5x"};
    if (g_mode_mgr.arm_level < ARM_ASSIST_COUNT) return names[g_mode_mgr.arm_level];
    return "?";
}

/* ===== 兼容旧接口 ===== */

uint8_t mode_get_local_mode(void)
{
    /* 映射到旧 LocalMode_e: 0=ZERO, 1=GAIT, 2=ARM */
    return (uint8_t)g_mode_mgr.main_mode;
}

uint8_t mode_get_gait_running(void)
{
    return s_gait_running;
}

void mode_gait_start_stop(uint8_t start)
{
    if (start && g_mode_mgr.main_mode == MODE_GAIT) {
        if (!s_gait_running) {
            s_gait_start_ms = HAL_GetTick();
            s_gait_running  = 1;
        }
    } else {
        s_gait_running = 0;
    }
}

uint32_t mode_get_gait_start_ms(void)
{
    return s_gait_start_ms;
}
