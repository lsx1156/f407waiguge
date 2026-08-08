/**
 * @file mode_manager.h
 * @brief 模式管理器 - 单按键 (WKUP) 状态机
 *
 * ★ v1.6.8fix: 放弃 KEY0, 仅用 WKUP 单键控制
 * 交互设计:
 *   WKUP (PA0) 短按<500ms: ZERO ↔ GAIT 切换 (进入 GAIT 自动启动步态)
 *   WKUP 长按>3s:          强制回 ZERO_TORQUE
 */

#ifndef __MODE_MANAGER_H
#define __MODE_MANAGER_H

#include "main.h"

/* ===== 主模式枚举 ===== */
typedef enum {
    MODE_ZERO_TORQUE = 0,   /* 零力矩透明 (默认安全态) */
    MODE_GAIT        = 1,   /* 步态混合控制 (GAIT + ABO) */
    MODE_ARM_ASSIST  = 2,   /* ★ 废弃 (v1.6.8fix: 不再通过按键进入, 保留枚举兼容) */
    MODE_INDUSTRIAL  = 3,   /* ★ v1.7: 工业助力 (负载自适应, 无相位, 极慢泄漏) */
    MODE_COUNT       = 4,
} MainMode_e;

/* ===== 步态子模式 ===== */
typedef enum {
    GAIT_SUB_FLAT  = 0,     /* 平地 */
    GAIT_SUB_UP    = 1,     /* 上坡 */
    GAIT_SUB_DOWN  = 2,     /* 下坡 */
    GAIT_SUB_STAIR = 3,     /* 楼梯 */
    GAIT_SUB_COUNT = 4,
} GaitSubMode_e;

/* ===== 鳌臂助力档位 (★ 废弃, 保留枚举兼容) ===== */
typedef enum {
    ARM_ASSIST_OFF   = 0,
    ARM_ASSIST_LIGHT = 1,
    ARM_ASSIST_NORM  = 2,
    ARM_ASSIST_STRONG= 3,
    ARM_ASSIST_COUNT = 4,
} ArmAssistLevel_e;

/* ===== 模式管理器状态 ===== */
typedef struct {
    MainMode_e       main_mode;       /* 当前主模式 */
    MainMode_e       prev_mode;       /* 上一主模式 */
    GaitSubMode_e    gait_sub;        /* 步态子模式 */
    ArmAssistLevel_e arm_level;       /* 鳌臂助力档位 (废弃) */
    uint8_t          abo_global_en;   /* ABO 全局总开关 */
    uint8_t          transitioning;   /* 正在切换中标志 */
    uint32_t         switch_ts;       /* 切换时间戳 */
} ModeManager_t;

extern ModeManager_t g_mode_mgr;

/* ===== 按键事件定义 ===== */
#define EVT_NONE         0x00
#define EVT_WKUP_SHORT   0x01
#define EVT_WKUP_LONG    0x02
#define EVT_KEY0_SHORT   0x04   /* ★ 废弃, 不再产生 */
#define EVT_KEY0_LONG    0x08   /* ★ 废弃, 不再产生 */

/* ===== API ===== */
void mode_manager_init(void);
void mode_fsm_process(void);           /* 主循环 10ms 调用, 内部扫描按键 */
void mode_handle_evt(uint8_t evt);      /* 处理按键事件 */
void mode_request_switch(MainMode_e new_mode);  /* 统一模式切换入口 */

/* 查询接口 (供 control_isr.c / lcd_status.c 使用) */
MainMode_e       mode_get_main(void);
GaitSubMode_e    mode_get_gait_sub(void);
ArmAssistLevel_e mode_get_arm_level(void);
uint8_t          mode_get_abo_enabled(void);
const char      *mode_get_main_name(void);
const char      *mode_get_gait_sub_name(void);
const char      *mode_get_arm_level_name(void);

/* 兼容旧接口 (control_isr.c 中 local_get_mode 等) */
uint8_t  mode_get_local_mode(void);      /* 0=ZERO, 1=GAIT, 2=ARM */
uint8_t  mode_get_gait_running(void);
void     mode_gait_start_stop(uint8_t start);
uint32_t mode_get_gait_start_ms(void);   /* 步态起始时间戳 */

#endif /* __MODE_MANAGER_H */
