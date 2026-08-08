#ifndef __SAFETY_H__
#define __SAFETY_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "bsp_config.h"
#include "contract.h"
#include "lwip/sys.h"   /* sys_arch_protect / sys_arch_unprotect */

#define SAFETY_VELOCITY_LIMIT    30000
#define SAFETY_TORQUE_LIMIT      10000
#define SAFETY_POSITION_JUMP     30000  /* v1.6.3+fix: 再放宽到 30000(30度).  5度/ms 阈值对人工转动太严格, 拧电机手拧几毫秒过 30度很常见. 30度/ms = 30000度/s 远超电机最大 1719度/s, 仍有安全意义 */

/* v1.6.8fix: 臂关节软限位 (mdeg) — 按关节类型分别设置
 * 肩关节: -30° ~ 150° (手臂后伸30°, 前屈150°)
 * 肘关节: 0° ~ 140° (伸直0°, 弯曲140°, 不可过伸) */
#define ARM_SHOULDER_POS_MAX   150000    /* 150° */
#define ARM_SHOULDER_POS_MIN   -30000    /* -30° */
#define ARM_ELBOW_POS_MAX      140000    /* 140° */
#define ARM_ELBOW_POS_MIN      0         /* 0° */

/* v1.6.8: 碰撞检测 — 力矩突变阈值 (mNm/ms), 超过则触发 FAULT_COLLISION */
#define SAFETY_COLLISION_TORQUE_DELTA  8000  /* 8 N·m/ms 突变 = 碰撞 */

/* v1.6.8: 新增故障码 */
#define FAULT_ARM_SOFT_LIMIT   0x0200  /* 臂关节软限位超限 */
#define FAULT_COLLISION        0x0400  /* 碰撞检测 (力矩突变) */

/* v1.8.1 P1-2: ESO3 发散故障 (WARN 级, 触发自动降级回 legacy, 不锁死) */
#define FAULT_ESO_DIVERGE      0x0800  /* ESO3 z3 发散, 已自动降级 */

/* v2.0: 算法故障接入安全状态机 (WARN 级 + 保持定时器, 不锁死机器人) */
#define FAULT_JACOBIAN_SINGULAR 0x1000  /* 雅可比奇异 (WARN, 已回退上一拍有效F_ext) */
#define FAULT_ZUPT_FAIL         0x2000  /* ZUPT 持续失效 (双足离地>2s) */

/* v2.0: 算法故障保持可见时间 (ms), 1kHz 递减; 期间上报故障, 到 0 自动清除 */
#define ALGO_FAULT_HOLD_MS      1000u

#define SAFETY_RECOVERY_OK_CYCLES   100

/* g_safety_state 在 ISR (1ms) 和主循环之间共享，
 * 对 fault_code 的读-改-写必须用临界区保护 */
#define SAFETY_LOCK()     sys_prot_t _safety_lev = sys_arch_protect()
#define SAFETY_UNLOCK()  sys_arch_unprotect(_safety_lev)

typedef enum {
    FAULT_LEVEL_NONE = 0,
    FAULT_LEVEL_WARN,
    FAULT_LEVEL_LATCH,
    FAULT_LEVEL_HW_ESTOP
} FaultLevel_e;

typedef struct {
    volatile uint32_t fault_code;       /* 改为 uint32_t: Cortex-M4 上 32 位对齐访问是原子的 */
    uint8_t  estop_active;
    uint8_t  system_enabled;
    uint8_t  recovery_counter;
    FaultLevel_e fault_level;
    /* v2.0: 算法故障保持定时器 (ms, 1kHz 递减); >0 期间故障位置位, 到 0 清除 */
    uint16_t eso_div_hold_ms;           /* ESO 发散保持 */
    uint16_t jac_sing_hold_ms;          /* 雅可比奇异保持 */
    uint16_t zupt_fail_hold_ms;         /* ZUPT 失效保持 */
} SafetyState_t;

extern volatile SafetyState_t g_safety_state;

void safety_init(void);
void safety_check_estop(void);
void safety_check_motor_limits(JointStatus_t *leg, JointStatus_t *arm);
void safety_handle_fault(void);
void safety_task_run(void);
uint8_t safety_is_system_safe(void);
uint32_t safety_get_faults(void);
void safety_request_enable(void);
void safety_pull_en_low(void);    /* v1.6.2: 公共接口, 供 mode_manager 急停调用 */
void safety_check_algorithm_faults(void);  /* v2.0: 算法故障接入, 1kHz 调用 */

#endif
