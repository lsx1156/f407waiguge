#ifndef __EEPROM_PARAMS_H__
#define __EEPROM_PARAMS_H__

#include "stm32f4xx.h"
#include "contract.h"

/* ========== AT24C02 EEPROM (256 bytes) Address Map ==========
 *
 *  Offset  Size  Content
 *  ------  ----  ------------------------------------------
 *  0x00    4     Header (Magic + Version + CRC16)
 *  0x04    16    ABO shared defaults (gain/alpha/leak/enable_mask)
 *  0x14    24    ABO per-joint bias_est (6 joints × int32)
 *  0x2C    16    PID default params (1 set)
 *  0x3C    84    Temp coeff joint 0 (21 × float, P2-4)
 *  0x90    84    Temp coeff joint 1 (21 × float, P2-4)
 *  0xE4    28    Reserved (gait params, calibration)
 */

#define EEP_MAGIC_ADDR          0x00
#define EEP_VERSION_ADDR        0x01
#define EEP_CRC_ADDR            0x02

#define EEP_ABO_SHARED_ADDR     0x04
#define EEP_ABO_SHARED_SIZE     16

#define EEP_ABO_BIAS_ADDR       0x14
#define EEP_ABO_BIAS_PER_JOINT  4     /* int32_t per joint */

#define EEP_PID_DEFAULT_ADDR    0x2C
#define EEP_PID_DEFAULT_SIZE    16

/* P2-4: 温度零漂系数表 (每关节 21 个 float).
 * 256B EEPROM 的 Reserved 区 (0x3C~0xFF, 196B) 仅容 2 关节 (84B/关节).
 * 注: 21 = TEMP_TABLE_SIZE, 见 joint_unit.h (此处不引入 joint_unit.h 以免硬件依赖). */
#define EEP_TEMP_COEFF_ADDR_BASE   0x3C
#define EEP_TEMP_COEFF_PER_JOINT   (21u * 4u)   /* 84 字节: 21(TEMP_TABLE_SIZE) × sizeof(float) */
#define EEP_TEMP_COEFF_MAX_JOINTS  2            /* 256B EEPROM 仅容 2 关节 */

#define EEP_MAGIC_VALUE         0x5A   /* 首次写入后标记 EEPROM 已初始化 */
#define EEP_VERSION_CURRENT     0x01

/* ========== Shared ABO defaults (16 bytes) ========== */
typedef struct PACKED {
    uint16_t assist_gain_q10;      /* 默认助力增益 G * 1024 */
    uint16_t hpf_alpha_q16;        /* 默认 HPF α * 65536 */
    uint16_t bias_leak_q16;        /* 默认偏置泄漏 * 65536 */
    uint8_t  enable_mask;           /* bit0~5 = joint0~5 ABO 使能 */
    uint8_t  reserved[9];           /* 对齐到 16 字节 */
} EEP_ABOShared_t;

/* ========== Public API ========== */

/* 上电加载: 从 EEPROM 读取所有参数到全局变量
 * 返回: 0=OK (含首次使用用默认值初始化), 非0=读错误 */
uint8_t eeprom_params_load_all(void);

/* 保存: 把全局变量写入 EEPROM (带 CRC 校验)
 * 触发条件: 参数修改后 + 心跳稳定时 (非紧急) */
uint8_t eeprom_params_save_all(void);

/* 单独保存 ABO bias_est (偏置估计收敛后调用, 约每 30s 或模式切换时)
 * 比 save_all 快, 只写 24+6=30 字节 */
uint8_t eeprom_params_save_abo_bias(void);

/* P2-4: 温度零漂系数表读写 (每关节 21 个 float).
 *  write: 异步入队 (走 eeprom_write_buffer), 调用者需周期调 eeprom_process_write_buffer() 落盘,
 *         且两次 write 之间必须等上一次落盘完成 (单缓冲限制, 与 eeprom_params_save_all 一致).
 *  返回: 0=成功, 1=idx 超出 EEPROM 容量 (EEP_TEMP_COEFF_MAX_JOINTS) */
uint8_t eeprom_write_temp_coeff(uint8_t idx, const float coeff[21]);
uint8_t eeprom_read_temp_coeff(uint8_t idx, float coeff[21]);

#endif
