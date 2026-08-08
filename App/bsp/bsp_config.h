#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

#include "stm32f4xx.h"

/* ========== Compile-Time Mode Switch (v1.7) ==========
 * INDUSTRIAL_MODE = 1 : 工业外骨骼 (负载搬运, ABO 极慢泄漏 + 带通 + 增益调度)
 * INDUSTRIAL_MODE = 0 : 医疗/康复 (步态相位, ABO 分相切换)
 * WKUP 短按循环: ZERO → GAIT → [INDUSTRIAL if 1] → ZERO */
#define INDUSTRIAL_MODE     1

/* ========== Pin Definitions ========== */

/* CAN1 */
#define CAN1_RX_PIN         GPIO_PIN_11
#define CAN1_RX_PORT        GPIOA
#define CAN1_TX_PIN         GPIO_PIN_12
#define CAN1_TX_PORT        GPIOA

/* CAN2 (Remap) */
#define CAN2_RX_PIN         GPIO_PIN_12
#define CAN2_RX_PORT        GPIOB
#define CAN2_TX_PIN         GPIO_PIN_13
#define CAN2_TX_PORT        GPIOB

/* Enable Pins (Push-Pull, Active High, Default Low)
 * v1.6.7: EN_ARM_1 moved from PE4 to PE6 to avoid conflict with KEY0 (PE4)
 * EN_ARM_2/3/4 remain on PE5/PE2/PE3
 * EN_LEG_L/R moved from PD12/PD13 to PC6/PC7 to avoid FSMC_A17/A18 conflict
 * 硬件逻辑: 拉低 = 切断电机驱动板动力级使能（硬件急停回路） */
#define EN_LEG_L_PIN        GPIO_PIN_6
#define EN_LEG_L_PORT       GPIOC
#define EN_LEG_R_PIN        GPIO_PIN_7
#define EN_LEG_R_PORT       GPIOC

#define EN_ARM_1_PIN        GPIO_PIN_6
#define EN_ARM_1_PORT       GPIOE
#define EN_ARM_2_PIN        GPIO_PIN_5
#define EN_ARM_2_PORT       GPIOE
#define EN_ARM_3_PIN        GPIO_PIN_2
#define EN_ARM_3_PORT       GPIOE
#define EN_ARM_4_PIN        GPIO_PIN_3
#define EN_ARM_4_PORT       GPIOE

/* Ethernet PHY Address (DP83848/KSZ8081, adjust per hardware strap) */
#define ETH_PHY_ADDR         0

/* Emergency Stop (Opto-isolated, Active Low) */
#define ESTOP_PIN           GPIO_PIN_3
#define ESTOP_PORT          GPIOC

/* Watchdog Feed (SP706 WDI, Toggle) */
#define WDT_FEED_PIN        GPIO_PIN_0
#define WDT_FEED_PORT       GPIOB

/* I2C EEPROM */
#define I2C_SCL_PIN         GPIO_PIN_6
#define I2C_SCL_PORT        GPIOB
#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_SDA_PORT        GPIOB

/* Status LEDs (Low Active, 启动自检指示)
 *   LED0 - PF9   (运行指示)
 *   LED1 - PF10  (故障/状态指示)
 *   Pin macros now in main.h
 */
#define LED0_CLK_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED1_CLK_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()

/* SPI Flash (W25Q64, SPI1, 8MB/64Mbit)
 *   SPI1_SCK  - PB3   (AF5)
 *   SPI1_MISO - PB4   (AF5)
 *   SPI1_MOSI - PB5   (AF5)
 *   F_CS      - PB14  (软件控制, 推挽输出)
 */
#define FLASH_CS_PIN        GPIO_PIN_14
#define FLASH_CS_PORT       GPIOB
#define FLASH_SPI           SPI1
#define FLASH_SPI_SCK_PIN   GPIO_PIN_3
#define FLASH_SPI_SCK_PORT  GPIOB
#define FLASH_SPI_MISO_PIN  GPIO_PIN_4
#define FLASH_SPI_MISO_PORT GPIOB
#define FLASH_SPI_MOSI_PIN  GPIO_PIN_5
#define FLASH_SPI_MOSI_PORT GPIOB
#define FLASH_SPI_AF        GPIO_AF5_SPI1

#define FLASH_CS_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define FLASH_SPI_CLK_ENABLE()  __HAL_RCC_SPI1_CLK_ENABLE()

/* W25Q64 容量: 8MB (64Mbit) */
#define FLASH_SIZE           (8 * 1024 * 1024)
#define FLASH_SECTOR_SIZE    4096
#define FLASH_PAGE_SIZE      256

/* ========== External SRAM (IS62WV51216BLL) Configuration ==========
 *
 * FSMC 引脚映射 (IS62WV51216BLL, 1MB, 16-bit):
 *
 *   地址线:
 *     FSMC_A0  - PF0     FSMC_A1  - PF1     FSMC_A2  - PF2
 *     FSMC_A3  - PF3     FSMC_A4  - PF4     FSMC_A5  - PF5
 *     FSMC_A6  - PF12    FSMC_A7  - PF13    FSMC_A8  - PF14
 *     FSMC_A9  - PF15    FSMC_A10 - PG0     FSMC_A11 - PG1
 *     FSMC_A12 - PG2     FSMC_A16 - PD11    FSMC_A17 - PD12
 *     FSMC_A18 - PD13
 *
 *   数据线:
 *     FSMC_D0  - PD14    FSMC_D1  - PD15    FSMC_D2  - PD0
 *     FSMC_D3  - PD1     FSMC_D4  - PE7     FSMC_D5  - PE8
 *     FSMC_D6  - PE9     FSMC_D7  - PE10    FSMC_D8  - PE11
 *     FSMC_D9  - PE12    FSMC_D10 - PE13    FSMC_D11 - PE14
 *     FSMC_D12 - PE15    FSMC_D13 - PD8     FSMC_D14 - PD9
 *     FSMC_D15 - PD10
 *
 *   控制信号:
 *     FSMC_NE3  - PG10   (片选, Bank3 -> 0x68000000)
 *     FSMC_NOE  - PD4    (读使能)
 *     FSMC_NWE  - PD5    (写使能)
 *     FSMC_NBL0 - PE0    (低字节掩码)
 *     FSMC_NBL1 - PE1    (高字节掩码)
 *
 * ===============================================================
 */

#define SRAM_FSMC_NEX       3
#define SRAM_BASE_ADDR      0x68000000
#define SRAM_SIZE           (1024 * 1024)

#define SRAM_CS_GPIO_PIN    GPIO_PIN_10
#define SRAM_CS_GPIO_PORT   GPIOG
#define SRAM_WR_GPIO_PIN    GPIO_PIN_5
#define SRAM_WR_GPIO_PORT   GPIOD
#define SRAM_RD_GPIO_PIN    GPIO_PIN_4
#define SRAM_RD_GPIO_PORT   GPIOD

#define SRAM_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define SRAM_WR_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()
#define SRAM_RD_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

/* ========== Motor Configuration ========== */

#define MOTOR_COUNT_LEG     2
#define MOTOR_COUNT_ARM     4
#define MOTOR_COUNT_TOTAL   (MOTOR_COUNT_LEG + MOTOR_COUNT_ARM)

#define MOTOR_ID_LEG_LHIP   0x01
#define MOTOR_ID_LEG_LKNEE  0x02
#define MOTOR_ID_ARM_1      0x10
#define MOTOR_ID_ARM_2      0x11
#define MOTOR_ID_ARM_3      0x12
#define MOTOR_ID_ARM_4      0x13

#ifndef MOTOR_LEG_0_ID
#define MOTOR_LEG_0_ID      MOTOR_ID_LEG_LHIP
#endif
#ifndef MOTOR_LEG_1_ID
#define MOTOR_LEG_1_ID      MOTOR_ID_LEG_LKNEE
#endif
#ifndef MOTOR_ARM_0_ID
#define MOTOR_ARM_0_ID      MOTOR_ID_ARM_1
#endif

/* ========== Communication Configuration ========== */

#define UDP_PORT            5001
#define CAN_BAUDRATE        1000000

/* ========== Control Period ========== */

#define CONTROL_PERIOD_LEG  1   /* ms */
#define CONTROL_PERIOD_ARM  1   /* ms, 与接收周期一致, 避免观测器/控制器采样频率混叠 */

/* v1.8.1 P1-1b: §C/§D/§E 全局任务降频目标 (Hz). control_task_run 默认 100Hz,
 * 用静态分频计数器把 §C/§D/§E 三调用降到本频率 (节省 CPU). safety 保持 100Hz.
 * 取值须能整除 100 (如 50/25/20). 100/本值 = 分频系数. */
#ifndef GLOBAL_TASK_RATE_HZ
#define GLOBAL_TASK_RATE_HZ 50
#endif

/* 髋关节步态目标钳位范围 (mdeg) — 亦作 JointUnit 腿部软限位 */
#define LOCAL_HIP_POS_MAX_MDEG    90000
#define LOCAL_HIP_POS_MIN_MDEG   -30000

/* ========== Interpolation Table Size ========== */

#define DAMPING_TABLE_SIZE  128
#define FRICTION_TABLE_SIZE 128
#define PID_PARAM_SET_COUNT 16

/* LZ4 表格压缩开关: 0=关闭(当前表小, 直接原始数组), 1=启用(表>4KB时打开, 需配合 tools/gen_lz4_tables.py 生成的头文件 */
#ifndef LZ4_TABLES_USE_COMPRESSION
#define LZ4_TABLES_USE_COMPRESSION  0
#endif

/* ========== Memory Partition Configuration ==========
 *
 *  External SRAM (IS62WV51216BLL, 1MB @ 0x68000000) Layout:
 *
 *  0x6800_0000 ┌─────────────────────────────┐
 *              │  TABLE Area (128 KB)         │  阻尼/摩擦/步态表, 未来可扩展
 *              │  - 0x6800_0000 Damping      │
 *              │  - 0x6801_0000 Friction     │
 *              │  - 0x6802_0000 (reserved)   │
 *  0x6802_0000 ├─────────────────────────────┤
 *              │  LwIP Heap (64 KB)           │  MEM_LWIP_HEAP_SIZE
 *  0x6803_0000 ├─────────────────────────────┤
 *              │  PBUF Pool (32 KB)            │  MEM_PBUF_POOL_SIZE
 *  0x6803_8000 ├─────────────────────────────┤
 *              │  FIFO Ext (64 KB)             │  CAN/UDP 乒乓缓冲
 *  0x6804_8000 ├─────────────────────────────┤
 *              │  Log Buffer (64 KB)           │  运行日志环形缓冲
 *  0x6805_8000 ├─────────────────────────────┤
 *              │  Static Buffer (16 KB)        │  DMA/临时工作区
 *  0x6805_C000 ├─────────────────────────────┤
 *              │  (Free, ~656 KB)              │  未来扩展: 大表/OTA/帧存
 *  0x680F_FFFF └─────────────────────────────┘
 */

#define MEM_SRAM_START       0x20000000
#define MEM_SRAM_SIZE        (192 * 1024)   /* F407ZG 内部 RAM 192KB */
#define MEM_CCM_START        0x10000000
#define MEM_CCM_SIZE         (64 * 1024)
#define MEM_EXT_SRAM_START   SRAM_BASE_ADDR
#define MEM_EXT_SRAM_SIZE    SRAM_SIZE

/* --- External SRAM partitions (address + size) --- */
#define MEM_TABLE_ADDR        (MEM_EXT_SRAM_START + 0x00000000)   /* 0x6800_0000 */
#define MEM_TABLE_SIZE        (128 * 1024)

#define MEM_LWIP_HEAP_ADDR    (MEM_TABLE_ADDR + MEM_TABLE_SIZE)       /* 0x6802_0000 */
#define MEM_LWIP_HEAP_SIZE    (64 * 1024)

#define MEM_PBUF_POOL_ADDR    (MEM_LWIP_HEAP_ADDR + MEM_LWIP_HEAP_SIZE)  /* 0x6803_0000 */
#define MEM_PBUF_POOL_SIZE    (32 * 1024)

#define MEM_FIFO_EXT_ADDR     (MEM_PBUF_POOL_ADDR + MEM_PBUF_POOL_SIZE)   /* 0x6803_8000 */
#define MEM_FIFO_EXT_SIZE     (64 * 1024)

#define MEM_LOG_ADDR          (MEM_FIFO_EXT_ADDR + MEM_FIFO_EXT_SIZE)     /* 0x6804_8000 */
#define MEM_LOG_SIZE          (64 * 1024)

#define MEM_STATIC_BUF_ADDR   (MEM_LOG_ADDR + MEM_LOG_SIZE)               /* 0x6805_8000 */
#define MEM_STATIC_BUF_SIZE   (16 * 1024)

/* Kept for backward compatibility */
#define MEM_TABLES_SIZE        MEM_TABLE_SIZE

/* ========== FIFO Configuration ========== */

#define FIFO_REPORT_SIZE     32
#define FIFO_COMMAND_SIZE    4
#define FIFO_FAULT_SIZE      4

#endif
