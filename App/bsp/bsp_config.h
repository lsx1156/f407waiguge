#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

#include "stm32f4xx.h"

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

/* ADC */
#define ADC_WEIGHT_PIN      GPIO_PIN_0
#define ADC_WEIGHT_PORT     GPIOA
#define ADC_WEIGHT_CHANNEL  ADC_CHANNEL_0
#define ADC_VOLTAGE_PIN     GPIO_PIN_5
#define ADC_VOLTAGE_PORT    GPIOA
#define ADC_VOLTAGE_CHANNEL ADC_CHANNEL_5

/* Enable Pins (Push-Pull, Active High, Default Low)
 * EN_ARM_1/2 moved from PE0/PE1 to PE4/PE5 to avoid FSMC_NBL0/NBL1 conflict
 * EN_LEG_L/R moved from PD12/PD13 to PC6/PC7 to avoid FSMC_A17/A18 conflict */
#define EN_LEG_L_PIN        GPIO_PIN_6
#define EN_LEG_L_PORT       GPIOC
#define EN_LEG_R_PIN        GPIO_PIN_7
#define EN_LEG_R_PORT       GPIOC

#define EN_ARM_1_PIN        GPIO_PIN_4
#define EN_ARM_1_PORT       GPIOE
#define EN_ARM_2_PIN        GPIO_PIN_5
#define EN_ARM_2_PORT       GPIOE
#define EN_ARM_3_PIN        GPIO_PIN_2
#define EN_ARM_3_PORT       GPIOE
#define EN_ARM_4_PIN        GPIO_PIN_3
#define EN_ARM_4_PORT       GPIOE

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
 */
#define LED0_PIN            GPIO_PIN_9
#define LED0_PORT           GPIOF
#define LED1_PIN            GPIO_PIN_10
#define LED1_PORT           GPIOF

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

/* ========== Communication Configuration ========== */

#define UDP_PORT            5001
#define CAN_BAUDRATE        1000000

/* ========== Control Period ========== */

#define CONTROL_PERIOD_LEG  1   /* ms */
#define CONTROL_PERIOD_ARM  2   /* ms */

/* ========== Interpolation Table Size ========== */

#define DAMPING_TABLE_SIZE  128
#define FRICTION_TABLE_SIZE 128
#define PID_PARAM_SET_COUNT 16

/* ========== Memory Partition Configuration ========== */

#define MEM_SRAM_START       0x20000000
#define MEM_SRAM_SIZE        (128 * 1024)
#define MEM_CCM_START        0x10000000
#define MEM_CCM_SIZE         (64 * 1024)
#define MEM_EXT_SRAM_START   SRAM_BASE_ADDR
#define MEM_EXT_SRAM_SIZE    SRAM_SIZE

#define MEM_LWIP_HEAP_SIZE     (64 * 1024)
#define MEM_PBUF_POOL_SIZE     (32 * 1024)
#define MEM_FIFO_EXT_SIZE      (64 * 1024)
#define MEM_LOG_SIZE           (64 * 1024)
#define MEM_TABLES_SIZE        (8 * 1024)
#define MEM_RESERVED_SIZE      (832 * 1024)

#define MEM_STATIC_BUFFER_SIZE (16 * 1024)

/* ========== FIFO Configuration ========== */

#define FIFO_REPORT_SIZE     8
#define FIFO_COMMAND_SIZE    4
#define FIFO_FAULT_SIZE      4

#endif
