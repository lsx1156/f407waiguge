#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

void Error_Handler(void);

/* ========== CubeMX-generated HAL handles ========== */
extern CAN_HandleTypeDef   hcan1;
extern CAN_HandleTypeDef   hcan2;
extern I2C_HandleTypeDef   hi2c1;
extern TIM_HandleTypeDef   htim6;
extern SRAM_HandleTypeDef  hsram1;
extern ETH_HandleTypeDef   heth;
extern UART_HandleTypeDef  huart1;
extern IWDG_HandleTypeDef  hiwdg;

/* ========== App/ code compatibility aliases ========== */
#define g_can1_handle   hcan1
#define g_can2_handle   hcan2
#define g_i2c1_handle   hi2c1
#define g_tim6_handle   htim6
#define g_eth_handle    heth
#define g_sram_handler  hsram1

/* v1.6.2: SRAM 就绪标志, sram_init() 后置 1, sys_info_update_sram() 前检查 */
extern volatile uint8_t g_sram_ready;

/* ========== GPIO Pin Macros (from bsp_config.h) ========== */
/* LED */
#define LED0_PIN    GPIO_PIN_9
#define LED0_PORT   GPIOF
#define LED1_PIN    GPIO_PIN_10
#define LED1_PORT   GPIOF

/* EN pins */
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

/* ESTOP */
#define ESTOP_PIN           GPIO_PIN_3
#define ESTOP_PORT          GPIOC

/* WDT Feed */
#define WDT_FEED_PIN        GPIO_PIN_0
#define WDT_FEED_PORT       GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */