/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - v0.1 基础版(外置SRAM+LCD硬件资源展示)
  ******************************************************************************
  */
/* USER CODE END Header */

/* ===== 功能开关：定义V01_BASIC_ONLY则只保留v0.1基础功能(SRAM+LCD) ===== */
/* #define V01_BASIC_ONLY */    /* ← 注释掉此行启用完整功能: UDP上报 + 1ms控制ISR + ABO + EEPROM */
/* #define CAN_DISABLED */      /* v0.6: 启用CAN */
/* ========================================================================== */

/* Includes ------------------------------------------------------------------*/
#ifndef V01_BASIC_ONLY
#include "main.h"
#include "bsp_config.h"
#include "contract.h"
#include "sram.h"
#ifndef CAN_DISABLED
#include "can_motor.h"
#include "control_isr.h"
#endif
#include "ethernet.h"
#include "tasks.h"
#include "lwip_comm.h"
#include "lwip/memp.h"
#include "udp_net.h"
#include "mode_manager.h"
#include "udp_protocol.h"
#include "eeprom.h"
#include "eeprom_params.h"
#include "interpolation.h"
#include "safety.h"
#endif

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/SRAM/sram.h"
#include "./BSP/SYS_INFO/sys_info.h"
#include "./BSP/LCD_STATUS/lcd_status.h"
#include "can_motor.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

void MPU_Config(void);

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim6;
SRAM_HandleTypeDef hsram1;
volatile uint8_t g_sram_ready = 0;  /* sram_init() 后置 1 */
ETH_HandleTypeDef heth;
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef hiwdg;

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef SystemClock_Config(void);
#ifndef V01_BASIC_ONLY
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
#endif
static void early_uart_init(void);

/* Private user code ---------------------------------------------------------*/
int __io_putchar(int ch)
{
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = (uint8_t)ch;
    return ch;
}

int __io_getchar(void)
{
    while (!(USART1->SR & USART_SR_RXNE)) { }
    return (int)USART1->DR;
}

#define BOOT_TRACE(fmt, ...)  printf("[BOOT %5lu ms] " fmt "\r\n", (unsigned long)HAL_GetTick(), ##__VA_ARGS__)

/**
  * @brief Emergency UART init - works from HSI 16MHz before SystemClock config
  *        PA9=TX, PA10=RX, 115200, 8N1
  */
static void early_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* 先驱动 PA9 (TX) 为高电平，防止切换到 AF 时 TX 线拉低产生乱码 */
    GPIOA->MODER |= GPIO_MODER_MODE9_0;
    GPIOA->ODR   |= GPIO_ODR_OD9;
    for (volatile uint32_t d = 0; d < 100; d++) { __NOP(); }

    GPIOA->MODER   &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER   |=  (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10);
    GPIOA->PUPDR   |=  (GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD10_0);
    GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2);
    GPIOA->AFR[1]  |=  (7U << 4) | (7U << 8);

    USART1->BRR  = 0x08AE;
    USART1->CR1  = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART1->CR2  = 0;
    USART1->CR3  = 0;
}

/**
  * @brief System Clock Configuration
  * @note  HSE=25MHz, PLLM=25, PLLN=336, PLLP=2, PLLQ=7 → 168MHz
  * @retval HAL_OK if HSE+PLL works, HAL_ERROR if fell back to HSI
  */
HAL_StatusTypeDef SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_StatusTypeDef status = HAL_OK;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        status = HAL_ERROR;
        memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = 16;
        RCC_OscInitStruct.PLL.PLLN = 336;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 7;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            SystemCoreClock = 16000000;
            return HAL_ERROR;
        }
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return status;
}

#ifndef V01_BASIC_ONLY
/**
  * @brief GPIO Initialization (LEDs + EN pins + ESTOP + WDT_FEED)
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LED0_PIN | LED1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED0_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(WDT_FEED_PORT, WDT_FEED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = WDT_FEED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WDT_FEED_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ESTOP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ESTOP_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(EN_LEG_L_PORT, EN_LEG_L_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_LEG_R_PORT, EN_LEG_R_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EN_LEG_L_PIN | EN_LEG_R_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_LEG_L_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(EN_ARM_1_PORT, EN_ARM_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_2_PORT, EN_ARM_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_3_PORT, EN_ARM_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_ARM_4_PORT, EN_ARM_4_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EN_ARM_1_PIN | EN_ARM_2_PIN | EN_ARM_3_PIN | EN_ARM_4_PIN;
    HAL_GPIO_Init(EN_ARM_1_PORT, &GPIO_InitStruct);
}

/**
  * @brief USART1 Initialization (115200 baud)
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief IWDG Initialization (LSI 32kHz, Prescaler=256, Reload=500 → ~4s)
  */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload = 500;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
        for (volatile uint32_t i = 0; i < 2000000; i++) { __NOP(); }
    }
}
#endif /* V01_BASIC_ONLY */

/* ===== v0.1 基础功能: 外置SRAM测试 =====
 * v1.6: 已禁用 g_test_buffer 放在 0x68000000！
 *   原因: 250000 x 4 = 1MB 完全覆盖了外部 SRAM 的所有分区
 *         (MEM_TABLE/LWIP_HEAP/PBUF_POOL/FIFO_EXT/...)
 *         导致 LWIP heap 中分配的 udp_pcb->recv 函数指针被破坏
 *         触发 INVSTATE HardFault (LSB=0 的函数指针被调用)
 *   如需 SRAM 测试, 请使用 sram_full_test() (仅出厂测试调用)
 */
/* 测试缓冲区不再放在 0x68000000, 避免覆盖 LWIP heap */
uint32_t g_test_buffer[1024];  /* 缩小到 4KB, 放在内部 RAM */

/**
 * @brief       SRAM 1MB 全读写测试 (16-bit 字级别)
 * @param       x: LCD显示x坐标
 * @param       y: LCD显示y坐标
 * @retval      0=通过, 非0=首个错误地址
 */
uint32_t sram_full_test(uint16_t x, uint16_t y)
{
    uint32_t addr, errors = 0;
    uint16_t wdata, rdata;
    uint32_t first_err = 0;
    uint32_t start_tick;

    printf("\r\n[SRAM] ========== 1MB Full Read/Write Test ==========\r\n");

    /* ---- Phase 1: 写入递增模式 (0x0000~0xFFFF 循环) ---- */
    printf("[SRAM] Phase 1: Writing 1MB (0x0000~0xFFFF increment)...\r\n");
    lcd_show_string(x, y, 239, y + 16, 16, "SRAM Full: Writing...", BLUE);
    start_tick = HAL_GetTick();
    for (addr = 0; addr < 1024 * 1024; addr += 2)
    {
        wdata = (uint16_t)(addr & 0xFFFF);
        sram_write(&wdata, addr, 2);
    }
    printf("[SRAM] Phase 1 done: %lu ms\r\n", (unsigned long)(HAL_GetTick() - start_tick));

    /* ---- Phase 2: 回读校验 ---- */
    printf("[SRAM] Phase 2: Verifying...\r\n");
    lcd_show_string(x, y, 239, y + 16, 16, "SRAM Full: Verify...", BLUE);
    start_tick = HAL_GetTick();
    for (addr = 0; addr < 1024 * 1024; addr += 2)
    {
        sram_read(&rdata, addr, 2);
        wdata = (uint16_t)(addr & 0xFFFF);
        if (rdata != wdata)
        {
            if (errors == 0) first_err = addr;
            errors++;
            if (errors <= 10)  /* 只打印前10个错误 */
            {
                printf("[SRAM] ERR @0x%06lX: W=0x%04X R=0x%04X\r\n",
                       (unsigned long)addr, wdata, rdata);
            }
        }
    }
    printf("[SRAM] Phase 2 done: %lu ms, errors=%lu\r\n",
           (unsigned long)(HAL_GetTick() - start_tick), (unsigned long)errors);

    /* ---- Phase 3: 写入互补模式 (0xFFFF~0x0000 递减) ---- */
    printf("[SRAM] Phase 3: Writing 1MB (0xFFFF~0x0000 decrement)...\r\n");
    lcd_show_string(x, y, 239, y + 16, 16, "SRAM Full: Wr(Inv)...", BLUE);
    start_tick = HAL_GetTick();
    for (addr = 0; addr < 1024 * 1024; addr += 2)
    {
        wdata = (uint16_t)(0xFFFF - (addr & 0xFFFF));
        sram_write(&wdata, addr, 2);
    }
    printf("[SRAM] Phase 3 done: %lu ms\r\n", (unsigned long)(HAL_GetTick() - start_tick));

    /* ---- Phase 4: 回读校验互补模式 ---- */
    printf("[SRAM] Phase 4: Verifying inverse...\r\n");
    lcd_show_string(x, y, 239, y + 16, 16, "SRAM Full: Vfy(Inv).", BLUE);
    start_tick = HAL_GetTick();
    for (addr = 0; addr < 1024 * 1024; addr += 2)
    {
        sram_read(&rdata, addr, 2);
        wdata = (uint16_t)(0xFFFF - (addr & 0xFFFF));
        if (rdata != wdata)
        {
            if (errors == 0) first_err = addr;
            errors++;
            if (errors <= 10)
            {
                printf("[SRAM] ERR @0x%06lX: W=0x%04X R=0x%04X\r\n",
                       (unsigned long)addr, wdata, rdata);
            }
        }
    }
    printf("[SRAM] Phase 4 done: %lu ms, total errors=%lu\r\n",
           (unsigned long)(HAL_GetTick() - start_tick), (unsigned long)errors);

    /* ---- 结果 ---- */
    if (errors == 0)
    {
        printf("[SRAM] RESULT: PASS - 1MB OK\r\n");
        lcd_show_string(x, y, 239, y + 16, 16, "SRAM Full: PASS 1MB", GREEN);
    }
    else
    {
        printf("[SRAM] RESULT: FAIL - %lu errors, first @0x%06lX\r\n",
               (unsigned long)errors, (unsigned long)first_err);
        lcd_show_string(x, y, 239, y + 16, 16, "SRAM Full: FAIL!    ", RED);
    }

    return first_err;
}

/* ===== main函数: 根据V01_BASIC_ONLY宏切换版本 ===== */
#ifdef V01_BASIC_ONLY
/* v0.7: 基础版 + 双电机协同 + 步态轨迹 + 安全机制 */

/* ========== 电机控制状态 ========== */
typedef enum {
    MODE_ZERO_TORQUE = 0,
    MODE_GAIT        = 1     /* Gait mode: left+right hip alternating */
} CtrlMode_t;

/* ===== 安全限位 (硬限制，超过则强制裁剪) ===== */
/* Left/Right Hip: symmetric limits, flexion positive */
#define HIP_POS_MAX_MDEG    90000    /* Hip max +90 deg (flexion) */
#define HIP_POS_MIN_MDEG   -30000    /* Hip min -30 deg (extension) */
#define TORQUE_LIMIT_MNM    2000     /* ±2 Nm */
#define COMM_TIMEOUT_MS     2000      /* 2s no feedback → fault */

/* ===== 步态控制参数 ===== */
#define POS_KP              10.0f
#define POS_KD              0.5f
#define IDLE_TIMEOUT_MS     15000

/* ===== 步态轨迹参数 ===== */
#define GAIT_PERIOD_MS      4000      /* 4s per gait cycle */
#define GAIT_MAX_VEL_MDEG_S 60000     /* 60 deg/s max velocity */
#define GAIT_NUM_POINTS     5          /* 5 key points per cycle */

/* 步态关键点: [phase%] = {left_hip, right_hip} in mdeg
 * Left and Right hips alternate: when one flexes the other extends
 * 0%:   Left heel strike (L=0, R=max flex)
 * 25%:  Left mid stance (L=mid, R=mid)
 * 50%:  Left toe off / Right heel strike (L=max flex, R=0)
 * 75%:  Left swing / Right mid stance (L=mid, R=mid)
 * 100%: Back to left heel strike
 */
static const int32_t g_gait_table[GAIT_NUM_POINTS][2] = {
    /* Left Hip, Right Hip */
    {       0,   60000 },   /*   0%: L heel strike, R push-off */
    {   30000,   30000 },   /*  25%: Double support, both mid */
    {   60000,       0 },   /*  50%: L toe off, R heel strike */
    {   30000,   30000 },   /*  75%: Double support, both mid */
    {       0,   60000 }    /* 100%: Back to start */
};

static CtrlMode_t g_ctrl_mode = MODE_ZERO_TORQUE;
static uint32_t   g_last_activity_ms = 0;  /* last user interaction timestamp */

/* ===== 步态状态 ===== */
static uint8_t    g_gait_running = 0;        /* 0=stopped, 1=running */
static uint32_t   g_gait_start_ms = 0;       /* gait cycle start timestamp */
static int32_t    g_gait_left_hip = 0;       /* current left hip target (mdeg) */
static int32_t    g_gait_right_hip = 0;      /* current right hip target (mdeg) */

/* ===== 安全状态 (fault code bitmask) =====
 *  复用contract.h中定义，同时定义关节限位错误使用Bit 9~12
 */
#define FAULT_LHIP_LIMIT_POS   0x0200   /* Bit 9  - Left hip +limit */
#define FAULT_LHIP_LIMIT_NEG   0x0400   /* Bit 10 - Left hip -limit */
#define FAULT_RHIP_LIMIT_POS   0x0800   /* Bit 11 - Right hip +limit */
#define FAULT_RHIP_LIMIT_NEG   0x1000   /* Bit 12 - Right hip -limit */
static uint16_t   g_fault_code = 0;

/* ===== 前向声明 ===== */
extern int32_t trapezoidal_interpolate(int32_t current, int32_t target, int32_t max_rate, int32_t period_ms);
static void gait_step(void);
static void safety_check(void);
static uint8_t safety_clamp_targets(void);

/* Clamp helper */
static int32_t clamp_int32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Activity helper */
static void mark_activity(void) {
    g_last_activity_ms = HAL_GetTick();
}

/* Getter functions for LCD display */
uint8_t ctrl_get_mode(void)     { return (uint8_t)g_ctrl_mode; }
uint8_t ctrl_get_selected(void) { return 0; }
int32_t ctrl_get_target_pos(void) { return g_gait_left_hip; }
int32_t ctrl_get_target_torque(void) { return g_gait_right_hip; }
uint8_t ctrl_get_pos_manual(void) { return g_gait_running; }

/* ========== 步态轨迹生成 ==========
 *  根据当前时间在步态表中插值，计算左右髋目标角度
 */
static void gait_step(void)
{
    if (!g_gait_running) return;

    uint32_t elapsed = HAL_GetTick() - g_gait_start_ms;
    uint32_t phase = elapsed % GAIT_PERIOD_MS;           /* 0 ~ GAIT_PERIOD_MS */
    float t = (float)phase / (float)GAIT_PERIOD_MS;      /* 0.0 ~ 1.0 */
    float seg = t * (float)(GAIT_NUM_POINTS - 1);        /* 0.0 ~ 4.0 */
    uint8_t idx = (uint8_t)seg;
    float frac = seg - (float)idx;

    if (idx >= GAIT_NUM_POINTS - 1) idx = GAIT_NUM_POINTS - 2;

    /* Linear interpolation between key points */
    int32_t lhip0 = g_gait_table[idx][0];
    int32_t lhip1 = g_gait_table[idx + 1][0];
    int32_t rhip0 = g_gait_table[idx][1];
    int32_t rhip1 = g_gait_table[idx + 1][1];

    int32_t lhip_target = lhip0 + (int32_t)((float)(lhip1 - lhip0) * frac);
    int32_t rhip_target = rhip0 + (int32_t)((float)(rhip1 - rhip0) * frac);

    /* Trapezoidal interpolation for smooth motion */
    g_gait_left_hip  = trapezoidal_interpolate(g_gait_left_hip,  lhip_target, GAIT_MAX_VEL_MDEG_S, 100);
    g_gait_right_hip = trapezoidal_interpolate(g_gait_right_hip, rhip_target, GAIT_MAX_VEL_MDEG_S, 100);
}

/* ========== 安全检查 ========== */
static void safety_check(void)
{
    /* Check both motors online (communication timeout) */
    uint8_t m1_online = can_motor_is_online(0);
    uint8_t m2_online = can_motor_is_online(1);

    if (!m1_online || !m2_online) {
        g_fault_code |= FAULT_COMM_LOST;
    } else {
        g_fault_code &= ~FAULT_COMM_LOST;
    }

    /* If any fault, force zero-torque */
    if (g_fault_code != FAULT_NONE && g_ctrl_mode != MODE_ZERO_TORQUE) {
        printf("[SAFETY] Fault detected (0x%02X) → ZERO-TORQUE\r\n", g_fault_code);
        g_ctrl_mode = MODE_ZERO_TORQUE;
        g_gait_running = 0;
    }
}

/* ========== 目标值安全裁剪 ==========
 *  返回: 1 = 发生了裁剪(有限位触发), 0 = 正常
 */
static uint8_t safety_clamp_targets(void)
{
    uint8_t clamped = 0;

    /* Motor 1 = Left Hip (ID=1) */
    if (g_gait_left_hip > HIP_POS_MAX_MDEG)  { g_gait_left_hip = HIP_POS_MAX_MDEG;  g_fault_code |= FAULT_LHIP_LIMIT_POS; clamped = 1; }
    else { g_fault_code &= ~FAULT_LHIP_LIMIT_POS; }
    if (g_gait_left_hip < HIP_POS_MIN_MDEG)  { g_gait_left_hip = HIP_POS_MIN_MDEG;  g_fault_code |= FAULT_LHIP_LIMIT_NEG; clamped = 1; }
    else { g_fault_code &= ~FAULT_LHIP_LIMIT_NEG; }

    /* Motor 2 = Right Hip (ID=2) */
    if (g_gait_right_hip > HIP_POS_MAX_MDEG) { g_gait_right_hip = HIP_POS_MAX_MDEG; g_fault_code |= FAULT_RHIP_LIMIT_POS; clamped = 1; }
    else { g_fault_code &= ~FAULT_RHIP_LIMIT_POS; }
    if (g_gait_right_hip < HIP_POS_MIN_MDEG) { g_gait_right_hip = HIP_POS_MIN_MDEG; g_fault_code |= FAULT_RHIP_LIMIT_NEG; clamped = 1; }
    else { g_fault_code &= ~FAULT_RHIP_LIMIT_NEG; }

    return clamped;
}

int main(void)
{
    uint8_t key;
    uint8_t i = 0;
    /* v1.6: ts 变量不再使用 (移除了 g_test_buffer 填充) */

    /* ---- 先驱动 PA9 (TX) 为高电平，防止上电后 TX 低电平产生 \0 乱码 ---- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODE9_0;
    GPIOA->ODR   |= GPIO_ODR_OD9;

    HAL_Init();
    sys_stm32_clock_init(336, 25, 2, 7);
    delay_init(168);

    /* 168MHz 稳定后再初始化 UART，避免 BRR 随时钟切换变化 */
    usart_init(115200);

    printf("\r\n\r\n========== v1.6.8 ExtSRAM + LCD + 1MB R/W + Dual CAN + Motor Control ==========\r\n");
    printf("[BOOT] HAL_Init OK\r\n");
    printf("[BOOT] SystemClock: 168MHz (HSE 25MHz + PLL)\r\n");
    printf("[BOOT] delay_init OK\r\n");
    printf("[BOOT] usart_init OK (115200, 8N1)\r\n");

    led_init();
    printf("[BOOT] led_init OK\r\n");

    lcd_init();
    printf("[BOOT] lcd_init OK (ID:0x%04X)\r\n", lcddev.id);

    key_init();
    printf("[BOOT] key_init OK\r\n");

    sram_init();
    printf("[BOOT] sram_init OK (FSMC Bank3, 0x68000000)\r\n");

    sys_info_init();
    printf("[BOOT] sys_info_init OK\r\n");

    /* ---- CAN1/CAN2 初始化 (1Mbps) ---- */
    can_motor_init();

    /* ---- CAN 总线检测：发送测试帧检查是否有设备应答 ---- */
    uint8_t can1_ok = can_bus_detect(&g_can1_handle, 1);
    /* Skip CAN2 detect for now - it may corrupt CAN state for extended frames */
    uint8_t can2_ok = 0;  /* Assume OK for now */
    printf("[BOOT] CAN1 (CyberGear ID=1,2): %s\r\n", can1_ok == 0 ? "devices detected" : "NO DEVICES (bus open)");
    printf("[BOOT] CAN2 (RS01): %s\r\n", can2_ok == 0 ? "devices detected" : "NO DEVICES (bus open)");

    /* ---- 电机状态映射 (LCD显示ID → CAN总线 + 实际CAN ID) ---- */
    /* CAN1 (CyberGear) 显示 0,1 → 实际 ID 1,2 */
    can_motor_set_mapping(0, &g_can1_handle, 1);
    can_motor_set_mapping(1, &g_can1_handle, 2);
    /* CAN2 (RS01) 显示 2,3,4,5 → 实际 ID 0x10,0x11,0x12,0x13 */
    can_motor_set_mapping(2, &g_can2_handle, 0x10);
    can_motor_set_mapping(3, &g_can2_handle, 0x11);
    can_motor_set_mapping(4, &g_can2_handle, 0x12);
    can_motor_set_mapping(5, &g_can2_handle, 0x13);

    /* ---- 电机上电：CyberGear (CAN1) 发送零扭矩使能帧 ---- */
    /* RS01 (CAN2) 已在 can_motor_init() 中自动使能 */
    HAL_Delay(50);
    cybergear_mit_enable(&g_can1_handle, 1);
    HAL_Delay(5);
    printf("[MOTOR] CyberGear ID=1: enabled (zero-torque)\r\n");
    cybergear_mit_enable(&g_can1_handle, 2);
    HAL_Delay(5);
    printf("[MOTOR] CyberGear ID=2: enabled (zero-torque)\r\n");
    printf("[BOOT] All motors powered up and in zero-torque mode\r\n");

    printf("[BOOT] ========== Init Complete ==========\r\n\r\n");

    /* Initialize activity timestamp */
    mark_activity();

    /* v1.6: 已移除 g_test_buffer 全量填充！
     *   之前: for (ts = 0; ts < 250000; ts++) g_test_buffer[ts] = ts;
     *   原因: 填充整个 1MB 外部 SRAM 会覆盖 LWIP_HEAP/PBUF_POOL 等分区,
     *         导致 udp_pcb->recv 函数指针被破坏, 触发 HardFault。
     *   出厂 SRAM 测试请调用 sram_full_test()。
     */

    while (1)
    {
        /* ---- 处理 CAN 接收数据，更新电机在线状态 ---- */
        {
            JointStatus_t tmp;
            can_motor_receive_status(&g_can1_handle, &tmp);
            can_motor_receive_status(&g_can2_handle, &tmp);
        }

        /* ---- 电机在线状态检测 ---- */
        can_motor_online_tick();

        /* ---- Idle timeout safety: auto zero-torque after 15s no activity ---- */
        if (g_ctrl_mode != MODE_ZERO_TORQUE &&
            (HAL_GetTick() - g_last_activity_ms > IDLE_TIMEOUT_MS)) {
            g_ctrl_mode = MODE_ZERO_TORQUE;
            g_gait_running = 0;
            g_gait_left_hip = 0;
            g_gait_right_hip = 0;
            printf("[SAFETY] Idle timeout (15s) → ZERO-TORQUE\r\n");
        }

        /* ---- 周期发送电机控制命令 (每100ms) ---- */
        static uint32_t last_motor_ping = 0;
        if (HAL_GetTick() - last_motor_ping >= 100) {
            last_motor_ping = HAL_GetTick();

            /* ---- Safety check first ---- */
            safety_check();

            /* Safety: if offline, force zero-torque */
            uint8_t safe_mode = g_ctrl_mode;
            if (g_fault_code != FAULT_NONE) {
                safe_mode = MODE_ZERO_TORQUE;
            }

            /* Send command based on mode */
            switch (safe_mode) {
                case MODE_GAIT: {
                    /* Gait mode: left/right hip alternating */
                    gait_step();
                    safety_clamp_targets();

                    /* Motor 1 = Left Hip (ID=1) */
                    cybergear_mit_set_position(&g_can1_handle, 1, g_gait_left_hip, POS_KP, POS_KD);
                    /* Motor 2 = Right Hip (ID=2) */
                    cybergear_mit_set_position(&g_can1_handle, 2, g_gait_right_hip, POS_KP, POS_KD);
                    break;
                }
                default: /* MODE_ZERO_TORQUE: both motors zero-torque */
                    cybergear_mit_zero_torque(&g_can1_handle, 1);
                    cybergear_mit_zero_torque(&g_can1_handle, 2);
                    break;
            }

            /* RS01 (CAN2): keepalive */
            {
                CAN_TxHeaderTypeDef txh;
                uint8_t txd[8] = {0};
                uint32_t mb;
                memset(&txh, 0, sizeof(txh));
                txh.DLC = 8;
                txh.IDE = CAN_ID_STD;
                txh.RTR = CAN_RTR_DATA;
                txh.StdId = 1;
                HAL_CAN_AddTxMessage(&g_can2_handle, &txh, txd, &mb);
                txh.StdId = 2;
                HAL_CAN_AddTxMessage(&g_can2_handle, &txh, txd, &mb);
                txh.StdId = 3;
                HAL_CAN_AddTxMessage(&g_can2_handle, &txh, txd, &mb);
                txh.StdId = 4;
                HAL_CAN_AddTxMessage(&g_can2_handle, &txh, txd, &mb);
            }
        }

        /* ---- LCD 状态显示 (300ms 刷新) ---- */
        lcd_status_task();

        /* ---- 按键处理 ---- */
        key = key_scan();

        if (key == KEY0_PRES)
        {
            /* KEY0 short: 切换控制模式 Zero-Torque ↔ Gait */
            mark_activity();
            g_ctrl_mode = (CtrlMode_t)((g_ctrl_mode + 1) % 2);
            const char *mode_str[] = {"ZERO-TORQUE", "GAIT"};
            printf("[KEY] KEY0: Mode = %s\r\n", mode_str[g_ctrl_mode]);
            if (g_ctrl_mode == MODE_GAIT) {
                g_gait_running = 0;  /* Start gait in STOPPED state */
                g_gait_left_hip = 0;
                g_gait_right_hip = 0;
                printf("[GAIT] Mode selected (Left+Right Hip, press WKUP to start/stop)\r\n");
            }
        }
        else if (key == KEY0_LONG_PRES)
        {
            /* KEY0 long (>800ms): Emergency STOP → all motors zero-torque */
            g_ctrl_mode = MODE_ZERO_TORQUE;
            g_gait_running = 0;
            g_gait_left_hip = 0;
            g_gait_right_hip = 0;
            can_motor_disable_bus(&g_can1_handle);
            HAL_Delay(10);
            cybergear_mit_zero_torque(&g_can1_handle, 1);
            cybergear_mit_zero_torque(&g_can1_handle, 2);
            printf("[E-STOP] KEY0 long press → ALL MOTORS ZERO-TORQUE, GAIT STOPPED\r\n");
            mark_activity();
        }
        else if (key == WKUP_PRES)
        {
            mark_activity();
            if (g_ctrl_mode == MODE_GAIT) {
                /* WKUP short in GAIT mode: Start / Stop gait */
                g_gait_running = !g_gait_running;
                if (g_gait_running) {
                    g_gait_start_ms = HAL_GetTick();
                    printf("[GAIT] START (cycle=%us, L+R Hip alternating)\r\n", GAIT_PERIOD_MS / 1000);
                } else {
                    g_gait_left_hip = 0;
                    g_gait_right_hip = 0;
                    printf("[GAIT] STOP (returning to zero)\r\n");
                }
            } else {
                printf("[KEY] WKUP: (in ZERO-TORQUE mode, switch to GAIT first with KEY0)\r\n");
            }
        }
        else if (key == WKUP_LONG_PRES)
        {
            /* WKUP long (>800ms): Reset gait phase (only in GAIT mode) */
            mark_activity();
            if (g_ctrl_mode == MODE_GAIT) {
                /* Reset gait phase to start */
                g_gait_start_ms = HAL_GetTick();
                g_gait_left_hip = 0;
                g_gait_right_hip = 0;
                printf("[GAIT] Phase reset (back to 0%%)\r\n");
            } else {
                printf("[KEY] WKUP long: (in ZERO-TORQUE mode, no adjustment)\r\n");
            }
        }
        else
        {
            delay_ms(10);
        }

        i++;

        if (i == 20)
        {
            i = 0;
            LED0_TOGGLE();
        }
    }
}
#else
/* 完整功能版本: 外骨骼控制系统 */
int main(void)
{
    /* ★★★ 检查复位标志（不做早期 IWDG 配置 — 留到 MX_IWDG_Init 统一处理） ★★★ */
    if (RCC->CSR & RCC_CSR_IWDGRSTF) {
        RCC->CSR |= RCC_CSR_RMVF;
    }
    /* 如果 Option Bytes 里 IWDG_HW=1, IWDG 已经在跑了, 这里尽早喂一口 */
    if (IWDG->SR & IWDG_SR_PVU) {
        IWDG->KR = 0xAAAA;
    }

    /* ★★★ 最早串口：确认代码从 Reset 一路跑到这 ★★★ */
    early_uart_init();
    printf("\r\n\r\n[DBG] ========== RESET OCCURRED ==========\r\n");
    printf("[DBG] RCC_CSR=0x%08lX (RSTF:%s%s%s%s)\r\n", (unsigned long)RCC->CSR,
           (RCC->CSR & RCC_CSR_IWDGRSTF)  ? " IWDG" : "",
           (RCC->CSR & RCC_CSR_SFTRSTF)   ? " SFT"  : "",
           (RCC->CSR & RCC_CSR_PORRSTF)    ? " POR"  : "",
           (RCC->CSR & RCC_CSR_PINRSTF)    ? " PIN"  : "");
    RCC->CSR |= RCC_CSR_RMVF;

    HAL_Init();

    /* ★★★ 先配时钟到 168MHz ★★★ */
    HAL_RCC_EnableCSS();
    HAL_StatusTypeDef clk_status = SystemClock_Config();
    if (clk_status != HAL_OK) {
        printf("[BOOT     0 ms] *** WARNING: HSE 25MHz startup FAILED! Running from HSI 16MHz\r\n");
    }

    /* ★★★ 时钟切换后立即修正 USART1 波特率 ★★★
     * early_uart_init() 按 HSI 16MHz 配 BRR=0x08AE
     * SystemClock_Config() 后 APB2=84MHz, BRR 必须改为 0x2D93
     * 否则波特率从 115200 漂移到 604800, 后续 printf 全乱码 */
    USART1->BRR = 0x2D93;  /* 84MHz / 115200 = 729.17 → BRR=0x2D93 */

    /* ★★★ 初始化 delay 子系统（需要在168MHz时钟配置后） ★★★ */
    delay_init(168);

    /* ★★★ 先初始化外部 SRAM (必须在 MPU_Config 和 interpolation_init 之前) ★★★ */
    sram_init();
    g_sram_ready = 1;  /* 标记 SRAM 就绪, 允许 sys_info_update_sram() 扫描 */
    printf("[BOOT     0 ms] SRAM: OK (FSMC Bank3, 0x68000000)\r\n");

    /* ★★★ 清零外部 SRAM 动态区 (防止跨复位残留脏数据) ★★★
     * 外部 SRAM 掉电保持/软复位不清零, 上一次崩溃的垃圾数据
     * (如 LWIP Heap 元数据、PCB 链表指针) 会污染下一次启动.
     * MEM_TABLE (0x68000000, 128KB) 由 interpolation_init() 填充, 此处跳过.
     * 其余区域 (LWIP_HEAP / PBUF_POOL / FIFO_EXT / LOG_BUF / STATIC_BUF + 空闲区)
     * 共 896KB 全部清零, 确保 mem_init() 建立干净空闲链表. */
    {
        uint32_t sanitize_start = MEM_TABLE_ADDR + MEM_TABLE_SIZE;  /* 0x68020000 */
        uint32_t sanitize_end   = MEM_EXT_SRAM_START + MEM_EXT_SRAM_SIZE;  /* 0x68100000 */
        uint32_t sanitize_size  = sanitize_end - sanitize_start;
        memset((void *)sanitize_start, 0, sanitize_size);
        printf("[BOOT     0 ms] SRAM: Sanitized %luKB (0x%08lX - 0x%08lX)\r\n",
               (unsigned long)(sanitize_size / 1024),
               (unsigned long)sanitize_start,
               (unsigned long)sanitize_end);
    }

    /* ★★★ 扫描并清零内部 RAM 中的脏指针 (防止 IWDG 软复位残留) ★★★
     * 0x6806C666 / 0x6805C000 / 0xD005C000 是已知脏值,
     * 若出现在内部 RAM (作为链表指针或回调) 会导致 HardFault.
     * 只清静态区前 64KB (0x20000000 ~ 0x20010000), 远离栈. */
    {
        uint32_t *p = (uint32_t *)0x20000000;
        uint32_t *end = (uint32_t *)0x20010000;
        uint32_t bad1 = 0x6806C666;
        uint32_t bad2 = 0x6805C000;
        uint32_t bad3 = 0xD005C000;
        uint32_t cnt = 0;
        while (p < end) {
            if (*p == bad1 || *p == bad2 || *p == bad3) {
                *p = 0;
                cnt++;
            }
            p++;
        }
        if (cnt > 0) {
            printf("[BOOT     0 ms] RAM: Sanitized %lu dirty pointers\r\n", (unsigned long)cnt);
        }
    }

    /* ★★★ 最后开 MPU ★★★ */
    MPU_Config();

    MX_GPIO_Init();

    /* v1.6.7: key_init 必须在 MX_GPIO_Init 之后调用
     * MX_GPIO_Init 将 PE4 配为 EN_ARM_1 输出 (已移到 PE6)
     * key_init 将 PE4(KEY0) 配为上拉输入, PA0(WKUP) 配为下拉输入
     * 若顺序颠倒, MX_GPIO_Init 会覆盖 key_init 的输入配置 → 按键永远读到固定值 */
    key_init();

    printf("\r\n\r\n");
    printf("[BOOT     0 ms] ========== JuAo Power Exoskeleton Control System v1.6.8 ==========\r\n");
    printf("[BOOT     0 ms] Early UART: OK (HSI 16MHz, 115200)\r\n");

    MX_USART1_UART_Init();
    printf("[BOOT %5lu ms] System Clock: %lu MHz (%s)\r\n", (unsigned long)HAL_GetTick(),
           (unsigned long)(SystemCoreClock / 1000000),
           (clk_status == HAL_OK) ? "HSE 25MHz + PLL" : "HSI 16MHz (FALLBACK!)");

    /* LED startup: 3 quick flashes */
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);
    delay_ms(50);
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_SET);
    delay_ms(50);
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);
    delay_ms(50);
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_SET);

    /* ★★★ 初始化 LCD 与状态显示 ★★★ */
    BOOT_TRACE("Init: LCD...");
    lcd_init();
    BOOT_TRACE("LCD: OK (ID:0x%04X)", lcddev.id);

    BOOT_TRACE("Init: System Info...");
    sys_info_init();
    BOOT_TRACE("SysInfo: OK");

    /* ★★★ v1.6.3: 安全模块必须在 CAN 之前初始化, 确保 EN=LOW ★★★ */
    BOOT_TRACE("Init: Safety...");
    safety_init();
    /* ★★★ v1.6.3: 立即请求使能系统, 确保 EN 引脚在安全窗口后变 HIGH ★★★
     * 默认 recovery_counter=0, 需要 100 次主循环 (约 10s) 才能使能.
     * 显式调用后 recovery_counter=100, 下一次 safety_task_run() 即使能.
     * 但若驱动板 EN=LOW(禁能), 则必须先使能 EN 才能控制电机. */
    safety_request_enable();
    BOOT_TRACE("Safety: OK (EN=LOW→HIGH requested)");

    /* ★★★ v1.6.3: 模式管理器必须在 ISR 启动前初始化 ★★★ */
    mode_manager_init();
    BOOT_TRACE("ModeMgr: OK (ZERO_TORQUE, ABO=ON)");

    #ifndef CAN_DISABLED
    BOOT_TRACE("Init: CAN1/CAN2 (1Mbps)...");
    can_motor_init();
    BOOT_TRACE("CAN: OK");

    /* v1.6.3: 电机在线状态映射 (LCD显示ID → CAN总线 + 实际CAN ID)
     * CAN1 (CyberGear) 显示 0,1 → 实际 ID 1,2
     * CAN2 (RS01) 显示 2,3,4,5 → 实际 ID 0x10,0x11,0x12,0x13 */
    can_motor_set_mapping(0, &hcan1, 1);
    can_motor_set_mapping(1, &hcan1, 2);
    can_motor_set_mapping(2, &hcan2, 0x10);
    can_motor_set_mapping(3, &hcan2, 0x11);
    can_motor_set_mapping(4, &hcan2, 0x12);
    can_motor_set_mapping(5, &hcan2, 0x13);
    BOOT_TRACE("Motor mapping: OK (6 motors)");
#endif

    BOOT_TRACE("Init: I2C / EEPROM (400kHz)...");
    eeprom_init();
    BOOT_TRACE("I2C: OK");

    BOOT_TRACE("Init: Interpolation Tables...");
    interpolation_init();
    BOOT_TRACE("Interpolation: OK");

    BOOT_TRACE("Init: EEPROM Params (ABO bias + PID defaults)...");
    eeprom_params_load_all();
    BOOT_TRACE("EEPROM Params: OK");

    #ifndef CAN_DISABLED
    BOOT_TRACE("Init: TIM6 Control ISR (1ms)...");
    control_isr_init();
    BOOT_TRACE("TIM6: OK");
#endif

    BOOT_TRACE("Init: UDP Protocol FIFO...");
    udp_protocol_init();
    BOOT_TRACE("UDP FIFO: OK");

    /* 默认 STANDALONE 模式: 断网也能跑, 调试方便
     * 需要上位机模式时: 注释掉下面一行, 或按键切换 */
    g_comm_mode = COMM_MODE_STANDALONE;
    local_set_mode(0);  /* 本地默认零扭矩模式 (安全) */
    BOOT_TRACE("Comm Mode: STANDALONE (WKUP_S=Gait, KEY0_S=ArmAssist, WKUP_L=Zero, KEY0_L=EStop)");

    /* ★★★ 长耗时初始化前先喂狗（防 HW IWDG 已启用时中途复位） ★★★ */
    IWDG->KR = 0xAAAA;

    BOOT_TRACE("Init: Ethernet PHY + LwIP Stack (may take 2-3s)...");
    {
        /* === 双保险：lwip_comm_init() 之前紧邻清零所有动态区
         * 防止跨复位 SRAM 残留导致 mem_init() 沿用脏链表
         * 用硬编码地址，不依赖任何宏，确保 100% 正确 */
        memset((void *)0x68020000, 0, 64 * 1024);   /* LWIP Heap      64KB */
        memset((void *)0x68030000, 0, 32 * 1024);   /* PBUF Pool       32KB */
        memset((void *)0x68038000, 0, 64 * 1024);   /* FIFO Ext        64KB */
        memset((void *)0x68048000, 0, 64 * 1024);   /* Log Buffer      64KB */
        memset((void *)0x68058000, 0, 16 * 1024);   /* Static Buffer   16KB */
        BOOT_TRACE("SRAM: Dynamic regions sanitized (LWIP+PBUF+FIFO+LOG+STATIC)");

        /* ★★★ 扫描内部 RAM 静态区, 清零所有外部 SRAM 范围的脏指针 ★★★
         * IWDG 软复位不清内部 RAM .bss, MEMP 池里的 udp_pcb.recv 等指针
         * 会保留上次脏值 (如 0x6806C666, 在外部 SRAM 范围内).
         * 不能全量清零静态区 (会覆盖 CAN/I2C/EEPROM 等模块状态),
         * 只清零外部 SRAM/外设范围的可疑指针, 同时 recv 回调检查已加了上限,
         * 双保险确保 0x6806C666 不会被调用. */
        {
            uint32_t *p = (uint32_t *)0x20000000;
            uint32_t *end = (uint32_t *)0x20011F00;  /* 整个静态区, 不触及栈 */
            uint32_t sram_start = 0x68000000;
            uint32_t sram_end   = 0x68100000;
            uint32_t bad1 = 0xD005C000;
            uint32_t bad2 = 0xC0000000;  /* 外设范围起始 */
            uint32_t cnt = 0;
            while (p < end) {
                uint32_t val = *p;
                if ((val >= sram_start && val < sram_end) ||
                    val == bad1 ||
                    (val >= bad2 && val < 0xE0100000)) {  /* 外设范围 */
                    *p = 0;
                    cnt++;
                }
                p++;
            }
            BOOT_TRACE("MEMP: Scanned & cleared %lu stale pointers in RAM", (unsigned long)cnt);
        }

        /* ★★★ 终极保险: 显式清零 UDP PCB 池的 recv/recv_arg/next ★★★
         * MEMP_MEM_INIT=1 已确保 memp_init() 时清零所有池,
         * 但为了双保险, 在 lwip_init() 之前再显式清一次 UDP PCB 池,
         * 彻底杜绝 0x6806C666 这类脏回调指针残留. */
        memp_udp_pcb_pool_sanitize();
        BOOT_TRACE("MEMP: UDP PCB pool sanitized");

        uint32_t eth_start = HAL_GetTick();
        uint8_t eth_ret = lwip_comm_init();
        uint32_t eth_elapsed = HAL_GetTick() - eth_start;
        IWDG->KR = 0xAAAA;  /* 以太网初始化后立即喂狗 */
        if (eth_ret != 0) {
            BOOT_TRACE("Ethernet/Lwip: FAILED (code=%d, %lums), continuing without network", eth_ret, (unsigned long)eth_elapsed);
        } else {
            BOOT_TRACE("Ethernet/LwIP: OK (%lums)", (unsigned long)eth_elapsed);
            BOOT_TRACE("Init: UDP Port 5001...");
            udp_net_init();
            BOOT_TRACE("UDP: OK");
        }
    }

    BOOT_TRACE("========== Init Complete ==========");

    IWDG->KR = 0xAAAA;  /* 自检前喂一口 */
    BOOT_TRACE("Runtime Self-Test...");
    {
        /* v1.6: sram_test 已改为仅测试空闲区 (0x6805C000 之后), 不会覆盖已用分区 */
        uint8_t sram_ret = sram_test();
        BOOT_TRACE("SRAM Test: %s (idle region only)", sram_ret == 0 ? "PASS" : "FAIL");
    }
    if (g_lwip_inited) {
        uint32_t phy_bsr = 0;
        HAL_ETH_ReadPHYRegister(&g_eth_handle, 1, &phy_bsr);
        BOOT_TRACE("PHY BSR:  0x%04lX (Link=%s)", (unsigned long)phy_bsr,
                   (phy_bsr & 0x0004) ? "UP" : "DOWN");
        BOOT_TRACE("ETH Link:  %s", netif_is_link_up(&g_lwip_netif) ? "UP" : "DOWN");
        BOOT_TRACE("IP Addr :  %s", ip4addr_ntoa(&g_lwip_netif.ip_addr));
    } else {
        BOOT_TRACE("ETH:       Not initialized (no network)");
    }
    #ifndef CAN_DISABLED
    BOOT_TRACE("CAN1 Mode: %s", (hcan1.Init.Mode == CAN_MODE_NORMAL) ? "NORMAL" : "SILENT/LOOPBACK");
    BOOT_TRACE("CAN2 Mode: %s", (hcan2.Init.Mode == CAN_MODE_NORMAL) ? "NORMAL" : "SILENT/LOOPBACK");
    #endif

    IWDG->KR = 0xAAAA;  /* 启动 MX_IWDG 前再喂一口 (防 HW IWDG 已启用时超时) */
    BOOT_TRACE("Starting IWDG (4s timeout, will reset if tasks stall)...");
    MX_IWDG_Init();
    BOOT_TRACE("IWDG: ENABLED (4s timeout)");
    BOOT_TRACE("========== Self-Test Complete ==========\r\n");

    /* Main loop - 非阻塞，集成所有任务 */
    uint32_t last_tick = HAL_GetTick();
    uint32_t last_lcd_tick = 0;
    uint32_t last_status_tick = 0;
    uint32_t last_sram_tick = 0;
    uint32_t last_eeprom_save_tick = 0;
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);

    while (1)
    {
        /* ---- 喂狗保护带 (关中断保险喂狗, 防任务跑飞) ---- */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        HAL_IWDG_Refresh(&hiwdg);
        if (!primask) __enable_irq();

        uint32_t loop_start = HAL_GetTick();

        /* ---- 业务任务 ---- */
        sys_info_work_start();

        /* ★ v1.6.8fix2: 电机在线状态超时检测 (原仅最小版 main 有, 完整版遗漏) */
        can_motor_online_tick();

        /* v1.4: 智能模式切换
         *   - 上电默认 STANDALONE (自主模式, 零力矩, 不依赖上位机)
         *   - 收到上位机包 → 自动切 HOST (udp_net.c 中处理)
         *   - HOST 模式心跳超时 (>3s) → 自动切回 STANDALONE, 电机不急停 */
        if (g_comm_mode == COMM_MODE_HOST) {
            if (!comm_is_heartbeat_ok()) {
                /* 心跳超时 → 自动切回 STANDALONE, 不清电机使能, 不触发急停 */
                g_comm_mode = COMM_MODE_STANDALONE;
                local_set_mode(0);  /* 回零力矩 (OPERATION_ENABLE) */
                local_gait_start_stop(0);
                SAFETY_LOCK();
                g_safety_state.fault_code &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
                SAFETY_UNLOCK();
                printf("[MODE] HOST → STANDALONE (heartbeat timeout, no estop)\r\n");
            }
        } else {
            /* STANDALONE 模式: 确保通信故障标志始终清除, 电机不急停 */
            if (g_safety_state.fault_code & (FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT)) {
                SAFETY_LOCK();
                g_safety_state.fault_code &= ~(FAULT_COMM_LOST | FAULT_CAN1_TIMEOUT | FAULT_CAN2_TIMEOUT);
                SAFETY_UNLOCK();
            }
        }

        network_task_run();
        control_task_run();
        eeprom_task_run();

        /* v1.2: LwIP 初始化失败自动重试 (每 5 秒一次)
         * PHY 冷启动偶发锁死, 重试可恢复 */
        {
            static uint32_t lwip_retry_ts = 0;
            if (!g_lwip_inited) {
                if (HAL_GetTick() - lwip_retry_ts >= 5000) {
                    lwip_retry_ts = HAL_GetTick();
                    BOOT_TRACE("[MAIN] LwIP re-init attempt...");
                    IWDG->KR = 0xAAAA;  /* 长耗时前喂狗 */
                    if (lwip_comm_init() == 0) {
                        udp_net_init();
                        BOOT_TRACE("[MAIN] LwIP re-init OK, network restored");
                    } else {
                        BOOT_TRACE("[MAIN] LwIP re-init FAILED, will retry in 5s");
                    }
                }
            }
        }

        uint32_t now = HAL_GetTick();

        /* ---- EEPROM ABO 偏置持久化 (30秒一次, 偏置估计收敛后存盘) ----
         *  偏置变化很慢 (~1s 时间常数), 30s 存一次足够
         *  写入走非阻塞状态机, 只排队不阻塞主循环 */
        if (now - last_eeprom_save_tick >= 30000)
        {
            last_eeprom_save_tick = now;
            eeprom_params_save_abo_bias();
        }

        /* ---- LCD 状态显示 (300ms 刷新一次, 不含 SRAM 采样) ---- */
        if (now - last_lcd_tick >= 300)
        {
            last_lcd_tick = now;
            sys_info_update();
            lcd_status_task();
        }

        /* ---- 按键扫描 + 模式状态机 (v1.6.3) ---- */
        mode_fsm_process();

        sys_info_work_end();

        /* ---- 兜底: 单次循环超时检测 + 兜底喂狗 ---- */
        {
            uint32_t loop_elapsed = HAL_GetTick() - loop_start;
            if (loop_elapsed > 900) {
                /* 单次循环 > 900ms, 强制喂狗防复位, 报警 */
                HAL_IWDG_Refresh(&hiwdg);
                printf("[WARN] Loop stall: %lums (>900ms threshold)\r\n", (unsigned long)loop_elapsed);
            }
        }

        /* 喂狗：在所有任务之后 */
        HAL_IWDG_Refresh(&hiwdg);

        /* LED 闪烁 (100ms) */
        if (now - last_tick >= 100)
        {
            last_tick = now;
            HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
        }

        /* 每秒状态打印 */
        if (now - last_status_tick >= 1000)
        {
            last_status_tick = now;
            uint8_t  hb  = comm_is_heartbeat_ok();
            uint32_t cmd_v = safe_cmd_get_version();
            const char *mode_str = (g_comm_mode == COMM_MODE_HOST) ? "HOST" : "LOCAL";
            const char *local_str = local_get_mode() ? "GAIT" : "ZERO";
            printf("[%lu] Mode:%s/%s CPU:%u%% HB:%s CmdV:%lu Fault:0x%04X WKUP=%d KEY0=%d\r\n",
                   (unsigned long)(now / 1000),
                   mode_str, local_str,
                   sys_info_get()->cpu_usage,
                   hb ? "OK" : "LOST",
                   (unsigned long)cmd_v,
                   (unsigned)g_safety_state.fault_code,
                   (int)WK_UP, (int)KEY0);
        }

        /* v1.6.3+fix: 主循环 1ms 节流, 防止空转占满 CPU (原92%→<5%),
         * 业务任务周期都是 ms 级 (TIM6 已做 1kHz 硬实时), 主循环跑 1kHz 足够. */
        HAL_Delay(1);
    }
}
#endif /* V01_BASIC_ONLY */
