/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - v0.1 基础版(外置SRAM+LCD硬件资源展示)
  ******************************************************************************
  */
/* USER CODE END Header */

/* ===== 功能开关：定义V01_BASIC_ONLY则只保留v0.1基础功能(SRAM+LCD) ===== */
#define V01_BASIC_ONLY
/* ========================================================================== */

/* Includes ------------------------------------------------------------------*/
#ifndef V01_BASIC_ONLY
#include "main.h"
#include "bsp_config.h"
#include "contract.h"
#include "sram.h"
#include "can_motor.h"
#include "ethernet.h"
#include "lwip_comm.h"
#include "udp_net.h"
#include "udp_protocol.h"
#include "tasks.h"
#include "control_isr.h"
#include "eeprom.h"
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
#ifndef V01_BASIC_ONLY
#include "./BSP/SYS_INFO/sys_info.h"
#include "./BSP/LCD_STATUS/lcd_status.h"
#endif

#include <stdio.h>
#include <string.h>

void MPU_Config(void);

/* Private variables ---------------------------------------------------------*/
#ifndef V01_BASIC_ONLY
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim6;
SRAM_HandleTypeDef hsram1;
ETH_HandleTypeDef heth;
UART_HandleTypeDef huart1;
IWDG_HandleTypeDef hiwdg;
#endif

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef SystemClock_Config(void);
#ifndef V01_BASIC_ONLY
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void early_uart_init(void);
#endif

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

#define BOOT_TRACE(fmt, ...)  printf("[BOOT %5lu ms] " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)

/**
  * @brief Emergency UART init - works from HSI 16MHz before SystemClock config
  *        PA9=TX, PA10=RX, 115200, 8N1
  */
static void early_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

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

/* ===== v0.1 基础功能: 外置SRAM测试 ===== */
/* 测试缓冲区, 起始地址为: SRAM_BASE_ADDR */
#if (__ARMCC_VERSION >= 6010050)
uint32_t g_test_buffer[250000] __attribute__((section(".bss.ARM.__at_0x68000000")));
#else
uint32_t g_test_buffer[250000] __attribute__((at(SRAM_BASE_ADDR)));
#endif

/**
 * @brief       外部内存测试(最多支持1M字节内存测试)
 * @param       x: LCD显示x坐标
 * @param       y: LCD显示y坐标
 * @retval      无
 */
void fsmc_sram_test(uint16_t x, uint16_t y)
{
    uint32_t i = 0;
    uint8_t temp = 0;
    uint8_t sval = 0;

    lcd_show_string(x, y, 239, y + 16, 16, "Ex Memory Test:   0KB", BLUE);

    for (i = 0; i < 1024 * 1024; i += 4096)
    {
        sram_write(&temp, i, 1);
        temp++;
    }

    for (i = 0; i < 1024 * 1024; i += 4096)
    {
        sram_read(&temp, i, 1);

        if (i == 0)
        {
            sval = temp;
        }
        else if (temp <= sval)
        {
            break;
        }

        lcd_show_xnum(x + 15 * 8, y, (uint16_t)(temp - sval + 1) * 4, 4, 16, 0, BLUE);
    }
}

/* ===== main函数: 根据V01_BASIC_ONLY宏切换版本 ===== */
#ifdef V01_BASIC_ONLY
/* v0.1 基础版本: 外置SRAM + LCD硬件资源展示 */
int main(void)
{
    uint8_t key;
    uint8_t i = 0;
    uint32_t ts = 0;

    HAL_Init();
    sys_stm32_clock_init(336, 25, 2, 7);
    delay_init(168);
    usart_init(115200);
    led_init();
    lcd_init();
    key_init();
    sram_init();

    lcd_show_string(30,  50, 200, 16, 16, "STM32", RED);
    lcd_show_string(30,  70, 200, 16, 16, "SRAM TEST", RED);
    lcd_show_string(30,  90, 200, 16, 16, "ATOM@MZXQ", RED);
    lcd_show_string(30, 110, 200, 16, 16, "KEY0:Test Sram", RED);
    lcd_show_string(30, 130, 200, 16, 16, "KEYUP:TEST Data", RED);

    for (ts = 0; ts < 250000; ts++)
    {
        g_test_buffer[ts] = ts;
    }

    while (1)
    {
        key = key_scan();

        if (key == KEY0_PRES)
        {
            fsmc_sram_test(30, 150);
        }
        else if (key == WKUP_PRES)
        {
            for (ts = 0; ts < 250000; ts++)
            {
                lcd_show_xnum(30, 170, g_test_buffer[ts], 6, 16, 0, BLUE);
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
    /* ★★★ 极早喂狗：防 Option Bytes IWDG_HW=1 导致 128ms 复位循环 ★★★ */
    if (RCC->CSR & RCC_CSR_IWDGRSTF) {
        RCC->CSR |= RCC_CSR_RMVF;
    }
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }
    IWDG->KR = 0x5555;
    IWDG->PR = 4;
    while (IWDG->SR & IWDG_SR_PVU) { }
    IWDG->RLR = 2000;
    while (IWDG->SR & IWDG_SR_RVU) { }
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
    while (IWDG->SR) { }

    /* ★★★ 最早串口：确认代码从 Reset 一路跑到这 ★★★ */
    early_uart_init();
    printf("\r\n\r\n[DBG] ========== RESET OCCURRED ==========\r\n");
    printf("[DBG] RCC_CSR=0x%08lX (RSTF:%s%s%s%s)\r\n", RCC->CSR,
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

    /* ★★★ 初始化 delay 子系统（需要在168MHz时钟配置后） ★★★ */
    delay_init(168);

    /* ★★★ 再初始化外部 SRAM ★★★ */
#if defined(DATA_IN_ExtSRAM)
    BOOT_TRACE("Init: SRAM (FSMC Bank3)...");
    sram_init();
    BOOT_TRACE("SRAM: OK");
#endif

    /* ★★★ 最后开 MPU ★★★ */
    MPU_Config();

    MX_GPIO_Init();

    printf("\r\n\r\n");
    printf("[BOOT     0 ms] ========== 巨鳌动力外骨骼控制系统启动 ==========\r\n");
    printf("[BOOT     0 ms] Early UART: OK (HSI 16MHz, 115200)\r\n");

    MX_USART1_UART_Init();
    printf("[BOOT %5lu ms] System Clock: %ld MHz (%s)\r\n", HAL_GetTick(),
           SystemCoreClock / 1000000,
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

    BOOT_TRACE("Init: CAN1/CAN2 (1Mbps)...");
    can_motor_init();
    BOOT_TRACE("CAN: OK");

    BOOT_TRACE("Init: I2C / EEPROM (400kHz)...");
    eeprom_init();
    BOOT_TRACE("I2C: OK");

    BOOT_TRACE("Init: Interpolation Tables...");
    interpolation_init();
    BOOT_TRACE("Interpolation: OK");

    BOOT_TRACE("Init: Safety...");
    safety_init();
    BOOT_TRACE("Safety: OK");

    BOOT_TRACE("Init: TIM6 Control ISR (1ms)...");
    control_isr_init();
    BOOT_TRACE("TIM6: OK");

    BOOT_TRACE("Init: UDP Protocol FIFO...");
    udp_protocol_init();
    BOOT_TRACE("UDP FIFO: OK");

    BOOT_TRACE("Init: Ethernet PHY + LwIP Stack (may take 2-3s)...");
    {
        uint32_t eth_start = HAL_GetTick();
        uint8_t eth_ret = lwip_comm_init();
        uint32_t eth_elapsed = HAL_GetTick() - eth_start;
        if (eth_ret != 0) {
            BOOT_TRACE("Ethernet/LwIP: FAILED (code=%d, %lums), continuing without network", eth_ret, eth_elapsed);
        } else {
            BOOT_TRACE("Ethernet/LwIP: OK (%lums)", eth_elapsed);
            BOOT_TRACE("Init: UDP Port 5001...");
            udp_net_init();
            BOOT_TRACE("UDP: OK");
        }
    }

    BOOT_TRACE("========== 巨鳌动力系统初始化完成 ==========");

    BOOT_TRACE("Runtime Self-Test...");
    {
#if defined(DATA_IN_ExtSRAM)
        uint8_t sram_ret = sram_test();
        BOOT_TRACE("SRAM Test: %s (0x68000000, 1MB)", sram_ret == 0 ? "PASS" : "FAIL");
#endif
    }
    BOOT_TRACE("ETH Link:  %s", netif_is_link_up(&g_lwip_netif) ? "UP" : "DOWN");
    BOOT_TRACE("IP Addr :  %s", ip4addr_ntoa(&g_lwip_netif.ip_addr));
    BOOT_TRACE("CAN1 Mode: %s", (hcan1.Init.Mode == CAN_MODE_NORMAL) ? "NORMAL" : "SILENT/LOOPBACK");
    BOOT_TRACE("CAN2 Mode: %s", (hcan2.Init.Mode == CAN_MODE_NORMAL) ? "NORMAL" : "SILENT/LOOPBACK");

    BOOT_TRACE("Starting IWDG (4s timeout, will reset if tasks stall)...");
    MX_IWDG_Init();
    BOOT_TRACE("IWDG: ENABLED (4s timeout)");
    BOOT_TRACE("========== Self-Test Complete ==========\r\n");

    /* Main loop - 非阻塞，集成所有任务 */
    uint32_t last_tick = HAL_GetTick();
    uint32_t last_lcd_tick = 0;
    uint32_t last_status_tick = 0;
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);

    while (1)
    {
        /* ---- 业务任务 ---- */
        sys_info_work_start();

        if (!comm_is_heartbeat_ok()) {
            g_safety_state.fault_code |= FAULT_COMM_LOST;
        } else {
            g_safety_state.fault_code &= ~FAULT_COMM_LOST;
        }

        network_task_run();
        control_task_run();
        eeprom_task_run();

        /* ---- LCD 状态显示 (300ms 刷新一次) ---- */
        uint32_t now = HAL_GetTick();
        if (now - last_lcd_tick >= 300)
        {
            last_lcd_tick = now;
            sys_info_update();
            lcd_status_task();
        }

        /* ---- 按键扫描 ---- */
        key_scan();

        sys_info_work_end();

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
            printf("[%lu] CPU:%u%% Heartbeat:%s CmdVer:%lu Faults:0x%04X\r\n",
                   now / 1000,
                   sys_info_get()->cpu_usage,
                   hb ? "OK" : "LOST",
                   cmd_v,
                   (unsigned)g_safety_state.fault_code);
        }
    }
}
#endif /* V01_BASIC_ONLY */
