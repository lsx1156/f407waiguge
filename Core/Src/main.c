#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
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

I2C_HandleTypeDef g_i2c1_handle;

void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);
void MX_TIM6_Init(void);
void MX_ETH_Init(void);

void HardFault_Handler_C(uint32_t *stack)
{
    (void)stack;
    while(1)
    {
        for (int i = 0; i < 2000000; i++) { __NOP(); }
        HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
    }
}

void HardFault_Handler(void)
{
    __asm volatile (
        "TST LR, #4          \n"
        "ITE EQ              \n"
        "MRSEQ R0, MSP       \n"
        "MRSNE R0, PSP       \n"
        "B HardFault_Handler_C \n"
    );
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = LED0_PIN | LED1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED0_PORT, &GPIO_InitStruct);
}

void MX_USART1_UART_Init(void)
{
    usart_init(115200);
}

void MX_CAN1_Init(void)
{
    /* can_motor_init() initializes both CAN1 and CAN2 in SILENT mode.
     * Switch to CAN_MODE_NORMAL when motors are connected. */
    can_motor_init();
}

void MX_CAN2_Init(void) {}
void MX_ADC1_Init(void) {}

void MX_I2C1_Init(void)
{
    eeprom_init();
}

void MX_TIM6_Init(void)
{
    control_isr_init();
}

void MX_ETH_Init(void)
{
    /* Ethernet init deferred until hardware verified */
}

static void cpu_delay(volatile uint32_t count)
{
    while (count--) { __NOP(); }
}

int main(void)
{
    uint8_t ret;

    HAL_Init();

    /* Clock: 25MHz HSE -> PLL -> 168MHz */
    ret = sys_stm32_clock_init(336, 25, 2, 7);
    if (ret != 0)
    {
        /* HSE failed. Fallback: configure GPIO and blink error */
        MX_GPIO_Init();
        HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);
        while (1)
        {
            /* 2 fast blinks = HSE error */
            HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
            cpu_delay(5000000);
        }
    }

    delay_init(168);
    MX_GPIO_Init();

    printf("========== F407 Exoskeleton System Starting ==========\r\n");
    printf("System Clock: 168MHz (HSE 25MHz)\r\n");

    printf("Init: UART...\r\n");
    MX_USART1_UART_Init();
    printf("UART: OK\r\n");

    printf("Init: SRAM (FSMC)...\r\n");
    sram_init();
    printf("SRAM: Init done\r\n");

    printf("Init: CAN1/CAN2 (SILENT mode)...\r\n");
    MX_CAN1_Init();
    printf("CAN: OK (SILENT mode, switch to NORMAL when motors connected)\r\n");

    printf("Init: I2C / EEPROM...\r\n");
    MX_I2C1_Init();
    printf("I2C: OK\r\n");

    printf("Init: Interpolation Tables...\r\n");
    interpolation_init();
    printf("Interpolation: OK\r\n");

    printf("Init: Safety...\r\n");
    safety_init();
    printf("Safety: OK\r\n");

    printf("Init: TIM6 Control ISR (1ms)...\r\n");
    MX_TIM6_Init();
    printf("TIM6: OK\r\n");

    printf("========== System Init Complete ==========\r\n");

    /* Main loop */
    uint32_t led_toggle = 0;
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);

    while (1)
    {
        network_task_run();
        control_task_run();
        eeprom_task_run();

        led_toggle++;
        if (led_toggle >= 200)
        {
            led_toggle = 0;
            HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
        }

        delay_ms(1);
    }
}