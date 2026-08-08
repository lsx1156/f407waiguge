/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ethernet.h"
#include "control_isr.h"
#include "can_motor.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  if (__HAL_RCC_GET_IT(RCC_IT_CSS)) {
      printf("\r\n!!! NMI: Clock Security System (CSS) triggered - HSE failed !!!\r\n");
      __HAL_RCC_CLEAR_IT(RCC_IT_CSS);
  }
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
      HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
      for (volatile uint32_t i = 0; i < 1000000; i++) { __NOP(); }
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile(
      "TST   LR, #4            \n"
      "ITE   EQ                \n"
      "MRSEQ R0, MSP           \n"
      "MRSNE R0, PSP           \n"
      "B     HardFault_Dump    \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
}

#include <stdio.h>
#include <stdint.h>

void HardFault_Dump(uint32_t *stack)
{
    /* Disable interrupts, dump register state */
    __disable_irq();

    uint32_t r0  = stack[0];
    uint32_t r1  = stack[1];
    uint32_t r2  = stack[2];
    uint32_t r3  = stack[3];
    uint32_t r12 = stack[4];
    uint32_t lr  = stack[5];
    uint32_t pc  = stack[6];
    uint32_t psr = stack[7];

    uint32_t cfsr = (*((volatile uint32_t *)0xE000ED28));
    uint32_t hfsr = (*((volatile uint32_t *)0xE000ED2C));
    uint32_t dfsr = (*((volatile uint32_t *)0xE000ED30));
    uint32_t afsr = (*((volatile uint32_t *)0xE000ED3C));
    uint32_t mmfar = (*((volatile uint32_t *)0xE000ED34));
    uint32_t bfar = (*((volatile uint32_t *)0xE000ED38));

    printf("\r\n========== HARDFAULT ==========\r\n");
    printf("R0 : 0x%08X  R1 : 0x%08X\r\n", (unsigned)r0, (unsigned)r1);
    printf("R2 : 0x%08X  R3 : 0x%08X\r\n", (unsigned)r2, (unsigned)r3);
    printf("R12: 0x%08X  LR : 0x%08X\r\n", (unsigned)r12, (unsigned)lr);
    printf("PC : 0x%08X  PSR: 0x%08X\r\n", (unsigned)pc, (unsigned)psr);
    printf("CFSR: 0x%08X  HFSR: 0x%08X\r\n", (unsigned)cfsr, (unsigned)hfsr);
    printf("MMFAR: 0x%08X  BFAR: 0x%08X\r\n", (unsigned)mmfar, (unsigned)bfar);
    printf("DFSR:  0x%08X  AFSR: 0x%08X\r\n", (unsigned)dfsr, (unsigned)afsr);
    printf("==============================\r\n");

    while (1)
    {
        /* LED fast blink on HardFault */
        for (volatile uint32_t i = 0; i < 5000000; i++) { __NOP(); }
        HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  __disable_irq();
  uint32_t cfsr = (*((volatile uint32_t *)0xE000ED28));
  uint32_t mmfar = (*((volatile uint32_t *)0xE000ED34));
  printf("\r\n!!! MemManage Fault !!!\r\n");
  printf("CFSR:  0x%08X  (MMFSR=0x%02X)\r\n", (unsigned)cfsr, (unsigned)(cfsr & 0xFF));
  printf("MMFAR: 0x%08X\r\n", (unsigned)mmfar);
  if (cfsr & (1 << 7))  printf("  MMARVALID: address in MMFAR is valid\r\n");
  if (cfsr & (1 << 1))  printf("  DACCVIOL:  data access violation (MPU)\r\n");
  if (cfsr & (1 << 0))  printf("  IACCVIOL:  instruction access violation (MPU)\r\n");
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
      HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
      for (volatile uint32_t i = 0; i < 500000; i++) { __NOP(); }
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  __disable_irq();
  uint32_t cfsr = (*((volatile uint32_t *)0xE000ED28));
  uint32_t bfar = (*((volatile uint32_t *)0xE000ED38));
  printf("\r\n!!! Bus Fault !!!\r\n");
  printf("CFSR:  0x%08X  (BFSR=0x%02X)\r\n", (unsigned)cfsr, (unsigned)((cfsr >> 8) & 0xFF));
  printf("BFAR:  0x%08X\r\n", (unsigned)bfar);
  if (cfsr & (1 << 15)) printf("  BFARVALID: address in BFAR is valid\r\n");
  if (cfsr & (1 << 11)) printf("  LSPERR:    lazy state preservation error\r\n");
  if (cfsr & (1 << 10)) printf("  STKERR:    stacking error (exception entry)\r\n");
  if (cfsr & (1 << 9))  printf("  UNSTKERR:  unstacking error (exception return)\r\n");
  if (cfsr & (1 << 8))  printf("  IMPRECISERR: imprecise data bus error\r\n");
  if (cfsr & (1 << 7))  printf("  PRECISERR:  precise data bus error\r\n");
  if (cfsr & (1 << 6))  printf("  IBUSERR:    instruction bus error\r\n");
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
      HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
      for (volatile uint32_t i = 0; i < 300000; i++) { __NOP(); }
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __disable_irq();
  uint32_t cfsr = (*((volatile uint32_t *)0xE000ED28));
  printf("\r\n!!! Usage Fault !!!\r\n");
  printf("CFSR:  0x%08X  (UFSR=0x%04X)\r\n", (unsigned)cfsr, (unsigned)((cfsr >> 16) & 0xFFFF));
  if (cfsr & (1 << 25)) printf("  DIVBYZERO: divide by zero\r\n");
  if (cfsr & (1 << 24)) printf("  UNALIGNED: unaligned access\r\n");
  if (cfsr & (1 << 19)) printf("  NOCP:      no coprocessor\r\n");
  if (cfsr & (1 << 18)) printf("  INVPC:     invalid PC load\r\n");
  if (cfsr & (1 << 17)) printf("  INVSTATE:  invalid state (EPSR T bit)\r\n");
  if (cfsr & (1 << 16)) printf("  UNDEFINSTR: undefined instruction\r\n");
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
      HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN);
      for (volatile uint32_t i = 0; i < 400000; i++) { __NOP(); }
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt and DAC underrun error.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE END TIM6_DAC_IRQn 0 */
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
volatile uint32_t g_dbg_eth_irq_cnt = 0;
void ETH_IRQHandler(void)
{
    g_dbg_eth_irq_cnt++;
    HAL_ETH_IRQHandler(&heth);
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);
}
/* USER CODE END 1 */