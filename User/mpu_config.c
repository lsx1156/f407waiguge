#include "main.h"

/* Core/Src/mpu_config.c - 完整替换 MPU_Config() */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* 关闭 MPU 配置期间的保护 */
  HAL_MPU_Disable();

  /* ------------------------------------------------------------
   * Region 0: Flash 1MB (0x0800_0000) - 代码执行区
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x08000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;      // C=1
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE; // B=0 -> WT
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 1: Internal SRAM 256KB (0x2000_0000 ~ 0x2003_FFFF)
   * STM32F407 实际 128KB SRAM (0x20000000~0x2001FFFF)
   * 用 256KB Region + SRD=0x80 禁顶部 32KB, 覆盖 224KB
   * 确保栈顶和所有访问都在 Normal WT 区域内, 不落默认 Strongly-Ordered
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x20000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.SubRegionDisable = 0x80;  /* 禁顶部 32KB (0x2003C000~0x2003FFFF) */
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region 2 已合并到 Region 1 (256KB+SRD=0x80) */

  /* ------------------------------------------------------------
   * Region 3: Peripherals (0x4000_0000 ~ 0x5FFF_FFFF) - Device
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress      = 0x40000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 4: QSPI / FMC Bank1 (0x6000_0000 ~ 0x6FFF_FFFF) - 强序
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress      = 0x60000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 5: External SRAM (FSMC Bank3) 0x6800_0000 ~ 0x680F_FFFF (1MB)
   * v1.5: Non-Cacheable + Bufferable (C=0, B=1)
   *   必须 Bufferable (B=1): DMA 才能正常读写该区域
   *   设为 Non-Cacheable (C=0): 省去手动 Cache Clean/Invalidate, DMA 一致性最简单
   *   这是 ETH DMA 描述符/缓冲区放在外部 SRAM 的必要条件
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER5;
  MPU_InitStruct.BaseAddress      = 0x68000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;  // C=0: 非缓存, DMA 一致
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;      // B=1: DMA 可写
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 6: PPB (Private Peripheral Bus) 0xE000_0000 ~ 0xE00F_FFFF (1MB)
   * System Control Space, NVIC, SCB, MPU, FPU etc.
   * Must be Device, non-executable for correct fault handling
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER6;
  MPU_InitStruct.BaseAddress      = 0xE0000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* 启用 MPU + 默认规则(特权访问) + HFNMIENA/PRIVDEFENA */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
