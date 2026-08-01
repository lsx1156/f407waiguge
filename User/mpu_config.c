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
   * Region 1: Internal SRAM 192KB (0x2000_0000 ~ 0x2002_FFFF)
   *           + 栈顶 0x2003_0000
   * 必须按 256KB 对齐/大小，SubRegionDisable=0x80 屏蔽顶部 32KB
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x20000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256KB;     // <=== 关键：256KB (0x11)
  MPU_InitStruct.SubRegionDisable = 0x80;                      // <=== 关键：禁第7子区(顶部32KB)
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;            // TEX=0
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;    // RW
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;      // S=1
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;      // C=1
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE; // B=0 -> Normal WT
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 2: Peripherals (0x4000_0000 ~ 0x5FFF_FFFF) - Device
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress      = 0x40000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE; // XN
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;           // S=1
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;       // C=0
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;          // B=1 -> Device
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 3: QSPI / FMC Bank1 (0x6000_0000 ~ 0x6FFF_FFFF) - 强序
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress      = 0x60000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;      // Strongly Ordered
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* ------------------------------------------------------------
   * Region 4: External SRAM (FSMC Bank3) 0x6800_0000 ~ 0x680F_FFFF (1MB)
   * Normal WT, Shareable
   * ----------------------------------------------------------*/
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress      = 0x68000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE; // 数据区不执行
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;           // S=1
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;           // C=1
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;      // B=0 -> WT
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* 启用 MPU + 默认规则(特权访问) + HFNMIENA/PRIVDEFENA */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
