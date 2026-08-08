/**
 ****************************************************************************************************
 * @file        sram.c
 * @author      MZXQ
 * @version     V1.0
 * @date        2021-11-03
 * @brief       �ⲿSRAM ��������
 * @license     Copyright (c) 2020-2032, MZXQ
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:MZXQ F407������
 *
 * �޸�˵��
 * V1.0 20211103
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __SRAM_H
#define __SRAM_H

#include "./SYSTEM/sys/sys.h"
#include "bsp_config.h"   /* SRAM 地址/分区统一在 bsp_config.h 定义 */


/******************************************************************************************/
/* SRAM WR/RD/CS ���� ���� 
 * SRAM_D0~D15 �� ��ַ��,��������̫��,�Ͳ������ﶨ����,ֱ����SRAM_init�����޸�.��������ֲ��ʱ��,
 * ���˸���3��IO��, ���ø�SRAM_init����� ������ �� ��ַ�� ���ڵ�IO��.
 */

#define SRAM_WR_GPIO_PORT               GPIOD
#define SRAM_WR_GPIO_PIN                GPIO_PIN_5
#ifndef SRAM_WR_GPIO_CLK_ENABLE
#define SRAM_WR_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#endif     /* ����IO��ʱ��ʹ�� */

#define SRAM_RD_GPIO_PORT               GPIOD
#define SRAM_RD_GPIO_PIN                GPIO_PIN_4
#ifndef SRAM_RD_GPIO_CLK_ENABLE
#define SRAM_RD_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#endif     /* ����IO��ʱ��ʹ�� */

/* SRAM_CS(��Ҫ����SRAM_FSMC_NEX������ȷ��IO��) ���� ���� */
#define SRAM_CS_GPIO_PORT                GPIOG
#define SRAM_CS_GPIO_PIN                 GPIO_PIN_10
#ifndef SRAM_CS_GPIO_CLK_ENABLE
#define SRAM_CS_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)
#endif    /* ����IO��ʱ��ʹ�� */

/* FSMC 寄存器宏 (SRAM_FSMC_NEX 统一在 bsp_config.h 定义)
 * 注意: 默认通过 FSMC 第3个片选接 SRAM, 可改范围 1~4
 * 修改 SRAM_FSMC_NEX 时, 需同步修改 bsp_config.h 中的 SRAM_CS_GPIO 配置
 */
#define SRAM_FSMC_BCRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2]       /* BCR 寄存器, 根据 SRAM_FSMC_NEX 自动计算 */
#define SRAM_FSMC_BTRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2 + 1]   /* BTR 寄存器 */
#define SRAM_FSMC_BWTRX         FSMC_Bank1E->BWTR[(SRAM_FSMC_NEX - 1) * 2]      /* BWTR 寄存器 */

/******************************************************************************************/

/* SRAM_BASE_ADDR 统一由 bsp_config.h 定义 (0x68000000, FSMC Bank3 NE3)
 * 如需修改地址/片选, 请在 bsp_config.h 中同步修改 SRAM_FSMC_NEX 和相关 GPIO 配置
 */

extern SRAM_HandleTypeDef g_sram_handler;    /* SRAM句柄 */


void sram_init(void);
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen);

uint8_t sram_test_read(uint32_t addr);
void sram_test_write(uint32_t addr, uint8_t data);

#endif
