#ifndef __SRAM_H__
#define __SRAM_H__

#include "stm32f4xx_hal.h"
#include "bsp_config.h"

extern SRAM_HandleTypeDef g_sram_handler;

/* SRAM 分区与 bsp_config.h 保持一致:
 * 0x68000000: MEM_TABLE  (128KB)
 * 0x68020000: LWIP_HEAP  ( 64KB)
 * 0x68030000: PBUF_POOL  ( 32KB)
 * 0x68038000: FIFO_EXT   ( 64KB)
 * 0x68048000: LOG_BUF    ( 64KB)
 * 0x68058000: STATIC_BUF ( 16KB)
 * 0x6805C000: FREE       (~656KB)
 */
#define SRAM_ADDR_TABLES       ((uint32_t)MEM_TABLE_ADDR)
#define SRAM_ADDR_LWIP_HEAP    ((uint32_t)MEM_LWIP_HEAP_ADDR)
#define SRAM_ADDR_PBUF_POOL    ((uint32_t)MEM_PBUF_POOL_ADDR)
#define SRAM_ADDR_FIFO         ((uint32_t)MEM_FIFO_EXT_ADDR)
#define SRAM_ADDR_LOG          ((uint32_t)MEM_LOG_ADDR)

void sram_init(void);
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_write_16b(uint16_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read_16b(uint16_t *pbuf, uint32_t addr, uint32_t datalen);
uint8_t sram_test(void);

#endif