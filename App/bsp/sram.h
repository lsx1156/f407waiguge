#ifndef __SRAM_H__
#define __SRAM_H__

#include "stm32f4xx_hal.h"
#include "bsp_config.h"

extern SRAM_HandleTypeDef g_sram_handler;

#define SRAM_ADDR_LWIP_HEAP    ((uint32_t)(MEM_EXT_SRAM_START))
#define SRAM_ADDR_PBUF_POOL    ((uint32_t)(MEM_EXT_SRAM_START + MEM_LWIP_HEAP_SIZE))
#define SRAM_ADDR_FIFO         ((uint32_t)(MEM_EXT_SRAM_START + MEM_LWIP_HEAP_SIZE + MEM_PBUF_POOL_SIZE))
#define SRAM_ADDR_LOG          ((uint32_t)(MEM_EXT_SRAM_START + MEM_LWIP_HEAP_SIZE + MEM_PBUF_POOL_SIZE + MEM_FIFO_EXT_SIZE))
#define SRAM_ADDR_TABLES       ((uint32_t)(MEM_EXT_SRAM_START + MEM_LWIP_HEAP_SIZE + MEM_PBUF_POOL_SIZE + MEM_FIFO_EXT_SIZE + MEM_LOG_SIZE))

void sram_init(void);
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_write_16b(uint16_t *pbuf, uint32_t addr, uint32_t datalen);
void sram_read_16b(uint16_t *pbuf, uint32_t addr, uint32_t datalen);
uint8_t sram_test(void);

#endif