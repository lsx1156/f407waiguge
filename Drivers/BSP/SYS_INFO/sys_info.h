#ifndef __SYS_INFO_H
#define __SYS_INFO_H

#include "./SYSTEM/sys/sys.h"

#define SRAM_TOTAL_SIZE   (1024 * 1024)
#define FLASH_TOTAL_SIZE  (1024 * 1024)
#define RAM_TOTAL_SIZE     (128 * 1024)

typedef struct
{
    uint32_t sysclk_mhz;
    uint32_t hclk_mhz;
    uint32_t pclk1_mhz;
    uint32_t pclk2_mhz;
} sys_clk_info_t;

typedef struct
{
    uint32_t total;
    uint32_t used;
    uint32_t free;
} mem_info_t;

typedef struct
{
    sys_clk_info_t clk;
    mem_info_t flash;
    mem_info_t ram;
    mem_info_t sram;
    uint8_t cpu_usage;
    uint32_t uptime_ms;
} sys_info_t;

void sys_info_init(void);
void sys_info_update(void);
void sys_info_update_sram(void);
sys_info_t *sys_info_get(void);
void sys_info_idle_hook(void);
void sys_info_work_start(void);
void sys_info_work_end(void);

#endif
