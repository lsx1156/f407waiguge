#include "./BSP/SYS_INFO/sys_info.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include <stdio.h>

extern uint32_t Image$$ER_IROM1$$Base;
extern uint32_t Image$$ER_IROM1$$Limit;
extern uint32_t Image$$RW_IRAM1$$Base;
extern uint32_t Image$$RW_IRAM1$$RW$$Length;
extern uint32_t Image$$RW_IRAM1$$ZI$$Length;
extern uint32_t Load$$ER_IROM1$$RW$$Base;
extern uint32_t Load$$ER_IROM1$$RW$$Length;

#define RAM_END_ADDR             0x20020000
#define CPU_MEASURE_WINDOW_MS    1000

#define INITIAL_MSP              (*((uint32_t *)0x08000000))

static sys_info_t g_sys_info;
static volatile uint32_t g_work_cycles = 0;
static volatile uint32_t g_last_measure_tick = 0;
static volatile uint32_t g_work_start_cyccnt = 0;
static uint8_t g_cpu_usage_cached = 0;
static uint8_t g_dwt_inited = 0;
static uint8_t g_in_work = 0;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_dwt_inited = 1;
}

static uint32_t dwt_get_cyccnt(void)
{
    if (!g_dwt_inited) dwt_init();
    return DWT->CYCCNT;
}

void sys_info_idle_hook(void)
{
}

void sys_info_work_start(void)
{
    if (!g_in_work)
    {
        g_work_start_cyccnt = dwt_get_cyccnt();
        g_in_work = 1;
    }
}

void sys_info_work_end(void)
{
    if (g_in_work)
    {
        uint32_t now = dwt_get_cyccnt();
        g_work_cycles += (now - g_work_start_cyccnt);
        g_in_work = 0;
    }
}

uint32_t sys_info_get_stack_usage(void)
{
    uint32_t initial_msp = INITIAL_MSP;
    uint32_t current_msp = __get_MSP();
    if (current_msp < initial_msp)
    {
        return initial_msp - current_msp;
    }
    return 0;
}

void sys_info_init(void)
{
    uint32_t i;
    uint32_t *sram_base = (uint32_t *)0x68000000;
    uint32_t rw_len = (uint32_t)&Image$$RW_IRAM1$$RW$$Length;
    uint32_t zi_len = (uint32_t)&Image$$RW_IRAM1$$ZI$$Length;
    uint32_t msp_val = __get_MSP();

    g_sys_info.flash.total = FLASH_TOTAL_SIZE;
    g_sys_info.ram.total = RAM_TOTAL_SIZE;
    g_sys_info.sram.total = SRAM_TOTAL_SIZE;

    printf("[SYS_INFO] RW=%u ZI=%u static=%u InitMSP=0x%08X MSP=0x%08X stack=%u total_ram=%u\r\n",
           (unsigned)rw_len, (unsigned)zi_len, (unsigned)(rw_len + zi_len),
           (unsigned)INITIAL_MSP, (unsigned)msp_val,
           (unsigned)(INITIAL_MSP - msp_val),
           (unsigned)(rw_len + zi_len + INITIAL_MSP - msp_val));

    for (i = 0; i < SRAM_TOTAL_SIZE / 4; i++)
    {
        sram_base[i] = 0x00000000;
    }

    dwt_init();
    g_last_measure_tick = HAL_GetTick();
    sys_info_update();
}

void sys_info_update_clk(void)
{
    g_sys_info.clk.sysclk_mhz = HAL_RCC_GetSysClockFreq() / 1000000;
    g_sys_info.clk.hclk_mhz = HAL_RCC_GetHCLKFreq() / 1000000;
    g_sys_info.clk.pclk1_mhz = HAL_RCC_GetPCLK1Freq() / 1000000;
    g_sys_info.clk.pclk2_mhz = HAL_RCC_GetPCLK2Freq() / 1000000;
}

void sys_info_update_flash(void)
{
    uint32_t ro_size = (uint32_t)&Image$$ER_IROM1$$Limit - (uint32_t)&Image$$ER_IROM1$$Base;
    uint32_t rw_init_size = (uint32_t)&Load$$ER_IROM1$$RW$$Length;
    uint32_t flash_used = ro_size + rw_init_size;
    g_sys_info.flash.used = flash_used;
    g_sys_info.flash.free = g_sys_info.flash.total - flash_used;
}

void sys_info_update_ram(void)
{
    uint32_t static_ram = (uint32_t)&Image$$RW_IRAM1$$RW$$Length + (uint32_t)&Image$$RW_IRAM1$$ZI$$Length;
    uint32_t ram_used = static_ram + sys_info_get_stack_usage();

    if (ram_used > g_sys_info.ram.total)
    {
        ram_used = g_sys_info.ram.total;
    }

    g_sys_info.ram.used = ram_used;
    g_sys_info.ram.free = g_sys_info.ram.total - ram_used;
}

void sys_info_update_sram(void)
{
    uint32_t i;
    uint8_t val;
    uint32_t used_bytes = 0;
    uint8_t *sram_base = (uint8_t *)0x68000000;

    for (i = 0; i < SRAM_TOTAL_SIZE; i += 256)
    {
        val = sram_base[i];
        if (val != 0xFF && val != 0x00)
        {
            used_bytes += 256;
        }
    }

    g_sys_info.sram.used = used_bytes;
    g_sys_info.sram.free = g_sys_info.sram.total - used_bytes;
}

void sys_info_update_cpu(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - g_last_measure_tick;

    if (elapsed >= CPU_MEASURE_WINDOW_MS)
    {
        uint32_t sysclk_hz = HAL_RCC_GetSysClockFreq();
        uint64_t total_expected = (uint64_t)sysclk_hz * elapsed / 1000;

        if (total_expected > 0)
        {
            uint32_t work = g_work_cycles;
            if (work > total_expected) work = (uint32_t)total_expected;
            g_cpu_usage_cached = (uint8_t)((uint64_t)work * 100 / total_expected);
        }

        g_work_cycles = 0;
        g_last_measure_tick = now;
    }

    g_sys_info.cpu_usage = g_cpu_usage_cached;
}

void sys_info_update(void)
{
    g_sys_info.uptime_ms = HAL_GetTick();
    sys_info_update_clk();
    sys_info_update_flash();
    sys_info_update_ram();
    sys_info_update_sram();
    sys_info_update_cpu();
}

sys_info_t *sys_info_get(void)
{
    return &g_sys_info;
}
