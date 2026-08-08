#include "./BSP/SYS_INFO/sys_info.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "bsp_config.h"
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

    /* 注意: 不要全量清零 SRAM! 会擦掉插值表/MEM_TABLE等已初始化数据
     * 各模块已通过 section(.bss.EXT_RAM) 自动清零, 或自行初始化 */

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
    /* v1.6.2: SRAM 未就绪时跳过扫描, 防止 FSMC 未初始化导致 BusFault */
    extern volatile uint8_t g_sram_ready;
    if (!g_sram_ready) return;

    /* v1.5: 真实分区统计 + 剩余区域分片扫描
     * 固定分区 (已知布局, 直接累加):
     *   MEM_TABLE     = 128KB @ 0x68000000
     *   LWIP_HEAP     =  64KB @ 0x68020000
     *   PBUF_POOL     =  32KB @ 0x68030000
     *   FIFO_EXT      =  64KB @ 0x68038000
     *   LOG_BUF       =  64KB @ 0x68048000
     *   STATIC_BUF    =  16KB @ 0x68058000
     *   固定合计      = 368KB
     * 剩余区域 (0x6805_C000 ~ 0x680F_FFFF, ~656KB):
     *   分片扫描, 每次 1/10, 统计非 0/非 FF 的用量
     */
    #define SRAM_SCAN_SLICES    10
    #define SRAM_FIXED_USED     (MEM_TABLE_SIZE + MEM_LWIP_HEAP_SIZE + MEM_PBUF_POOL_SIZE + MEM_FIFO_EXT_SIZE + MEM_LOG_SIZE + MEM_STATIC_BUF_SIZE)
    #define SRAM_SCAN_START     (MEM_STATIC_BUF_ADDR + MEM_STATIC_BUF_SIZE)
    #define SRAM_SCAN_END       (SRAM_BASE_ADDR + SRAM_TOTAL_SIZE)
    #define SRAM_SCAN_TOTAL     (SRAM_SCAN_END - SRAM_SCAN_START)

    static uint32_t s_slice_idx = 0;
    static uint32_t s_used_accum = 0;

    /* 防御: 如果 s_slice_idx 被栈溢出/内存损坏覆写, 重置 */
    if (s_slice_idx >= SRAM_SCAN_SLICES) {
        s_slice_idx = 0;
        s_used_accum = 0;
    }

    uint32_t slice_size = SRAM_SCAN_TOTAL / SRAM_SCAN_SLICES;
    uint32_t start = SRAM_SCAN_START + s_slice_idx * slice_size;

    /* i 是绝对地址 (如 0x6805C000), 直接解引用, 不要再加 SRAM_BASE_ADDR */
    for (uint32_t i = start; i < start + slice_size; i += 256)
    {
        uint8_t val = *(volatile uint8_t *)i;
        if (val != 0xFF && val != 0x00)
        {
            s_used_accum += 256;
        }
    }

    s_slice_idx++;
    if (s_slice_idx >= SRAM_SCAN_SLICES) {
        /* 一轮扫描结束, 更新结果: 固定分区 + 扫描到的动态用量 */
        g_sys_info.sram.used = SRAM_FIXED_USED + s_used_accum;
        g_sys_info.sram.free = g_sys_info.sram.total - g_sys_info.sram.used;
        s_used_accum = 0;
        s_slice_idx = 0;

        static uint32_t last_print = 0;
        if (HAL_GetTick() - last_print >= 5000) {
            last_print = HAL_GetTick();
            /* v1.6.6: 暂时关闭 SRAM 调试打印, 减少串口噪音 */
#if 0
            printf("[DBG] SRAM: used=%uKB fixed=%uKB scan=%uKB free=%uKB (table=0x%02X heap=0x%02X)\r\n",
                   (unsigned)(g_sys_info.sram.used / 1024),
                   (unsigned)(SRAM_FIXED_USED / 1024),
                   (unsigned)(s_used_accum / 1024),
                   (unsigned)(g_sys_info.sram.free / 1024),
                   (unsigned)((uint8_t *)MEM_TABLE_ADDR)[0],
                   (unsigned)((uint8_t *)MEM_LWIP_HEAP_ADDR)[0]);
#endif
        }
    }
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
    static uint32_t last_sram_update = 0;

    g_sys_info.uptime_ms = HAL_GetTick();
    sys_info_update_clk();
    sys_info_update_flash();
    sys_info_update_ram();

    /* SRAM 分片扫描: 每次 1/10 (<0.5ms), 每 500ms 调一次, 5s 扫完 1MB */
    if (g_sys_info.uptime_ms - last_sram_update >= 500) {
        last_sram_update = g_sys_info.uptime_ms;
        sys_info_update_sram();
    }

    sys_info_update_cpu();
}

sys_info_t *sys_info_get(void)
{
    return &g_sys_info;
}
