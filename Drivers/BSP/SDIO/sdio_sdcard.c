/**
 ****************************************************************************************************
 * @file        sdio_sdcard.c
 * @brief       SD 卡驱动 (SDIO + DMA)
 *
 * ★ 本文件基于厂商例程 实验39 的 sdio_sdcard.c 改写, 关键差异:
 *   厂商版 sd_read_disk/sd_write_disk 使用【关中断 + 轮询】:
 *       __disable_irq(); HAL_SD_WriteBlocks(...); while(state!=OK); __enable_irq();
 *   一次 32KB 写会全程关中断 2~3ms —— 对本工程 1kHz (TIM6, NVIC prio 0) 控制/采集
 *   ISR 是致命的: 会造成周期性丢数据与力矩指令间隔拉长。
 *
 *   本版改为【DMA 传输, 全程不关中断】:
 *       HAL_SD_WriteBlocks_DMA / ReadBlocks_DMA + 等待 TxCplt/RxCplt 标志
 *   - DMA2_Stream6 CH4 = SDIO_TX, DMA2_Stream3 CH4 = SDIO_RX (F407 固定映射)
 *   - 二者 NVIC 优先级 5 (低于 TIM6 的 0) → 绝不抢占控制 ISR
 *   - 缓冲 4 字节不对齐时自动走 4 字节对齐的 bounce 缓冲 (DMA 以 WORD 访问, 地址必须对齐)
 ****************************************************************************************************
 */
#include "string.h"
#include "sdio_sdcard.h"
#include "stm32f4xx_hal.h"

/* ===== 句柄 ===== */
SD_HandleTypeDef        g_sdcard_handler;
HAL_SD_CardInfoTypeDef  g_sd_card_info_handle;

static DMA_HandleTypeDef s_hdma_sdio_tx;      /* DMA2_Stream6, Channel4 (SDIO TX) */
static DMA_HandleTypeDef s_hdma_sdio_rx;      /* DMA2_Stream3, Channel4 (SDIO RX) */

/* ===== 传输状态 ===== */
static volatile uint8_t  s_tx_done;
static volatile uint8_t  s_rx_done;
static volatile uint8_t  s_sd_err;            /* 0=无错, 1=DMA/卡错误 */

/* bounce 缓冲: 供非 4 字节对齐的调用方使用 (放外部 SRAM 空闲区, 见 bsp_config.h 内存图) */
#define SDIO_BOUNCE_ADDR   0x680E0000UL
#define SDIO_BOUNCE_SIZE   (8 * 1024)
#define SDIO_BOUNCE        ((uint8_t *)SDIO_BOUNCE_ADDR)

#define SD_TRANSFER_TIMEOUT_MS   3000u

/* ==================================================================================
 *  HAL 回调 (DMA 完成由 SDIO 中断里回调, 不关中断)
 * ================================================================================== */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    s_tx_done = 1;
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    s_rx_done = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    (void)hsd;
    s_sd_err = 1;
    s_tx_done = 1;      /* 解放等待方 */
    s_rx_done = 1;
}

/* 中断入口 (工程原有 stm32f4xx_it.c 未定义时才需要; 用 weak 防重复定义) */
void SDIO_IRQHandler(void)
{
    HAL_SD_IRQHandler(&g_sdcard_handler);
}

void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(g_sdcard_handler.hdmatx);
}

void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(g_sdcard_handler.hdmarx);
}

/* ==================================================================================
 *  底层 MSP: 引脚 + DMA + NVIC
 *  (引脚配置沿用厂商版, 已核对与本工程无冲突: PC8~PC12 / PD2 全空闲)
 * ================================================================================== */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_SDIO_CLK_ENABLE();
    SD_D0_GPIO_CLK_ENABLE();
    SD_D1_GPIO_CLK_ENABLE();
    SD_D2_GPIO_CLK_ENABLE();
    SD_D3_GPIO_CLK_ENABLE();
    SD_CLK_GPIO_CLK_ENABLE();
    SD_CMD_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = SD_D0_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(SD_D0_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = SD_D1_GPIO_PIN;
    HAL_GPIO_Init(SD_D1_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = SD_D2_GPIO_PIN;
    HAL_GPIO_Init(SD_D2_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = SD_D3_GPIO_PIN;
    HAL_GPIO_Init(SD_D3_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = SD_CLK_GPIO_PIN;
    HAL_GPIO_Init(SD_CLK_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = SD_CMD_GPIO_PIN;
    HAL_GPIO_Init(SD_CMD_GPIO_PORT, &gpio_init_struct);

    /* ---- DMA2 时钟 ---- */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* SDIO_TX: DMA2_Stream6, Channel4, 内存→外设 */
    s_hdma_sdio_tx.Instance                 = DMA2_Stream6;
    s_hdma_sdio_tx.Init.Channel             = DMA_CHANNEL_4;
    s_hdma_sdio_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdma_sdio_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_sdio_tx.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_sdio_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_hdma_sdio_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    s_hdma_sdio_tx.Init.Mode                = DMA_PFCTRL;
    s_hdma_sdio_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_sdio_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    s_hdma_sdio_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    s_hdma_sdio_tx.Init.MemBurst            = DMA_MBURST_INC4;
    s_hdma_sdio_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init(&s_hdma_sdio_tx);
    __HAL_LINKDMA(hsd, hdmatx, s_hdma_sdio_tx);

    /* SDIO_RX: DMA2_Stream3, Channel4, 外设→内存 */
    s_hdma_sdio_rx.Instance                 = DMA2_Stream3;
    s_hdma_sdio_rx.Init.Channel             = DMA_CHANNEL_4;
    s_hdma_sdio_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_sdio_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_sdio_rx.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_sdio_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_hdma_sdio_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    s_hdma_sdio_rx.Init.Mode                = DMA_PFCTRL;
    s_hdma_sdio_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_sdio_rx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    s_hdma_sdio_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    s_hdma_sdio_rx.Init.MemBurst            = DMA_MBURST_INC4;
    s_hdma_sdio_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init(&s_hdma_sdio_rx);
    __HAL_LINKDMA(hsd, hdmarx, s_hdma_sdio_rx);

    /* ---- NVIC: 优先级 5, 低于 TIM6(0), 不抢占控制/采集 ISR ---- */
    HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/* ==================================================================================
 *  初始化
 * ================================================================================== */
uint8_t sd_init(void)
{
    /* 复位外设, 保证重复调用(如重试)时状态干净 */
    HAL_SD_DeInit(&g_sdcard_handler);
    memset(&g_sdcard_handler, 0, sizeof(g_sdcard_handler));

    g_sdcard_handler.Instance = SDIO;
    g_sdcard_handler.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    g_sdcard_handler.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    g_sdcard_handler.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    g_sdcard_handler.Init.BusWide             = SDIO_BUS_WIDE_1B;
    g_sdcard_handler.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    g_sdcard_handler.Init.ClockDiv            = SDIO_TRANSF_CLK_DIV;

    if (HAL_SD_Init(&g_sdcard_handler) != HAL_OK) {
        return 1;
    }
    if (HAL_SD_GetCardInfo(&g_sdcard_handler, &g_sd_card_info_handle) != HAL_OK) {
        return 2;
    }
    if (HAL_SD_ConfigWideBusOperation(&g_sdcard_handler, SDIO_BUS_WIDE_4B) != HAL_OK) {
        return 3;
    }
    return 0;
}

uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo)
{
    return (uint8_t)HAL_SD_GetCardInfo(&g_sdcard_handler, cardinfo);
}

uint8_t get_sd_card_state(void)
{
    return (HAL_SD_GetCardState(&g_sdcard_handler) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK
                                                                           : SD_TRANSFER_BUSY;
}

/* ==================================================================================
 *  传输 (DMA, 不关中断)
 * ================================================================================== */

/* 检查指针与长度是否满足 WORD(4B) DMA 对齐要求 */
static uint8_t is_word_aligned(const void *p, uint32_t bytes)
{
    return (((uint32_t)p & 0x3u) == 0u) && ((bytes & 0x3u) == 0u);
}

/* 等卡内部操作完成 */
static uint8_t wait_card_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&g_sdcard_handler) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return 1;
        }
    }
    return 0;
}

static uint8_t sd_write_blocks_dma(const uint8_t *buf, uint32_t saddr, uint32_t cnt)
{
    s_tx_done = 0;
    s_sd_err  = 0;
    if (HAL_SD_WriteBlocks_DMA(&g_sdcard_handler, (uint8_t *)buf, saddr, cnt) != HAL_OK) {
        return 1;
    }
    uint32_t t0 = HAL_GetTick();
    while (!s_tx_done) {
        if ((HAL_GetTick() - t0) > SD_TRANSFER_TIMEOUT_MS) {
            return 2;
        }
    }
    if (s_sd_err) {
        return 3;
    }
    return wait_card_ready(SD_TRANSFER_TIMEOUT_MS) ? 4 : 0;
}

static uint8_t sd_read_blocks_dma(uint8_t *buf, uint32_t saddr, uint32_t cnt)
{
    s_rx_done = 0;
    s_sd_err  = 0;
    if (HAL_SD_ReadBlocks_DMA(&g_sdcard_handler, buf, saddr, cnt) != HAL_OK) {
        return 1;
    }
    uint32_t t0 = HAL_GetTick();
    while (!s_rx_done) {
        if ((HAL_GetTick() - t0) > SD_TRANSFER_TIMEOUT_MS) {
            return 2;
        }
    }
    if (s_sd_err) {
        return 3;
    }
    return wait_card_ready(SD_TRANSFER_TIMEOUT_MS) ? 4 : 0;
}

/* ---- 对外: 写 SD (FatFS disk_write 调用) ----
 * 返回 0 成功; 非 0 失败
 * 注: 不关中断; 大块写期间由 DMA 搬运, CPU 与 1kHz ISR 不受影响 */
uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint32_t bytes = cnt * 512u;

    if (pbuf == NULL || cnt == 0) {
        return 1;
    }
    if (is_word_aligned(pbuf, bytes)) {
        return sd_write_blocks_dma(pbuf, saddr, cnt);
    }

    /* 非 4 字节对齐: 分块经 bounce 缓冲搬运 (本工程 FatFS 内部 win[] 可能不对齐) */
    uint32_t done_sect = 0;
    while (done_sect < cnt) {
        uint32_t chunk_sect = cnt - done_sect;
        uint32_t max_sect   = SDIO_BOUNCE_SIZE / 512u;
        if (chunk_sect > max_sect) {
            chunk_sect = max_sect;
        }
        memcpy(SDIO_BOUNCE, pbuf + done_sect * 512u, chunk_sect * 512u);
        uint8_t r = sd_write_blocks_dma(SDIO_BOUNCE, saddr + done_sect, chunk_sect);
        if (r) {
            return r;
        }
        done_sect += chunk_sect;
    }
    return 0;
}

/* ---- 对外: 读 SD ---- */
uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint32_t bytes = cnt * 512u;

    if (pbuf == NULL || cnt == 0) {
        return 1;
    }
    if (is_word_aligned(pbuf, bytes)) {
        return sd_read_blocks_dma(pbuf, saddr, cnt);
    }

    uint32_t done_sect = 0;
    while (done_sect < cnt) {
        uint32_t chunk_sect = cnt - done_sect;
        uint32_t max_sect   = SDIO_BOUNCE_SIZE / 512u;
        if (chunk_sect > max_sect) {
            chunk_sect = max_sect;
        }
        uint8_t r = sd_read_blocks_dma(SDIO_BOUNCE, saddr + done_sect, chunk_sect);
        if (r) {
            return r;
        }
        memcpy(pbuf + done_sect * 512u, SDIO_BOUNCE, chunk_sect * 512u);
        done_sect += chunk_sect;
    }
    return 0;
}
