#include "sram.h"
#include "main.h"
#include "bsp_config.h"

/* SRAM handle now in main.c via CubeMX (hsram1), aliased in main.h */

static uint8_t sram_test_write_read_16b(uint32_t addr, uint16_t data);

void sram_init(void)
{
    FSMC_NORSRAM_TimingTypeDef fsmc_timing;

    /* GPIO init is handled by HAL_SRAM_MspInit → HAL_FSMC_MspInit in msp.c */

    g_sram_handler.Instance = FSMC_NORSRAM_DEVICE;
    g_sram_handler.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;

    g_sram_handler.Init.NSBank = (SRAM_FSMC_NEX == 1) ? FSMC_NORSRAM_BANK1 : \
                                 (SRAM_FSMC_NEX == 2) ? FSMC_NORSRAM_BANK2 : \
                                 (SRAM_FSMC_NEX == 3) ? FSMC_NORSRAM_BANK3 :
                                                        FSMC_NORSRAM_BANK4;
    g_sram_handler.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    g_sram_handler.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    g_sram_handler.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    g_sram_handler.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    g_sram_handler.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    g_sram_handler.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    g_sram_handler.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    g_sram_handler.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    g_sram_handler.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    g_sram_handler.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    g_sram_handler.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;

    /* IS62WV51216BLL-55 timing (55ns, HCLK=168MHz → tHCLK≈5.95ns):
     *   Read: (ADDSET + DATAST + 2) * tHCLK >= tRC=55ns → ADDSET+ DATAST >= 8
     *   Write: (DATAST + 1) * tHCLK >= tPWE=40ns → DATAST >= 6
     *   ADDSET=2, DATAST=8 → Read=71.4ns, Write=53.6ns (safe margin) */
    fsmc_timing.AddressSetupTime = 0x02;
    fsmc_timing.AddressHoldTime = 0x01;
    fsmc_timing.DataSetupTime = 0x08;
    fsmc_timing.BusTurnAroundDuration = 0x01;
    fsmc_timing.AccessMode = FSMC_ACCESS_MODE_A;

    HAL_SRAM_Init(&g_sram_handler, &fsmc_timing, &fsmc_timing);
}

void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *(volatile uint8_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr++;
        pbuf++;
    }
}

void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint8_t *)(SRAM_BASE_ADDR + addr);
        addr++;
    }
}

void sram_write_16b(uint16_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *(volatile uint16_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr += 2;
        pbuf++;
    }
}

void sram_read_16b(uint16_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint16_t *)(SRAM_BASE_ADDR + addr);
        addr += 2;
    }
}

static uint8_t sram_test_write_read_16b(uint32_t addr, uint16_t data)
{
    uint16_t readback;
    *(volatile uint16_t *)(SRAM_BASE_ADDR + addr) = data;
    readback = *(volatile uint16_t *)(SRAM_BASE_ADDR + addr);
    return (readback == data) ? 0 : 1;
}

uint8_t sram_test(void)
{
    uint32_t test_addr;
    uint16_t test_data;
    /* v1.6: 只测试空闲区域 (0x6805C000 之后), 避免覆盖 MEM_TABLE/LWIP_HEAP/PBUF_POOL/FIFO/LOG/STATIC_BUF
     * 之前: test_size = 64*1024, 从 0 开始 → 覆盖 MEM_TABLE 前64KB
     * 现在: 从 MEM_STATIC_BUF 之后的空闲区开始, 测试 32KB 即可 */
    uint32_t test_start = MEM_STATIC_BUF_ADDR + MEM_STATIC_BUF_SIZE - SRAM_BASE_ADDR;
    uint32_t test_size  = 32 * 1024;

    test_data = 0x55AA;
    for (test_addr = test_start; test_addr < test_start + test_size; test_addr += 2)
    {
        if (sram_test_write_read_16b(test_addr, test_data))
        {
            return 1;
        }
        test_data ^= 0xFFFF;
    }

    for (test_addr = test_start; test_addr < test_start + test_size; test_addr += 2)
    {
        test_data = (uint16_t)(test_addr & 0xFFFF);
        *(volatile uint16_t *)(SRAM_BASE_ADDR + test_addr) = test_data;
    }

    for (test_addr = test_start; test_addr < test_start + test_size; test_addr += 2)
    {
        test_data = (uint16_t)(test_addr & 0xFFFF);
        if (*(volatile uint16_t *)(SRAM_BASE_ADDR + test_addr) != test_data)
        {
            return 2;
        }
    }

    return 0;
}

/**
 * @brief       Test function: write 1 byte to SRAM
 * @param       addr: address to write
 * @param       data: byte to write
 * @retval      none
 */
void sram_test_write(uint32_t addr, uint8_t data)
{
    sram_write(&data, addr, 1);
}

/**
 * @brief       Test function: read 1 byte from SRAM
 * @param       addr: address to read
 * @retval      read byte
 */
uint8_t sram_test_read(uint32_t addr)
{
    uint8_t data;
    sram_read(&data, addr, 1);
    return data;
}