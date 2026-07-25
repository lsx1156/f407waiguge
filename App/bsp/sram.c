#include "sram.h"
#include "bsp_config.h"

SRAM_HandleTypeDef g_sram_handler;

static uint8_t sram_test_write_read_16b(uint32_t addr, uint16_t data);

void sram_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FSMC_NORSRAM_TimingTypeDef fsmc_timing;

    SRAM_CS_GPIO_CLK_ENABLE();
    SRAM_WR_GPIO_CLK_ENABLE();
    SRAM_RD_GPIO_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_FSMC_CLK_ENABLE();

    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FSMC;

    /* PG10: FSMC_NE3 (片选) */
    gpio_init_struct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);

    /* PD4: FSMC_NOE (读), PD5: FSMC_NWE (写) */
    gpio_init_struct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    /* PE0: FSMC_NBL0, PE1: FSMC_NBL1 (高低字节选择) */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    /* GPIOD: D0/D1/D2/D3/D13/D14/D15/A16/A17/A18
       PD0=D2, PD1=D3, PD8=D13, PD9=D14, PD10=D15,
       PD11=A16, PD12=A17, PD13=A18, PD14=D0, PD15=D1 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                           GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                           GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    /* GPIOE: D4~D12
       PE7=D4, PE8=D5, PE9=D6, PE10=D7, PE11=D8,
       PE12=D9, PE13=D10, PE14=D11, PE15=D12 */
    gpio_init_struct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                           GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                           GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    /* GPIOF: A0~A5, A6~A9
       PF0=A0, PF1=A1, PF2=A2, PF3=A3, PF4=A4, PF5=A5,
       PF12=A6, PF13=A7, PF14=A8, PF15=A9 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12 |
                           GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);

    /* GPIOG: A10/A11/A12
       PG0=A10, PG1=A11, PG2=A12  (PG10=NE3 已在上方初始化) */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);

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

    fsmc_timing.AddressSetupTime = 0x02;
    fsmc_timing.AddressHoldTime = 0x01;
    fsmc_timing.DataSetupTime = 0x08;
    fsmc_timing.BusTurnAroundDuration = 0x00;
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
    uint32_t test_size = 64 * 1024;

    test_data = 0x55AA;
    for (test_addr = 0; test_addr < test_size; test_addr += 2)
    {
        if (sram_test_write_read_16b(test_addr, test_data))
        {
            return 1;
        }
        test_data ^= 0xFFFF;
    }

    for (test_addr = 0; test_addr < test_size; test_addr += 2)
    {
        test_data = (uint16_t)(test_addr & 0xFFFF);
        *(volatile uint16_t *)(SRAM_BASE_ADDR + test_addr) = test_data;
    }

    for (test_addr = 0; test_addr < test_size; test_addr += 2)
    {
        test_data = (uint16_t)(test_addr & 0xFFFF);
        if (*(volatile uint16_t *)(SRAM_BASE_ADDR + test_addr) != test_data)
        {
            return 2;
        }
    }

    return 0;
}