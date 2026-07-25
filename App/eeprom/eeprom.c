#include "eeprom.h"
#include <string.h>

extern I2C_HandleTypeDef g_i2c1_handle;

typedef struct {
    uint8_t data[EEPROM_WRITE_BUF_SIZE];
    uint16_t addr;
    uint16_t len;
    uint16_t current_addr;
    uint16_t remaining;
    uint8_t state;
    uint32_t wait_timestamp;
} EEPROMWriteBuffer_t;

static EEPROMWriteBuffer_t g_write_buf = {0};

void eeprom_init(void)
{
    memset(&g_write_buf, 0, sizeof(g_write_buf));

    g_i2c1_handle.Instance = I2C1;
    g_i2c1_handle.Init.ClockSpeed = 100000;
    g_i2c1_handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_i2c1_handle.Init.OwnAddress1 = 0;
    g_i2c1_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_i2c1_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_i2c1_handle.Init.OwnAddress2 = 0;
    g_i2c1_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_i2c1_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&g_i2c1_handle);
}

uint8_t eeprom_read_byte(uint16_t addr)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&g_i2c1_handle, EEPROM_I2C_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

void eeprom_write_byte(uint16_t addr, uint8_t data)
{
    HAL_I2C_Mem_Write(&g_i2c1_handle, EEPROM_I2C_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(5);
}

void eeprom_read_buffer(uint16_t addr, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read(&g_i2c1_handle, EEPROM_I2C_ADDR, addr, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void eeprom_write_buffer(uint16_t addr, uint8_t *buf, uint16_t len)
{
    if (len > EEPROM_WRITE_BUF_SIZE) {
        len = EEPROM_WRITE_BUF_SIZE;
    }
    
    g_write_buf.addr = addr;
    g_write_buf.len = len;
    g_write_buf.current_addr = addr;
    g_write_buf.remaining = len;
    g_write_buf.state = 1;
    g_write_buf.wait_timestamp = 0;
    for (int i = 0; i < len; i++) {
        g_write_buf.data[i] = buf[i];
    }
}

void eeprom_process_write_buffer(void)
{
    if (g_write_buf.state == 0 || g_write_buf.remaining == 0) {
        return;
    }
    
    if (g_write_buf.state == 1) {
        uint16_t page_offset = g_write_buf.current_addr % EEPROM_PAGE_SIZE;
        uint16_t write_len = (g_write_buf.remaining > (EEPROM_PAGE_SIZE - page_offset)) ? 
                             (EEPROM_PAGE_SIZE - page_offset) : g_write_buf.remaining;
        
        HAL_I2C_Mem_Write(&g_i2c1_handle, EEPROM_I2C_ADDR, g_write_buf.current_addr, 
                          I2C_MEMADD_SIZE_8BIT, g_write_buf.data + (g_write_buf.current_addr - g_write_buf.addr), 
                          write_len, 100);
        
        g_write_buf.current_addr += write_len;
        g_write_buf.remaining -= write_len;
        g_write_buf.state = 2;
        g_write_buf.wait_timestamp = HAL_GetTick();
    } else if (g_write_buf.state == 2) {
        if (HAL_GetTick() - g_write_buf.wait_timestamp >= 5) {
            if (g_write_buf.remaining > 0) {
                g_write_buf.state = 1;
            } else {
                g_write_buf.state = 0;
                g_write_buf.len = 0;
            }
        }
    }
}
