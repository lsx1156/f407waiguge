#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "stm32f4xx.h"

#define EEPROM_I2C_ADDR      0xA0
#define EEPROM_PAGE_SIZE     8
#define EEPROM_TOTAL_SIZE    256
#define EEPROM_WRITE_BUF_SIZE 64

void eeprom_init(void);
uint8_t eeprom_read_byte(uint16_t addr);
void eeprom_write_byte(uint16_t addr, uint8_t data);
void eeprom_read_buffer(uint16_t addr, uint8_t *buf, uint16_t len);
void eeprom_write_buffer(uint16_t addr, uint8_t *buf, uint16_t len);
void eeprom_process_write_buffer(void);

#endif
