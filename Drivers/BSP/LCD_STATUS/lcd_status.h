#ifndef __LCD_STATUS_H
#define __LCD_STATUS_H

#include "./SYSTEM/sys/sys.h"

#define LCD_STATUS_REFRESH_MS  300

void lcd_status_init(void);
void lcd_status_update(void);
void lcd_status_task(void);

#endif
