#ifndef __LCDIF_H
#define __LCDIF_H

#include "stm32g0xx_hal.h"

#define LCD_CS 8
#define LCD_RST 9
#define LCD_READ_EN 10
#define LCD_WRITE_EN 11
#define LCD_A0 12

void lcd_idle();
void lcd_init();
void lcd_write(uint8_t cmd_dat, uint8_t data);

#endif /* __LCDIF_H */
