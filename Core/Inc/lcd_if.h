#ifndef __LCDIF_H
#define __LCDIF_H

#include "stm32g0xx_hal.h"

#define LCD_CS 8
#define LCD_RST 9
#define LCD_READ_EN 10
#define LCD_WRITE_EN 11
#define LCD_A0 12

#define LCD 0
#define SSD1309 1
#define SSD1305 2

#define DISPLAY_TYPE SSD1309
void displayInit();
void oledInit();
void lcdInit();
void resetDisplay();
void clearDisplay();
void displayWrite(uint8_t cmd_dat, uint8_t data);

#endif /* __LCDIF_H */
