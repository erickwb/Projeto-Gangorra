#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "lcd.h"

#define LCD_DEGREE_CHAR lcdCUSTOM_CHAR_2

extern void vInitDisplay(void);

extern void vClearDisplay();

extern void vSetDisplayCursor(uint8_t row, uint8_t columm);

extern void vDisplayWriteChar(char c);

extern void vDisplayWriteString(const char *str);

extern void vEraseDisplayCells(uint32_t row, uint32_t first_columm, uint32_t last_columm);

#endif