#include "display.hpp"
#include "delay.h"
static lcd_handler_t lcdHandler;

const uint8_t null_char[] = {
     0b00000,
     0b00000,
     0b00000,
     0b00000,
     0b00000,
     0b00000,
     0b00000,
     0b00000
};

const uint8_t degree_char[] = {
    0b00110,
    0b01001,
    0b00110,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

void vInitDisplay(void) {
     lcdHandler = {
          .data = {GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_23},
          .rs = GPIO_NUM_33,
          .en = GPIO_NUM_19
     };
     lcdInitPinsModule(&lcdHandler);
     lcdCreateChar(&lcdHandler, lcdCUSTOM_CHAR_1, null_char);
     lcdCreateChar(&lcdHandler, lcdCUSTOM_CHAR_2, degree_char);
}

void vClearDisplay() {
     lcdClearDisplay(&lcdHandler);
}

void vSetDisplayCursor(uint8_t row, uint8_t columm) {
     lcdSetCursor(&lcdHandler, row, columm);
}

void vDisplayWriteChar(char c) {
     lcdWriteChar(&lcdHandler, c);
}     

void vDisplayWriteString(const char *str) {
     lcdWriteString(&lcdHandler, str);
}     

void vEraseDisplayCells(uint32_t row, uint32_t first_columm, uint32_t last_columm) {
     if(row > 1 || first_columm > last_columm || first_columm > 16 || last_columm > 16) {
          return;
     }
     lcdSetCursor(&lcdHandler, row, first_columm);
     while(first_columm++ <= last_columm) {
          lcdWriteChar(&lcdHandler, lcdCUSTOM_CHAR_1);
     }
}    