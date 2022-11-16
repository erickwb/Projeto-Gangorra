#include "delay.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros() {
    return (unsigned long) (esp_timer_get_time());
}

void IRAM_ATTR delayMicroseconds(uint32_t us) {
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e) {
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

void vDelayMs(int ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
    uint32_t remainderUS = (ms % portTICK_PERIOD_MS)*1000;
    if(remainderUS) delayMicroseconds(remainderUS);
}

/*
void vDelayMs(int ms) {
     TickType_t currentTick = xTaskGetTickCount();
     while(xTaskGetTickCount() - currentTick < pdMS_TO_TICKS(ms));
}
*/