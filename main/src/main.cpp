// =============================================================================
// LIBRARIES
// =============================================================================

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "bluetooth.hpp"
#include "display.hpp"
#include "motors.hpp"
#include "sensors.hpp"
#include "system.hpp"

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
static xSemaphoreHandle angleMutex;
static xSemaphoreHandle settingsMutex;
static bool updateDisplay = true;

// =============================================================================
// FREE FUNCTIONS PROTOTYPES
// =============================================================================

void vSystemInit(void);

// =============================================================================
// MAIN CODE
// =============================================================================

extern "C" void app_main(void) {
    vSystemInit();
    while (true);
}

// =============================================================================
// TASKS IMPLEMENTATION
// =============================================================================

static void vTaskSensor(void *pvParameters) {
    while(true) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        (xGetCurrentSettings()->currentSensor == ULTRASONIC_SENSOR) ? fReadUltrasonicSensor() : fReadAccelerometerSensor();
        xSemaphoreGive(settingsMutex);
        xSemaphoreGive(angleMutex);
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

static void vTaskMotors(void *pvParameters) {
    while(true) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        vAdjustMotors();
        xSemaphoreGive(settingsMutex);
        xSemaphoreGive(angleMutex);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

static void vTaskBluetooth(void *pvParameters) {
    BluetoothHandle handle;
    while(true) {
        xSemaphoreTake(settingsMutex, portMAX_DELAY);
        if (xGetCurrentSettings()->currentMode == DISCONNECTED_MODE) {
            vAwaitBluetoothConnection(&handle);
        }
        else {
            vReceiveBluetoothData(&handle);
            // updateDisplay = true;
        }
        xSemaphoreGive(settingsMutex);
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}

static void vTaskDisplay(void *pvParameters) {
    char angleString[16];
    while(true) {
        if(updateDisplay) {
            vClearDisplay();
            vSetDisplayCursor(0, 0);
            vDisplayWriteString("Angulacao: ");
            vSetDisplayCursor(1, 2);
            switch(xGetCurrentSettings()->currentMode) {
                case MANUAL_MODE:
                    vDisplayWriteString("Modo: Manual");
                    break;
                case DEFAULT_MODE:
                    vDisplayWriteString("Modo: Livre");
                    break;
                case DISCONNECTED_MODE:
                    vDisplayWriteString("Desconectado");
                    break;
            }
            updateDisplay = false;
        }
        vEraseDisplayCells(0, 11, 15);
        vSetDisplayCursor(0, 11);

        float angle = fGetCurrentAngle();
        sprintf(angleString, "%.1f", angle);
        if(angle > 0) {
            vDisplayWriteChar('+');
        }
        vDisplayWriteString(angleString);
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

// =============================================================================
// FREE FUNCTIONS IMPLEMENTATION
// =============================================================================

void vTasksInit(void) {
    angleMutex = xSemaphoreCreateMutex();
    settingsMutex = xSemaphoreCreateMutex();
    if (angleMutex && settingsMutex) {
        xTaskCreate(vTaskSensor, "TaskSensor", configMINIMAL_STACK_SIZE*10, NULL, 3, NULL);
        xTaskCreate(vTaskMotors, "TaskMotors", configMINIMAL_STACK_SIZE*10, NULL, 2, NULL);
        xTaskCreate(vTaskBluetooth, "TaskBluetooth", configMINIMAL_STACK_SIZE*10, NULL, 1, NULL);
        xTaskCreate(vTaskDisplay, "TaskDisplay", configMINIMAL_STACK_SIZE*10, NULL, 1, NULL);
    }
}

void vSystemInit(void) {
    vComponentsInit();
    vStartupScreen();
    vTasksInit();
}