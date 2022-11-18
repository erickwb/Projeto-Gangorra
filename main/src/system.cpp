#include "system.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.hpp"
#include "motors.hpp"
#include "display.hpp"
#include "bluetooth.hpp"
#include "delay.h"

#define SYSTEM_DEFAULT_P      1.0
#define SYSTEM_DEFAULT_I      1.0
#define SYSTEM_DEFAULT_D      0.0
#define SYSTEM_DEFAULT_ANGLE  0
#define SYSTEM_DEFAULT_MODE   DISCONNECTED_MODE
#define SYSTEM_DEFAULT_SENSOR ULTRASONIC_SENSOR
#define ACCELEROMETER         ACCELEROMETER_SENSOR

static SystemSettings currentSettings;

SystemSettings *xGetCurrentSettings(void) {
     return &currentSettings;
}

void vSetCurrentSettings(SystemSettings *settings) {
     currentSettings = *settings;
}

void vComponentsInit(void) {
     currentSettings = {
          .pid = {
               .P = SYSTEM_DEFAULT_P,
               .I = SYSTEM_DEFAULT_I,
               .D = SYSTEM_DEFAULT_D,
          },
          .referenceAngle = SYSTEM_DEFAULT_ANGLE,
          .currentMode = SYSTEM_DEFAULT_MODE,
          // .currentSensor = SYSTEM_DEFAULT_SENSOR
          .currentSensor = ACCELEROMETER
     };
     vInitAccelerometerSensor();
     vInitUltrasonicSensor();
     vInitMotors();
     vInitBluetooth();
     vInitDisplay();
}

void vStartupScreen(void) {
     vClearDisplay();
     vSetDisplayCursor(0, 0);
     vDisplayWriteString("Balanco Didatico");
     vSetDisplayCursor(1, 6);
     vDisplayWriteString("v0.1");
     vDelayMs(2000);

     vClearDisplay();
     vSetDisplayCursor(0, 0);
     vDisplayWriteString("Ligando Motores");
     vSetDisplayCursor(1, 5);
     vDisplayWriteString("em 3s");
     vDelayMs(1000);

     vSetDisplayCursor(1, 8);
     vDisplayWriteChar('2');
     vDelayMs(1000);

     vSetDisplayCursor(1, 8);
     vDisplayWriteChar('1');
     vDelayMs(1000);
}