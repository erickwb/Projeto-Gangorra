#include "system.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.hpp"
#include "motors.hpp"
#include "display.hpp"
#include "bluetooth.hpp"
#include "delay.hpp"

#define SYSTEM_ANGLE_UPPER_LIMIT   14
#define SYSTEM_ANGLE_LOWER_LIMIT   -14
#define SYSTEM_DEFAULT_P           1.0
#define SYSTEM_DEFAULT_I           1.0
#define SYSTEM_DEFAULT_D           0.0
#define SYSTEM_DEFAULT_ANGLE       0
#define SYSTEM_DEFAULT_MODE        DISCONNECTED_MODE
#define SYSTEM_DEFAULT_SENSOR      ULTRASONIC_SENSOR

static SystemSettings currentSettings;
static bool modeUpdate;

bool bCheckModeUpdate(void) {
     bool currentModeUpdate = modeUpdate;
     modeUpdate = false;
     return currentModeUpdate;
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
          .currentSensor = SYSTEM_DEFAULT_SENSOR
     };
     modeUpdate = true;
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
     vDisplayWriteString("v0.2");
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

// =============================================================================
// SETTERS
// =============================================================================

void vSetReferenceAngle(float referenceAngle) {
     if(referenceAngle > SYSTEM_ANGLE_UPPER_LIMIT || referenceAngle < SYSTEM_ANGLE_LOWER_LIMIT) {
          return;
     }
     currentSettings.referenceAngle = referenceAngle;
}

void vSetPidThermP(float P) {
     currentSettings.pid.P = P;
}

void vSetPidThermI(float I) {
     currentSettings.pid.I = I;
}

void vSetPidThermD(float D) {
     currentSettings.pid.D = D;
}

void vSetCurrentSensor(Sensor currentSensor) {
     if(currentSensor > ULTRASONIC_SENSOR && currentSensor < ACCELEROMETER_SENSOR) {
          return;
     }
     currentSettings.currentSensor = currentSensor;
}

void vSetCurrentMode(Mode currentMode) {
     if(currentMode > FREE_MODE && currentMode < DISCONNECTED_MODE) {
          return;
     }
     currentSettings.currentMode = currentMode;
     modeUpdate = true;
}

// =============================================================================
// GETTERS
// =============================================================================

float fGetReferenceAngle(void) {
     return currentSettings.referenceAngle;
}

float fGetPidThermP(void) {
     return currentSettings.pid.P;
}

float fGetPidThermI(void) {
     return currentSettings.pid.I;
}

float fGetPidThermD(void) {
     return currentSettings.pid.D;
}

Sensor xGetCurrentSensor(void) {
     return currentSettings.currentSensor;
}

Mode xGetCurrentMode(void) {
     return currentSettings.currentMode;
}