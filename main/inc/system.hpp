#ifndef SETTINGS_HPP
#define SETTINGS_HPP

enum Sensor {
     ACCELEROMETER_SENSOR,
     ULTRASONIC_SENSOR
};

enum Mode {
     MANUAL_MODE,
     DEFAULT_MODE,
     DISCONNECTED_MODE
};

struct PID {
     float P;
     float I;
     float D;
};

struct SystemSettings {
     PID pid;
     float referenceAngle;
     Mode currentMode;
     Sensor currentSensor;
};

extern SystemSettings *xGetCurrentSettings(void);

extern void vSetCurrentSettings(SystemSettings *settings);

extern void vComponentsInit(void);

extern void vStartupScreen(void);

#endif