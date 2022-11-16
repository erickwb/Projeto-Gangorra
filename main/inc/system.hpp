#ifndef SETTINGS_HPP
#define SETTINGS_HPP

typedef enum {
     ACCELEROMETER_SENSOR,
     ULTRASONIC_SENSOR
} Sensor;

typedef enum {
     DISCONNECTED_MODE,
     DEFAULT_MODE,
     FREE_MODE
} Mode;

typedef struct {
     float P;
     float I;
     float D;
} PID;

typedef struct {
     PID pid;
     float referenceAngle;
     Mode currentMode;
     Sensor currentSensor;
} SystemSettings;

extern bool bCheckModeUpdate(void);

extern void vComponentsInit(void);

extern void vStartupScreen(void);

extern float fGetReferenceAngle(void);

extern float fGetPidThermP(void);

extern float fGetPidThermI(void);

extern float fGetPidThermD(void);

extern Sensor xGetCurrentSensor(void);

extern Mode xGetCurrentMode(void);

extern void vSetReferenceAngle(float referenceAngle);

extern void vSetPidThermP(float P);

extern void vSetPidThermI(float I);

extern void vSetPidThermD(float D);

extern void vSetCurrentSensor(Sensor currentSensor);

extern void vSetCurrentMode(Mode currentMode);

#endif