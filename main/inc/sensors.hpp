#ifndef SENSORS_HPP
#define SENSORS_HPP

extern float fGetCurrentAngle(void);

extern bool vInitAccelerometerSensor(void);

extern void vInitUltrasonicSensor(void);

extern float fReadAccelerometerSensor(void);

extern float fReadUltrasonicSensor(void);

#endif