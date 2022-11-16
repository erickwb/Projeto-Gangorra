#include "sensors.hpp"

#include <cmath>

#include "mpu6050.hpp"
#include "ultrasonic.hpp"

#define CM_TO_M                         100
#define ACCELEROMETER_RADIAN_TO_DEGREES 57.2958
#define ULTRASONIC_BAR_LENGTH           0.25
#define ULTRASONIC_MAX_DISTANCE_CM      400
#define ULTRASONIC_GPIO_TRIGGER_R       GPIO_NUM_32
#define ULTRASONIC_GPIO_ECHO_R          GPIO_NUM_34
#define ULTRASONIC_GPIO_TRIGGER_L       GPIO_NUM_16
#define ULTRASONIC_GPIO_ECHO_L          GPIO_NUM_17
#define ACCELEROMETER_I2C_SCL           GPIO_NUM_22
#define ACCELEROMETER_I2C_SDA           GPIO_NUM_21
#define ACCELEROMETER_I2C_PORT          I2C_NUM_0

static MPU6050 accelerometer(ACCELEROMETER_I2C_SCL, ACCELEROMETER_I2C_SDA, ACCELEROMETER_I2C_PORT);
static ultrasonic_sensor_t sensor_right;
static ultrasonic_sensor_t sensor_left;
static float currentAngle;

float fGetCurrentAngle(void) {
     return currentAngle;
}

bool vInitAccelerometerSensor(void) {
     return accelerometer.init();
}

void vInitUltrasonicSensor(void) {
     sensor_right = {
          .trigger_pin = ULTRASONIC_GPIO_TRIGGER_R,
          .echo_pin = ULTRASONIC_GPIO_ECHO_R
     };
     ultrasonic_init(&sensor_right);

     sensor_left = {
          .trigger_pin = ULTRASONIC_GPIO_TRIGGER_L,
          .echo_pin = ULTRASONIC_GPIO_ECHO_L
     };
     ultrasonic_init(&sensor_left);
}

float fReadAccelerometerSensor(void) {
     float ax = accelerometer.getAccX();
     float az = accelerometer.getAccZ();
     return (currentAngle = atan(ax/az)*ACCELEROMETER_RADIAN_TO_DEGREES);
}

float fReadUltrasonicSensor(void) {
     float distance_sensor_right = 0;
     float distance_sensor_left = 0;
     ultrasonic_measure(&sensor_right, ULTRASONIC_MAX_DISTANCE_CM, &distance_sensor_right);
     ultrasonic_measure(&sensor_left, ULTRASONIC_MAX_DISTANCE_CM, &distance_sensor_left);
     return (currentAngle = asin((distance_sensor_left - distance_sensor_right)/ULTRASONIC_BAR_LENGTH)*CM_TO_M);
}