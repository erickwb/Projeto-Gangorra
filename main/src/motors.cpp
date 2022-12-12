#include "motors.hpp"
#include "driver/mcpwm.h"

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <cmath>
#include "sensors.hpp"
#include <iostream>


#define ACCELEROMETER_DEGREES_TO_RADIAN M_PI/180

#define Kd                              23.43
#define Ki                              0.15
#define Kp                              1.17
#define X                               0.0425
#define Ts                              0.05

#define REF_ANGLE                       0.0
#define MOTOR_LEFT                      12
#define MOTOR_RIGHT                     13

static float OUT_CONTROL_PAST1 = 0;
static float OUT_CONTROL_PAST2 = 0;

static float ERROR_CONTROL = 0;
static float ERROR_CONTROL_PAST1 = 0;
static float ERROR_CONTROL_PAST2 = 0;


// float i = 31;

void vInitMotors(void) {

     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_LEFT);
     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_RIGHT);
     mcpwm_config_t motor_pwm_config = {
         .frequency = 300, // frequency = 300Hz,
         .cmpr_a = 30,     // duty cycle of PWMxA
         .cmpr_b = 30,     // duty cycle of PWMxb
         .duty_mode = MCPWM_DUTY_MODE_0,
         .counter_mode = MCPWM_UP_COUNTER,
     };

     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_config);
     // vTaskDelay(3000/portTICK_PERIOD_MS);
     usleep(3000000);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40.0);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 33.0);
     /*
     ESP_LOGI("Frequency Motor_Left", "Using rev \"%u\"Hz", mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
     ESP_LOGI("Duty Cycle Motor_Left", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
     ESP_LOGI("Duty Cycle Motor_Right", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
     */
}

void vAdjustMotors(void) {

     float angle = fGetCurrentAngle();
     ESP_LOGI("Angle", "%.2f", angle);

}