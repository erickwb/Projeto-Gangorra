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

#define Kd                              79.52
#define Ki                              0.1
#define Kp                              126
#define X                               0.0425
#define Ts                              0.05
#define Ct                              2.3134

#define REF_ANGLE                       0.0
#define MOTOR_LEFT                      12
#define MOTOR_RIGHT                     13

float OUT_CONTROL_PAST1 = 0;
float OUT_CONTROL_PAST2 = 0;

float ERROR_CONTROL = 0;
float ERROR_CONTROL_PAST1 = 0;
float ERROR_CONTROL_PAST2 = 0;



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
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40.0);
     /*
     ESP_LOGI("Frequency Motor_Left", "Using rev \"%u\"Hz", mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
     ESP_LOGI("Duty Cycle Motor_Left", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
     ESP_LOGI("Duty Cycle Motor_Right", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
     */
}

void vAdjustMotors(void) {

     float angle = fGetCurrentAngle();
     ESP_LOGI("Angle", "%.2f", angle);

     if (angle > -14 && angle < 14) {


          ERROR_CONTROL = REF_ANGLE - angle;
          ERROR_CONTROL = ERROR_CONTROL * ACCELEROMETER_DEGREES_TO_RADIAN;

          float OUT_CONTROL = ERROR_CONTROL * (2 * Ts * Kp + Ki * pow(Ts, 2) + 4 * Kd) + ERROR_CONTROL_PAST1 * (2 * Ki * pow(Ts, 2) - 8 * Kd) + ERROR_CONTROL_PAST2 * (Ki * pow(Ts, 2) - 2 * Ts * Kp + 4 * Kd) - OUT_CONTROL_PAST2 * (2 * Ts);
          float pwm = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
          float out_control_ct = OUT_CONTROL * Ct * 1.25;
          ESP_LOGI("OUT_PWM1", "%.2f", out_control_ct);
          // if((REF_ANGLE - angle) < 0) {
          //      out_control_ct *= -1;
          // } else {
          //      out_control_ct *= 1;
          // }
          float out = out_control_ct * 0.3 + pwm;
          ESP_LOGI("OUT_PWM2", "%.2f", out);

          //out_pwm = out_pwm * 0.3 + pwm;
          usleep(5000);
          if((out > 31) && (out < 55)) {
               mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, out);
          } else if(out < 31) {
               mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 31);
          } else if(out > 55) {
               mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 55);
          } else {
               mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40);
          }

          ERROR_CONTROL_PAST2 = ERROR_CONTROL_PAST1;
          ERROR_CONTROL_PAST1 = ERROR_CONTROL;
          OUT_CONTROL_PAST2 = OUT_CONTROL_PAST1;
          OUT_CONTROL_PAST1 = OUT_CONTROL;

     }

}