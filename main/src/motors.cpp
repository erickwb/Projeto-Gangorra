#include "motors.hpp"
#include "driver/mcpwm.h"

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <cmath>
#include "sensors.hpp"

#define REF_ANGLE 0.0
#define MOTOR_LEFT 18
#define MOTOR_RIGHT 17

// float i = 31;

void vInitMotors(void)
{

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
     vTaskDelay(100 / portTICK_PERIOD_MS);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40);
     /*
     ESP_LOGI("Frequency Motor_Left", "Using rev \"%u\"Hz", mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
     ESP_LOGI("Duty Cycle Motor_Left", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
     ESP_LOGI("Duty Cycle Motor_Right", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
     */
}

void vAdjustMotors(void) {
     
     float angle = fGetCurrentAngle(); 
     ESP_LOGI("Angle", "%.2f", angle);
     float target = REF_ANGLE;
     float out_past1 = 0;
     float out_past2 = 0;
     float error = target - angle;
     float error1 = 0;
     float error2 = 0;

     float ts = 0.05;
     float Kp = 23.43;
     float Ki = 0.15;
     float Kd = 1.17;
     float dutycycle = 0;
     while (error != 0){
          float out = error * (2 * ts * Kp + Ki * pow(ts,2) + 4 * Kd) \
          + error1 * (2 * Ki * pow(ts,2) - 8 * Kd) \
          + error2 * (Ki * pow(ts,2) - 2 * ts * Kp + 4 * Kd) \
          - out_past2 * 2 * ts;

          if(error > 0){
               ESP_ERROR_CHECK(dutycycle = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
               dutycycle -= 0.25;
               ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutycycle));
          }else if(error < 0){
               ESP_ERROR_CHECK(dutycycle = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
               dutycycle += 0.25;
               ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutycycle));
          }


          out_past2 = out_past1;
          out_past1 = out;
          error2 = error1;
          error1 = error;
          error = out - target;
     }


     /*
     if (i > 100) {
          i = 0;
     }
     ESP_LOGI("Duty Cycle Motor_Left", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
     ESP_LOGI("Duty Cycle Motor_Right","%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, i);
     i += 10;
     vTaskDelay(1000 / portTICK_PERIOD_MS);
     */

     /* 
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40); 
     */

}