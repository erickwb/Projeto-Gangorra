#include "motors.hpp"
#include "driver/mcpwm.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define MOTOR_LEFT       18
#define MOTOR_RIGHT      17

// float i = 31;

void vInitMotors(void) {

     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_LEFT); 
     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_RIGHT); 
     mcpwm_config_t motor_pwm_config = {
          .frequency = 300,    //frequency = 300Hz,
          .cmpr_a = 30,    //duty cycle of PWMxA
          .cmpr_b = 30,    //duty cycle of PWMxb
          .duty_mode = MCPWM_DUTY_MODE_0,
          .counter_mode = MCPWM_UP_COUNTER,
     };

     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_config);

     /* 
     ESP_LOGI("Frequency Motor_Left", "Using rev \"%u\"Hz", mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
     ESP_LOGI("Duty Cycle Motor_Left", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
     ESP_LOGI("Duty Cycle Motor_Right", "%f %%", mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
     */
}

void vAdjustMotors(void) {
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
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40);
     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40);
}