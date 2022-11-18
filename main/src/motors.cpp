#include "motors.hpp"
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include <cmath>

#define LEDC_TIMER1              LEDC_TIMER_1
#define LEDC_TIMER2              LEDC_TIMER_2
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO1          (18) // Define the output GPIO
#define LEDC_OUTPUT_IO2          (17) // Define the output GPIO
#define LEDC_CHANNEL1            LEDC_CHANNEL_0
#define LEDC_CHANNEL2            LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY1               (3194) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_DUTY2               (3194) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (300) // Frequency in Hertz. Set frequency at 5 kHz

static void example_ledc_init1(void) {
     // Prepare and then apply the LEDC PWM timer configuration
     ledc_timer_config_t ledc_timer = {
         .speed_mode = LEDC_MODE,
         .duty_resolution = LEDC_DUTY_RES,
         .timer_num = LEDC_TIMER1,
         .freq_hz = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
         .clk_cfg = LEDC_AUTO_CLK
     };
     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

     // Prepare and then apply the LEDC PWM channel configuration
     ledc_channel_config_t ledc_channel = {
         .gpio_num = LEDC_OUTPUT_IO1,
         .speed_mode = LEDC_MODE,
         .channel = LEDC_CHANNEL1,
         .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = LEDC_TIMER1,
         .duty = 0, // Set duty to 0%
         .hpoint = 0,
         .flags = {
           .output_invert = 0
         }
     };
     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void example_ledc_init2(void) {
     // Prepare and then apply the LEDC PWM timer configuration
     ledc_timer_config_t ledc_timer = {
         .speed_mode = LEDC_MODE,
         .duty_resolution = LEDC_DUTY_RES,
         .timer_num = LEDC_TIMER2,
         .freq_hz = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
         .clk_cfg = LEDC_AUTO_CLK
     };
     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

     // Prepare and then apply the LEDC PWM channel configuration
     ledc_channel_config_t ledc_channel = {
         .gpio_num = LEDC_OUTPUT_IO2,
         .speed_mode = LEDC_MODE,
         .channel = LEDC_CHANNEL2,
         .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = LEDC_TIMER2,
         .duty = 0, // Set duty to 0%
         .hpoint = 0,
         .flags = {
           .output_invert = 0
         }
     };
     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void vInitMotors(void) {
     example_ledc_init1();
     // Set duty to 50%
     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL1, LEDC_DUTY1));
     // Update duty to apply the new value
     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL1));

     example_ledc_init2();
     // Set duty to 50%
     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, LEDC_DUTY2));
     // Update duty to apply the new value
     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
}

void vAdjustMotors(void) { //dado do sensor e target
     // int target = ; //target
     // int out_past1 = 0;//dado do sensor
     // int out_past2 = 0 
     // int err = ; //target-dado do sensor
     // int err1 = 0;
     // int err2 = 0;

     // float ts=0.05;
     // float kd=23.43;
     // float ki=0.15;
     // float kp=1.17;

     // while(err!=0){
     //     float out = err*(2*ts*kp+ki*pow(ts,2)+4*kd)+err1*(2*ki*pow(ts,2)-8*kd)+err2*(ki*pow(ts,2)-2*ts*kd_4*kd)-out_past2*2*ts;

     //     if(err > 0){
     //        LEDC_DUTY2 -= 0.25;
     //        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, LEDC_DUTY2));
     //        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
     //     }else if(err < 0){
     //        LEDC_DUTY2 += 0.25;
     //        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, LEDC_DUTY2));
     //        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
     //     }



     //     out_past2 = out_past1;
     //     out_past = out;
     //     err2 = err1;
     //     err1 = err;
     //     err = out - target;
     // }
}
