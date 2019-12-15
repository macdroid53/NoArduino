
#include "_ansi.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
//#include "soc/mcpwm_periph.h"
#include <dmx.h>
float percent_multiplier = 0.392156; // 100%/255

//Define GPIO PWM pins and sync input pin
#define GPIO_PWM0A_OUT 26   //Set GPIO 13 as PWM0A, Dimmer channel 1
#define GPIO_PWM0B_OUT 25   //Set GPIO 18 as PWM0B, Dimmer channel 2
#define GPIO_PWM1A_OUT 15   //Set GPIO 15 as PWM1A, Dimmer channel 3
#define GPIO_PWM1B_OUT 14   //Set GPIO 14 as PWM1B, Dimmer channel 4
#define GPIO_SYNC0_IN   4   //Set GPIO 04 as SYNC0 (change to GPIO 36, blew 4?)

int slot_A = 0;

mcpwm_config_t pwm_config_timer0;
mcpwm_config_t pwm_config_timer1;
mcpwm_pin_config_t pin_config;

static void mcpwm_config(void *arg)
{
  //Configure sync, timers, PWM and init duty cycles to 0% or triacs off
  //Serial.println("initializing mcpwm gpio quad...\n");
  
  pin_config.mcpwm0a_out_num = GPIO_PWM0A_OUT;
  pin_config.mcpwm0b_out_num = GPIO_PWM0B_OUT;
  pin_config.mcpwm1a_out_num = GPIO_PWM1A_OUT;
  pin_config.mcpwm1b_out_num = GPIO_PWM1B_OUT;
  pin_config.mcpwm_sync0_in_num  = GPIO_SYNC0_IN;
  mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
  gpio_pulldown_en((gpio_num_t) GPIO_PWM1A_OUT);
  //gpio_pulldown_en((gpio_num_t) GPIO_SYNC0_IN);
  //gpio_pulldown_dis(GPIO_NUM_4);   //Enable pull down on SYNC0  signal
  gpio_pulldown_dis((gpio_num_t) GPIO_SYNC0_IN);   //Enable pull down on SYNC0  signal
  //gpio_pulldown_dis(GPIO_NUM_36);
  //mcpwm_config_t pwm_config;
  pwm_config_timer0.frequency = 120;    //frequency = 60Hz
  pwm_config_timer0.cmpr_a = 80.0;       //duty cycle of PWMxA = 10.0%
  pwm_config_timer0.cmpr_b = 80.0;       //duty cycle of PWMxb = 50.0%
  pwm_config_timer0.counter_mode = MCPWM_UP_COUNTER;
  pwm_config_timer0.duty_mode = MCPWM_DUTY_MODE_1;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_timer0);   //Configure PWM0A & PWM0B with above settings
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_SYNC0, 0);    //Load counter value with 20% of period counter of mcpwm timer 0 when sync 0 occurs
  pwm_config_timer1.frequency = 120;    //frequency = 60Hz
  pwm_config_timer1.cmpr_a = 80.0;       //duty cycle of PWMxA = 60.0%
  pwm_config_timer1.cmpr_b = 80.0;       //duty cycle of PWMxb = 50.0%
  pwm_config_timer1.counter_mode = MCPWM_UP_COUNTER;
  pwm_config_timer1.duty_mode = MCPWM_DUTY_MODE_1;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_timer1);   //Configure PWM1A & PWM1B with above settings
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 0);    //Load counter value with 20% of period counter of mcpwm timer 0 when sync 0 occurs

  vTaskDelete(NULL);
}

static void bump_duty_cycles(void *arg)
{
    while (1) {
        slot_A = Read(1);
        printf("slot a : %d\n", slot_A);
    }

}

void app_main(void)
{
    printf("Testing MCPWM...\n");
    xTaskCreate(mcpwm_config, "mcpwm_config", 4096, NULL, 5, NULL);
    Initialize();
    xTaskCreate(bump_duty_cycles, "bump_duty_cycles", 4096, NULL, 5, NULL);
}