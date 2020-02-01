
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <dmx.h>

//#include "soc/mcpwm_periph.h"

#define HIGH 0x01
#define LOW 0x00

#define Shft_LD_pin 27   //LOW = load registers on next clock pulse
#define Shft_CLK_pin 32  //HIGH -> LOW clocks data
#define Shft_DAT_pin 33   //Data input high bit first
unsigned int DMXStrtAdrBCD = 0;
unsigned int DMXStrtAdr = 0;
unsigned int DMXAdr[4] = {0,0,0,0};
unsigned int DMXValNew[4] = {0,0,0,0};
unsigned int DMXValCurr[4] = {0,0,0,0};

bool bigendian = true;


float percent_multiplier = 0.392156; // 100%/255

//Define GPIO PWM pins and sync input pin
#define GPIO_PWM0A_OUT 26   //Set GPIO 13 as PWM0A, Dimmer channel 1
#define GPIO_PWM0B_OUT 25   //Set GPIO 18 as PWM0B, Dimmer channel 2
#define GPIO_PWM1A_OUT 15   //Set GPIO 15 as PWM1A, Dimmer channel 3
#define GPIO_PWM1B_OUT 14   //Set GPIO 14 as PWM1B, Dimmer channel 4
#define GPIO_SYNC0_IN   4   //Set GPIO 04 as SYNC0 (change to GPIO 36, blew 4?)

#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */

int slot_A = 0;

mcpwm_config_t pwm_config_timer0;
mcpwm_config_t pwm_config_timer1;
mcpwm_pin_config_t pin_config;

static void mcpwm_config()//(void *arg)
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
  pwm_config_timer0.cmpr_a = 99.0;       //duty cycle of PWMxA = 10.0%
  pwm_config_timer0.cmpr_b = 99.0;       //duty cycle of PWMxb = 50.0%
  pwm_config_timer0.counter_mode = MCPWM_UP_COUNTER;
  pwm_config_timer0.duty_mode = MCPWM_DUTY_MODE_1;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_timer0);   //Configure PWM0A & PWM0B with above settings
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_SYNC0, 0);    //Load counter value with 20% of period counter of mcpwm timer 0 when sync 0 occurs
  pwm_config_timer1.frequency = 120;    //frequency = 60Hz
  pwm_config_timer1.cmpr_a = 99.0;       //duty cycle of PWMxA = 60.0%
  pwm_config_timer1.cmpr_b = 99.0;       //duty cycle of PWMxb = 50.0%
  pwm_config_timer1.counter_mode = MCPWM_UP_COUNTER;
  pwm_config_timer1.duty_mode = MCPWM_DUTY_MODE_1;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_timer1);   //Configure PWM1A & PWM1B with above settings
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 0);    //Load counter value with 20% of period counter of mcpwm timer 0 when sync 0 occurs

  //vTaskDelete(NULL);
}

static void get_new_dmx_values(void *arg)
{
  int i;
  while (1) 
  {
    for(i=0; i<4; i++)
    {
      DMXValNew[i] = Read(DMXAdr[i]);
    }
  //Comment out for now...it only gets a 1 occasionally
  /*uint8_t health = IsHealthy();
  printf("IsHealthy %d\n", health);*/
  vTaskDelay(10 / portTICK_PERIOD_MS);         //delay of 10ms
  }

}

static void bump_duty_cycles(void *arg)
{
  while (1) 
  {
    if(DMXValNew[0] != DMXValCurr[0])
    {
      DMXValCurr[0] = DMXValNew[0];
      printf("DMX %d : %d\n", DMXAdr[0], DMXValCurr[0]);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100.0-(DMXValCurr[0]*percent_multiplier));
    }
    if(DMXValNew[1] != DMXValCurr[1])
    {
      DMXValCurr[1] = DMXValNew[1];
      printf("DMX %d : %d\n",DMXAdr[1], DMXValCurr[1]);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0-(DMXValCurr[1]*percent_multiplier));
    }
    if(DMXValNew[2] != DMXValCurr[2])
    {
      DMXValCurr[2] = DMXValNew[2];
      printf("DMX %d : %d\n",DMXAdr[2], DMXValCurr[2]);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 100.0-(DMXValCurr[2]*percent_multiplier));
    }
    if(DMXValNew[3] != DMXValCurr[3])
    {
      DMXValCurr[3] = DMXValNew[3];
      printf("DMX %d : %d\n",DMXAdr[3], DMXValCurr[3]);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0-(DMXValCurr[3]*percent_multiplier));
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);         //delay of 10ms
  }
}

static void view_duty_cycles(void *arg)
{
  while (1) 
  {
    printf("DMX %u\n", Read(1));
    printf("DMX %u\n", Read(2));
    printf("DMX %u\n", Read(476));
    printf("DMX %u\n", Read(512));
    vTaskDelay(100 / portTICK_PERIOD_MS);         //delay of 10ms
  }
}


static void tg0_timer0_init()
{
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_0;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
}

void Addressing()
{
gpio_set_level( Shft_LD_pin, HIGH );
gpio_set_level( Shft_CLK_pin, LOW );

//byte inbit = 0; //the number of bits recorded so far

// Set load low
gpio_set_level( Shft_LD_pin, LOW );

//clock once to load
gpio_set_level( Shft_CLK_pin, HIGH );
vTaskDelay(50 / portTICK_PERIOD_MS);
gpio_set_level( Shft_CLK_pin, LOW );

// Clr load
gpio_set_level( Shft_LD_pin, HIGH );

// Now loop to get the bits
for (int i=11; i>=0; i--)
  {
    if ( !gpio_get_level( Shft_DAT_pin ) )
    {
      //inbit = 1;
      DMXStrtAdrBCD |= (1 << i);
    }
    else
    {
      //inbit = 0;
    }
    //load next bit
    gpio_set_level( Shft_CLK_pin, HIGH );
    vTaskDelay(50 /  portTICK_PERIOD_MS);
    gpio_set_level( Shft_CLK_pin, LOW );
  }

return;

}  //end Addressing()

int BCDtoInt( int BCDVal )
{
  int ones;
  int tens;
  int hunds;
  // Little Endian
  if (bigendian){
    ones = BCDVal & 0x00f;
    tens = ((BCDVal & 0x0f0) >> 4) * 10;
    hunds = ((BCDVal & 0xf00) >> 8) * 100;
  } else{
    ones = ((BCDVal & 0xf00) >> 8);
    tens = ((BCDVal & 0x0f0) >> 4) * 10;
    hunds =  (BCDVal & 0x00f) * 100;
    
  }
  //Serial.println( ones );
  //Serial.println( tens );
  //Serial.println( hunds );
  return hunds + tens + ones;
}

static void get_dmx_address()
{
  int i;
    //Get the starting address from the thumbwheels
  //Serial.println("Get DMX address");
  gpio_set_direction( Shft_LD_pin, GPIO_MODE_OUTPUT );
  gpio_set_direction( Shft_CLK_pin, GPIO_MODE_OUTPUT );
  gpio_set_direction( Shft_DAT_pin, GPIO_MODE_INPUT );
  DMXStrtAdrBCD = 0;
  Addressing();
  DMXStrtAdr = BCDtoInt(DMXStrtAdrBCD);
  printf("DMX start address: %d\n", DMXStrtAdr);
  for(i=0; i<4; i++)
  {
    DMXAdr[i] = DMXStrtAdr + i;
  }
  //vTaskDelete(NULL);
}

void app_main(void)
{
    get_dmx_address();
    printf("Setup MCPWM...\n");
    mcpwm_config();
    //xTaskCreate(get_dmx_address, "", 4096, NULL, 5, NULL);
    //xTaskCreate(mcpwm_config, "mcpwm_config", 4096, NULL, 5, NULL);
    //tg0_timer0_init();
    printf("UART Init...\n");
    Initialize();
    printf("Start reading...\n");
    xTaskCreate(get_new_dmx_values,"get new dmx values", 4096, NULL, 5, NULL);
    //printf("Start bumping...\n");
    xTaskCreate(bump_duty_cycles, "bump_duty_cycles", 4096, NULL, 5, NULL);
    xTaskCreate(view_duty_cycles, "view_duty_cycles", 4096, NULL, 5, NULL);
}