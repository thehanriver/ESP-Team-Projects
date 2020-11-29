/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "math.h"


//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
void calibrateESC();

#define MAX_RIGHT 700
#define MAX_LEFT 2000
#define NEUTRAL 1300
#define SPEED_GPIO 39

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_3; //Thermistor GPIO 36 A3
// static const adc_channel_t channel2 = ADC_CHANNEL_6; //IR GPIO 39 A2
// static const adc_channel_t channel3 = ADC_CHANNEL_0; //ultrasonic GPIO 34 A4
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 10  //Multisampling
static int timer;
static double speedC;
static int count;
static float rotations;
static float speed_rpm;
static void calc_speed();
//
// static void printstate()
// {
//
//     while (1)
//     {
//         //add this line into the quest-2/code/data/sensors.csv
//         if (timer == 0)
//         {
//             printf("SPEEED: \n");
//         }
//         else
//         {
//
//
//             printf(" volts : %f\n", speedC);
//             // printf(" speed: %f\n", speed_rpm);
//         }
//         timer += 2;
//         vTaskDelay(10 / portTICK_RATE_MS);
//     }
// }

// static void speed(){
//   while (1)
//   {
//     uint32_t adc_reading = 0;
//     //Multisampling
//     // for (int i = 0; i < NO_OF_SAMPLES; i++)
//     // {
//         if (unit == ADC_UNIT_1)
//         {
//             adc_reading = adc1_get_raw((adc1_channel_t)channel1);
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//         }
//     // }
//     // adc_reading /= NO_OF_SAMPLES;
//
//     //Convert adc_reading to voltage in mV
//     uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
//
//     printf(" volts : %d\n", voltage);
//   }
// }

static void calc_speed(){
  vTaskDelay(6000 / portTICK_PERIOD_MS);
  while(1){

    count = 0;
    int flag = 0;
    for (int j = 0; j < 40; j++){
      uint32_t adc_reading = 0;
      //Multisampling
      // for (int i = 0; i < NO_OF_SAMPLES; i++)
      // {
          if (unit == ADC_UNIT_1)
          {
              adc_reading = adc1_get_raw((adc1_channel_t)channel1);
              vTaskDelay(10 / portTICK_PERIOD_MS);
          }
      // }
      // adc_reading /= NO_OF_SAMPLES;

      //Convert adc_reading to voltage in mV
      uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
      // printf(" volts : %d\n", voltage);

      if(voltage > 2000 && flag == 1){
        count += 1;
        flag = 0;
      }
      if(voltage < 2000){
        flag = 1;
      }

    }

    rotations = count / 6.0;
    speed_rpm =  rotations * 60 * (62.0/100);
    // printf("count: %d\n", count);
    printf(" speed: %f\n", speed_rpm);


  }

}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12); //Set GPIO 12 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 12); //Steering
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 32); //speed
}

/**
 * @brief Configure MCPWM module
 */
void movement(void *arg)
{
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Give yourself time to turn on crawler
    printf("Calibrate\n");
    calibrateESC();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Give yourself time to turn on crawler

    while (1)
    {
        int i;
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // Neutral signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds
        vTaskDelay(50 / portTICK_PERIOD_MS);
        for (i = SERVO_MIN_PULSEWIDTH; i <= 2100; i = i + 10)
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i);
            printf("Count %d\n", i);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        for (i = SERVO_MAX_PULSEWIDTH; i >= 1400; i = i - 10)
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i);
            printf("Count %d\n", i);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        printf("Neutral\n");
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // LOW signal in microseconds
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // LOW signal in microseconds
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        for (i = 700; i <= 1300; i = i + 10)
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i);
            printf("Count %d\n", i);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Reset\n");
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // LOW signal in microseconds
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void steering(void *arg)
{
    //2. initial mcpwm configuration
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    vTaskDelay(1000 / portTICK_PERIOD_MS);                // Give yourself time to turn on crawler
    while (1)
    {
        int i;
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, NEUTRAL); // Neutral signal in microseconds
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MAX_LEFT); // LOW signal in microseconds
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MAX_RIGHT); // LOW signal in microseconds
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        for (i = MAX_RIGHT; i <= MAX_LEFT; i = i + 10)
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, i);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        for (i = MAX_LEFT; i >= MAX_RIGHT; i = i - 10)
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, i);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void calibrateESC()
{
    printf("Calibrate ESC\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);                                // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // LOW signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
}

void app_main(void)
{
    timer = 0;
    printf("Testing servo motor.......\n");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel1, atten);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    mcpwm_example_gpio_initialize();
    //
     xTaskCreate(movement, "movement", 4096, NULL, 5, NULL);
    // xTaskCreate(steering, "steering", 4096, NULL, 5, NULL);
     xTaskCreate(calc_speed, "calc_speed", 4096, NULL, 6, NULL);
       // xTaskCreate(printstate, "printstate", 4096, NULL, 6, NULL);
}
