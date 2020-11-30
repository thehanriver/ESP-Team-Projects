/*
Authored by Vivek Cherian
11-28-2020
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_vfs_dev.h"
#include "driver/timer.h"
//ultrasound includes
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>



#define TIMER_INTERVAL_SEC 0.100
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

#define SET_POINT 0.50
#define K_p 1
#define K_i 1
#define K_d 1

#define GPIO_RED 12
#define GPIO_GREEN 27
#define GPIO_BLUE 33

// ultrasound defines
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   20          //Multisampling


// ultrasound variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static float distance = 0;


// Flag for dt
int dt_complete = 0;

// Define timer interrupt handler
void IRAM_ATTR timer_isr()
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    // Indicate timer has fired
    dt_complete = 1;
}

// Set up periodic timer for dt = 100ms
static void periodic_timer_init()
{
    // Basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_DOWN;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);

    // Configure the alarm value and the interrupt on alarm.
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 0);

    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr,(void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}


static void led_init()
{
    gpio_reset_pin(GPIO_RED);
    gpio_pad_select_gpio(GPIO_RED);
    gpio_set_direction(GPIO_RED,GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_GREEN);
    gpio_pad_select_gpio(GPIO_GREEN);
    gpio_set_direction(GPIO_GREEN,GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_BLUE);
    gpio_pad_select_gpio(GPIO_BLUE);
    gpio_set_direction(GPIO_BLUE,GPIO_MODE_OUTPUT);
}

// BEGIN ULTRASOUND FUNCTIONS

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
static void init()
{

    led_init();
    periodic_timer_init();

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}


// returns distances in cm
static double voltage_to_distance(uint32_t reading)
{
    if (reading==0)
    {
        printf("returning 0");
        return 0;
    }
    else
    {
        double dist = ((1 / 6.4 * (reading))-4.25)*.0254; // 6.4 mV per in
        // uint32_t dist = (1 / 6.4 * (reading)); // 6.4 mV per in
        return dist;
    }
}


static void ultrasound_task()
{
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        // display voltage
        distance = voltage_to_distance(voltage);

        printf("Raw: %d\tVoltage: %dmV\tDistance: %.2fm\n", adc_reading, voltage, distance);
    }
}

static void PID_task() {
    float error, derivative, output, previous_error, integral, dt;
    previous_error = 0;
    integral = 0;
    dt = 0.1; // 100 ms


    while(1)
    {
        if (dt_complete == 1) {
            error = SET_POINT - distance;
            integral = integral + error * dt;
            derivative = (error - previous_error) / dt;
            output = K_p * error + K_i * integral + K_d * derivative;
            previous_error = error;
            if (error<-0.005)
            {
                gpio_set_level(GPIO_RED,1);
                gpio_set_level(GPIO_GREEN,0);
                gpio_set_level(GPIO_BLUE,0);
            }
            else if (error>0.005)
            {
                gpio_set_level(GPIO_RED,0);
                gpio_set_level(GPIO_GREEN,0);
                gpio_set_level(GPIO_BLUE,1);
            }
            else
            {
                gpio_set_level(GPIO_RED,0);
                gpio_set_level(GPIO_GREEN,1);
                gpio_set_level(GPIO_BLUE,0);
            }
            dt_complete = 0;
            // Re-enable alarm
            TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
        }
        
        vTaskDelay(100 / portTICK_RATE_MS);
    }

}

void app_main(){

    init();
    xTaskCreate(ultrasound_task, "ultrasound_task", 2048, NULL, 4, NULL);
    xTaskCreate(PID_task, "PID_task", 2048, NULL,3, NULL);
}
