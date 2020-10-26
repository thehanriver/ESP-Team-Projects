//Quest 2
//Mario Han, Vivek Cherian, Hussain Valiuddin
#include <stdio.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_types.h"
#include "esp_attr.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
#include "math.h"
#include <stdio.h>
#include <math.h>
#include "./ADXL343.h"
#include "./init_functions.h"

//Wifi includes
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

//Thermistor defines
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 39  //Multisampling
#define E 2.718

//accel defines
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value
#define SLAVE_ADDR ADXL343_ADDRESS          // 0x53

//LED defines
#define BLINK_GPIO 13

//Wifi defines
#define EXAMPLE_ESP_WIFI_SSID "Group_2"
#define EXAMPLE_ESP_WIFI_PASS "1GBSt0rage!"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_3; //Thermistor GPIO 36 A3
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//Temp
static double temperature;

//Accel
static float xVal, yVal, zVal;

//LED
static int toggle;

//Wifi
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static int s_retry_num = 0;

static int timer;
//static char file_name[] = "../data/sensors.csv";

static void thermistor()
{
    while (1)
    {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading += adc1_get_raw((adc1_channel_t)channel1);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float temp, rtherm;
        rtherm = adc_reading;
        rtherm = (1000 * ((4096 / rtherm) - 1));
        temp = rtherm / 10000;
        temp = log(temp);
        temp /= 3435;
        temp += 1.0 / (20 + 273.15);
        temp = 1.0 / temp;
        temp -= 273.15;
        temperature = temp;
    }
}

// function to get acceleration
void getAccel(float *xp, float *yp, float *zp)
{
    uint8_t data_x0;
    uint8_t data_y0;
    uint8_t data_z0;

    uint8_t data_x1;
    uint8_t data_y1;
    uint8_t data_z1;

    uint16_t dataX;
    uint16_t dataY;
    uint16_t dataZ;

    readRegister(ADXL343_REG_DATAX0, &data_x0);
    readRegister(ADXL343_REG_DATAY0, &data_y0);
    readRegister(ADXL343_REG_DATAZ0, &data_z0);
    readRegister(ADXL343_REG_DATAX1, &data_x1);
    readRegister(ADXL343_REG_DATAY1, &data_y1);
    readRegister(ADXL343_REG_DATAZ1, &data_z1);

    dataX = (data_x0 | (data_x1 << 8));
    dataY = (data_y0 | (data_y1 << 8));
    dataZ = (data_z0 | (data_z1 << 8));
    int16_t ex = dataX;
    int16_t why = dataY;
    int16_t jeez = dataZ;

    *xp = ex * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *yp = why * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *zp = jeez * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    //printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343()
{
    printf("\n>> Polling ADAXL343\n");
    while (1)
    {
        getAccel(&xVal, &yVal, &zVal);
    }
}

// static void toggle_LED()
// {
//     gpio_pad_select_gpio(BLINK_GPIO);
//     /* Set the GPIO as a push/pull output */
//     gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

//     while (1)
//     {
//         gpio_set_level(BLINK_GPIO, toggle);
//         vTaskDelay(1000 / portTICK_RATE_MS);
//     }
// }

static void printstate()
{

    // static FILE *fp;
    // while (1)
    // {
    //     //add this line into the quest-2/code/data/sensors.csv
    //     if (timer == 0)
    //     {
    //         fp = fopen(file_name, "w");
    //         fprintf(fp, "Time: Temp: Dist(U_S): Dist2(IR)\n");
    //         fclose(fp);
    //     }
    //     else
    //     {
    //         fp = fopen(file_name, "a");
    //         int temp = timer - 2;
    //         fprintf(fp, "%d,%.1f,%d,%d\n", temp, temperature, ultrasonic, ir_rangefinder);
    //         fclose(fp);
    //     }
    //     timer += 2;
    //     vTaskDelay(2000 / portTICK_RATE_MS);
    // }

    while (1)
    {
        //add this line into the quest-2/code/data/sensors.csv
        if (timer == 0)
        {
            printf("Time: Dist(U_S): Dist2(IR): Temp\n");
        }
        else
        {
            int temp = timer - 2;
            printf("Time: %d \t Temperature: %.1f \t X: %.2f \t Y: %.2f \t Z: %.2f \t\n", temp, temperature, xVal, yVal, zVal);
            //printf("X: %.2f \t Y: %.2f \t Z: %.2f \t\n", x, y, z);
        }
        timer += 2;
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    //Wifi inits
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    //Configure ADC
    timer = 0;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel1, atten);

    // Routine
    i2c_master_init();
    i2c_scanner();
    Accel_init();

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // Create Task to get data from thermistor
    xTaskCreate(thermistor, "thermistor", 4096, NULL, 5, NULL);
    // Create Task to toggle LED
    //xTaskCreate(toggle_LED, "toggle_LED", 4096, NULL, 5, NULL);
    // Create task to poll ADXL343
    xTaskCreate(test_adxl343, "test_adxl343", 4096, NULL, 5, NULL);
    // Create Task to print out values received
    xTaskCreate(printstate, "printstate", 4096, NULL, 8, NULL);
}
