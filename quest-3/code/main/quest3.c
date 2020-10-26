//Quest 3
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
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

//#include "./init_functions.h"

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

// Function to initiate i2c-- note the MSB declaration !
static void i2c_master_init()
{
    // Debug
    printf("\n>> i2c Config\n");
    int err;
    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                        // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);     // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n");
    }

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner()
{
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."
           "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++)
    {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK)
        {
            printf("- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
    {
        printf("- No I2C devices found!"
               "\n");
    }
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data)
{
    // YOUR CODE HERE
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                   //start
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN); //slave address + write
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);                           // register address
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);                          // data
    i2c_master_stop(cmd);                                                    // stop
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg, uint8_t *data)
{
    // YOUR CODE HERE
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                      // 1. Start
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);    // 2.-3.
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);                              // 4.-5.
    i2c_master_start(cmd);                                                      // 6.
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);     // 7.-8.
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);                             // 9.-10. uint8_t pointer
    i2c_master_stop(cmd);                                                       // 11. Stop
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // This starts the I2C communication
    i2c_cmd_link_delete(cmd);
    return 0;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg, uint8_t *data1, uint8_t *data2)
{
    // YOUR CODE HERE

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data1, ACK_CHECK_DIS);
    i2c_master_read_byte(cmd, data2, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // This starts the I2C communication
    i2c_cmd_link_delete(cmd);
    //read_2b = ((int16_t)read_2 << 8 ) | read_1;
    return 0;
}

void setRange(range_t range)
{
    /* Red the data format register to preserve bits */
    uint8_t format;
    readRegister(ADXL343_REG_DATA_FORMAT, &format);
    /* Update the data rate */
    format &= ~0x0F;
    format |= range;

    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;

    /* Write the register back to the IC */
    writeRegister(ADXL343_REG_DATA_FORMAT, format);
}

range_t getRange(void)
{
    /* Red the data format register to preserve bits */
    uint8_t range;
    readRegister(ADXL343_REG_DATA_FORMAT, &range);
    return (range_t)(range & 0x03);
}

dataRate_t getDataRate(void)
{
    uint8_t rate;
    readRegister(ADXL343_REG_BW_RATE, &rate);
    return (dataRate_t)(rate & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

static void Accel_init()
{
    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5)
    {
        printf("\n>> Found ADAXL343\n");
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    // Set range
    setRange(ADXL343_RANGE_16_G);
    // Display range
    printf("- Range:         +/- ");
    switch (getRange())
    {
    case ADXL343_RANGE_16_G:
        printf("16 ");
        break;
    case ADXL343_RANGE_8_G:
        printf("8 ");
        break;
    case ADXL343_RANGE_4_G:
        printf("4 ");
        break;
    case ADXL343_RANGE_2_G:
        printf("2 ");
        break;
    default:
        printf("?? ");
        break;
    }
    printf(" g\n");

    // Display data rate
    printf("- Data Rate:    ");
    switch (getDataRate())
    {
    case ADXL343_DATARATE_3200_HZ:
        printf("3200 ");
        break;
    case ADXL343_DATARATE_1600_HZ:
        printf("1600 ");
        break;
    case ADXL343_DATARATE_800_HZ:
        printf("800 ");
        break;
    case ADXL343_DATARATE_400_HZ:
        printf("400 ");
        break;
    case ADXL343_DATARATE_200_HZ:
        printf("200 ");
        break;
    case ADXL343_DATARATE_100_HZ:
        printf("100 ");
        break;
    case ADXL343_DATARATE_50_HZ:
        printf("50 ");
        break;
    case ADXL343_DATARATE_25_HZ:
        printf("25 ");
        break;
    case ADXL343_DATARATE_12_5_HZ:
        printf("12.5 ");
        break;
    case ADXL343_DATARATE_6_25HZ:
        printf("6.25 ");
        break;
    case ADXL343_DATARATE_3_13_HZ:
        printf("3.13 ");
        break;
    case ADXL343_DATARATE_1_56_HZ:
        printf("1.56 ");
        break;
    case ADXL343_DATARATE_0_78_HZ:
        printf("0.78 ");
        break;
    case ADXL343_DATARATE_0_39_HZ:
        printf("0.39 ");
        break;
    case ADXL343_DATARATE_0_20_HZ:
        printf("0.20 ");
        break;
    case ADXL343_DATARATE_0_10_HZ:
        printf("0.10 ");
        break;
    default:
        printf("???? ");
        break;
    }
    printf(" Hz\n\n");

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);
}

//Wifi funtions
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

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
        timer++;
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
