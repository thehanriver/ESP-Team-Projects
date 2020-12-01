/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   Quest 5
   Hussain,Mario,Vivek - 11/29/20
*/
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/param.h>
#include "esp_system.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
#include "esp_console.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "driver/timer.h"
#include "driver/i2c.h"
#include "soc/mcpwm_periph.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "math.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_wifi.h"
#include "freertos/event_groups.h"

// 14-Segment Display
#define SLAVE_DISPLAY_ADDR 0x70      // alphanumeric address
#define OSC 0x21                     // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01 // Display on cmd
#define HT16K33_BLINK_OFF 0          // Blink off cmd
#define HT16K33_BLINK_CMD 0x80       // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0  // Brightness cmd

// LIDAR
#define SLAVE_ADDR (0x62) // 0x53

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 400000   // i2c master clock freq 400kHz
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value

//UDP
#define HOST_IP_ADDR "192.168.1.139" //"192.168.1.139"
#define PORT 1234

//Wifi defines
#define EXAMPLE_ESP_WIFI_SSID "Group_2"
#define EXAMPLE_ESP_WIFI_PASS "1GBSt0rage!"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define TIMER_INTERVAL_SEC 0.5                       // 100 ms
#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TEST_WITH_RELOAD 1                           // Testing will be done with auto reload

#define FRONT_SET_POINT 0.50 // m
#define SPEED_SET_POINT 2    // m/s
#define RIGHT_SET_POINT 0.28 // m
#define LEFT_SET_POINT 0.28  // m
#define K_p 50
#define K_i 0
#define K_d 0

// ultrasound defines
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 10  //Multisampling
#define E 2.718

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond

#define MAX_RIGHT 700
#define MAX_LEFT 2000
#define MIDDLE 1300 //straight

#define NEUTRAL 1400 //not moving

#define STOP_PWM_INCREMENT 50

//Wifi
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// UDP
static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

// Flag for dt
static int dt_complete = 0;

static float PID_speed;
static float PID_distance;
static float PID_steering;

void calibrateESC();

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_0; //Ultrasonic right GPIO 36 A3
static const adc_channel_t channel2 = ADC_CHANNEL_3; //ultrasonic left GPIO 39 A2
static const adc_channel_t channel3 = ADC_CHANNEL_5; //IR right GPIO 33
static const adc_channel_t channel4 = ADC_CHANNEL_6; //IR left GPIO 34 A4
static const adc_channel_t channel5 = ADC_CHANNEL_4; //Speed Sensor GPIO 32

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static int count;
static float rotations;
static float measured_speed_m_per_s;
static void calc_speed();
static int pwm_movement; //1400 neutral
static int pwm_steering; //1300 straight

static float US_left = 0;
static float US_right = 0;
static float IR_left = 0;
static float IR_right = 0;
static int start = 0;
static float LIDAR_front;

static const uint16_t alphafonttable[10] = {

    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000011001111, // 3
    0b0000000011100110, // 4
    0b0000000011101101, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111  // 9

};
static uint16_t displaybuffer[4];
// Function to initiate i2c -- note the MSB declaration!
static void alphanum_init()
{
    displaybuffer[0] = 0b0000000000000000; // 0
    displaybuffer[1] = 0b0000000000000000; // 0
    displaybuffer[2] = 0b0000000000000000; // 0
    displaybuffer[3] = 0b0000000000000000; // 0
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12); //Set GPIO 12 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 27); //Steering
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 14); //speed
}

//-------------------------Wifi Inits---------------------------------//
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

//I2c
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to initiate i2c -- note the MSB declaration!
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
    uint8_t cnt = 0;
    for (uint8_t i = 1; i < 127; i++)
    {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK)
        {
            printf("- Device found at address: 0x%X%s", i, "\n");
            cnt++;
        }
    }
    if (cnt == 0)
    {
        printf("- No I2C devices found!"
               "\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator()
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_DISPLAY_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set blink rate to off
int no_blink()
{
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_DISPLAY_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val)
{
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_DISPLAY_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
//LIDAR
////////////////////////////////////////////////////////////////////////////////

uint16_t get_Distance()
{
    uint8_t val1;
    uint8_t val2;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                   //start
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN); //slave address + write
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);                          // register address
    i2c_master_write_byte(cmd, 0x04, ACK_CHECK_EN);                          // data
    i2c_master_stop(cmd);                                                    // stop
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Write OK");
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else
    {
        ESP_LOGW(TAG, "Write Failed");
    }
    vTaskDelay(20 / portTICK_RATE_MS);

    i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();
    i2c_master_start(cmd1);
    i2c_master_write_byte(cmd1, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd1, 0x8F, ACK_CHECK_EN);
    i2c_master_stop(cmd1);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_RATE_MS);

    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd2, &val1, ACK_VAL);
    i2c_master_read_byte(cmd2, &val2, ACK_VAL);
    i2c_master_stop(cmd2);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    i2c_cmd_link_delete(cmd1);
    i2c_cmd_link_delete(cmd2);

    uint16_t output = (uint16_t)((val1 << 8) | val2);
    return output;
}

int checkBit()
{
    while (1)
    {
        uint8_t val1;

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, (0x01), ACK_CHECK_EN);
        i2c_master_stop(cmd);

        i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
        i2c_master_start(cmd2);
        i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd2, &val1, ACK_CHECK_DIS);
        i2c_master_stop(cmd2);

        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        i2c_cmd_link_delete(cmd2);

        if ((val1 & 1) == 0)
        {
            return 1;
        }
    }
}

//PID
////////////////////////////////////////////////////////////////////////////////
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

    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// BEGIN ULTRASOUND FUNCTIONS

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void ultrasonic_task()
{
    uint32_t left, right, volt_right, volt_left;
    float dist_left, dist_right;
    while (1)
    {
        left = 0;
        right = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                right += adc1_get_raw((adc1_channel_t)channel1);
                left += adc1_get_raw((adc1_channel_t)channel2);
                vTaskDelay(50 / portTICK_RATE_MS);
            }
        }
        left /= NO_OF_SAMPLES;
        right /= NO_OF_SAMPLES;
        volt_right = esp_adc_cal_raw_to_voltage(right, adc_chars);
        volt_left = esp_adc_cal_raw_to_voltage(left, adc_chars);
        dist_right = ((1 / 6.4 * (volt_right)) - 4.25) * .0254;
        dist_left = ((1 / 6.4 * (volt_left)) - 4.25) * .0254;
        US_right = dist_right;
        US_left = dist_left;
    }
}

float IR_v_to_dist(float voltage)
{
    int d;
    float temp;
    if (voltage > 2)
    {
        d = (30 / (voltage - 1));
    }
    else if (voltage < 2 && voltage > 1)
    {
        d = (57 / (voltage - 0.08));
    }
    else
    {
        temp = (3 - voltage) / 0.5;
        temp = pow(E, temp);
        temp = temp / 1.4;
        d = temp + 25.5;
    }
    d = d / 100.0;
    return d;
}

static void IR_task()
{
    uint32_t left, right, volt_right, volt_left;
    while (1)
    {
        left = 0;
        right = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                right += adc1_get_raw((adc1_channel_t)channel3);
                left += adc1_get_raw((adc1_channel_t)channel4);
                vTaskDelay(50 / portTICK_RATE_MS);
            }
        }
        left /= NO_OF_SAMPLES;
        right /= NO_OF_SAMPLES;
        volt_right = esp_adc_cal_raw_to_voltage(right, adc_chars);
        volt_left = esp_adc_cal_raw_to_voltage(left, adc_chars);
        volt_right /= 1000;
        volt_left /= 1000;
        IR_right = IR_v_to_dist(volt_right);
        IR_left = IR_v_to_dist(volt_left);
    }
}

void LIDAR_task()
{
    int i;
    uint16_t sum;
    float dist;
    while (1)
    {
        sum = 0;
        for (i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (checkBit())
            {
                sum += get_Distance();
            }
            vTaskDelay(50 / portTICK_RATE_MS);
        }
        dist = (sum / NO_OF_SAMPLES) - 13;
        LIDAR_front = dist;
        // printf("Distance: %f\n", dist);
    }
}

// }
//Movement,speed,steering
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static void set_pwm()
{
    if (start == 0) // if start is set to 0, move towards NEUTRAL and MIDDLE
    {
        // adjust speed to NEUTRAL
        if (pwm_movement < NEUTRAL + STOP_PWM_INCREMENT)
        {
            pwm_movement = NEUTRAL;
        }
        else
        {
            pwm_movement -= STOP_PWM_INCREMENT;
        }
        // adjust steering to middle
        if (pwm_steering < MIDDLE - STOP_PWM_INCREMENT)
        {
            pwm_steering += STOP_PWM_INCREMENT;
        }
        else if (pwm_steering > MIDDLE + STOP_PWM_INCREMENT)
        {
            pwm_steering -= STOP_PWM_INCREMENT;
        }
        else
        {
            pwm_steering = MIDDLE;
        }
    }
    else
    {
        if (LIDAR_front < FRONT_SET_POINT)
        {
            pwm_movement += PID_distance;
        }
        else
        {
            pwm_movement += PID_speed;
        }

        if (IR_right > RIGHT_SET_POINT && IR_left > LEFT_SET_POINT)
        {
            // adjust steering to middle
            if (pwm_steering < MIDDLE - STOP_PWM_INCREMENT)
            {
                pwm_steering += STOP_PWM_INCREMENT;
            }
            else if (pwm_steering > MIDDLE + STOP_PWM_INCREMENT)
            {
                pwm_steering -= STOP_PWM_INCREMENT;
            }
            else
            {
                pwm_steering = MIDDLE;
            }
        }
        else
        {
            pwm_steering += PID_steering;
        }

        pwm_steering += PID_steering;
    }
}

static void PID_task()
{
    float d_error, d_derivative, d_previous_error = 0, d_integral = 0;
    float sp_error, sp_derivative, sp_previous_error = 0, sp_integral = 0;
    float st_L_error, st_L_derivative, st_L_previous_error = 0, st_L_integral = 0;
    float st_R_error, st_R_derivative, st_R_previous_error = 0, st_R_integral = 0;
    float dt = TIMER_INTERVAL_SEC; // 50 ms

    while (1)
    {
        if (dt_complete == 1)
        {
            // distance PID
            d_error = FRONT_SET_POINT - LIDAR_front;
            d_integral = d_integral + d_error * dt;
            d_derivative = (d_error - d_previous_error) / dt;
            PID_distance = K_p * d_error + K_i * d_integral + K_d * d_derivative;
            d_previous_error = d_error;
            // speed PID

            sp_error = SPEED_SET_POINT - measured_speed_m_per_s;
            sp_integral = sp_integral + sp_error * dt;
            sp_derivative = (sp_error - sp_previous_error) / dt;
            PID_speed = K_p * sp_error + K_i * sp_integral + K_d * sp_derivative;
            sp_previous_error = sp_error;

            // steering PID will go here

            st_R_error = RIGHT_SET_POINT - IR_right;
            st_R_integral = st_R_integral + st_R_error * dt;
            st_R_derivative = (st_R_error - st_R_previous_error) / dt;
            PID_steering = (IR_right > RIGHT_SET_POINT) ? 0 : K_p * st_R_error + K_i * st_R_integral + K_d * st_R_derivative;
            st_R_previous_error = st_R_error;

            st_L_error = LEFT_SET_POINT - IR_left;
            st_L_integral = st_L_integral + st_L_error * dt;
            st_L_derivative = (st_L_error - st_L_previous_error) / dt;
            PID_steering -= (IR_left > LEFT_SET_POINT) ? 0 : K_p * st_L_error + K_i * st_L_integral + K_d * st_L_derivative;
            st_L_previous_error = st_L_error;

            dt_complete = 0;
            set_pwm();
            // Re-enable alarm
            TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
        }

        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

static void calc_speed()
{
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    while (1)
    {
        count = 0;
        int flag = 0;
        for (int j = 0; j < 50; j++)
        {
            uint32_t adc_reading = 0;
            if (unit == ADC_UNIT_1)
            {
                adc_reading = adc1_get_raw((adc1_channel_t)channel5);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            // printf(" volts : %d\n", voltage);

            if (voltage > 2000 && flag == 1)
            {
                count += 1;
                flag = 0;
            }
            if (voltage < 2000)
            {
                flag = 1;
            }
        }
        rotations = count / 6.0;
        measured_speed_m_per_s = rotations * (23.5 / 100);
    }
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
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm_movement); // Neutral signal in microseconds
        vTaskDelay(250 / portTICK_PERIOD_MS);
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
    vTaskDelay(2000 / portTICK_PERIOD_MS);                // Give yourself time to turn on crawler
    while (1)
    {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, pwm_steering); // Neutral signal in microseconds
        vTaskDelay(250 / portTICK_PERIOD_MS);
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
//UDP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1)
    {

        char addr_str[128];
        int addr_family;
        int ip_protocol;

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1)
        {
            char buffer2[4];
            snprintf(buffer2, 4, "%d", start);
            payload = buffer2;
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else
            {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                // ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                // ESP_LOGI(TAG, "%s", rx_buffer);
                ESP_LOGI(TAG, "Received %c", rx_buffer[0]);
                if (rx_buffer[0] == '1')
                {
                    start = 1;
                }
                else
                {
                    start = 0;
                }
            }

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

//alphanum prints speed
static void alphanum_display()
{

    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if (ret == ESP_OK)
    {
        printf("- oscillator: ok \n");
    }
    // Set display blink off
    ret = no_blink();
    if (ret == ESP_OK)
    {
        printf("- blink: off \n");
    }
    ret = set_brightness_max(0xF);
    if (ret == ESP_OK)
    {
        printf("- brightness: max \n");
    }

    // Initiallize characters to buffer
    while (1)
    {
        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, (SLAVE_DISPLAY_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i = 0; i < 4; i++)
        {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);

        displaybuffer[0] = alphafonttable[((int)measured_speed_m_per_s) % 10] | (1 << 14);
        displaybuffer[1] = alphafonttable[(int)((measured_speed_m_per_s)*10) % 10];   //
        displaybuffer[2] = alphafonttable[(int)((measured_speed_m_per_s)*100) % 10];  //
        displaybuffer[3] = alphafonttable[(int)((measured_speed_m_per_s)*1000) % 10]; //

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
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

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel1, atten);
    adc1_config_channel_atten(channel2, atten);
    adc1_config_channel_atten(channel3, atten);
    adc1_config_channel_atten(channel4, atten);
    adc1_config_channel_atten(channel5, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    mcpwm_example_gpio_initialize();
    // Routine
    i2c_master_init();
    i2c_scanner();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    // ESP_ERROR_CHECK(example_connect());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    * Read "Establishing Wi-Fi or Ethernet Connection" section in
    * examples/protocols/README.md for more information about this function.
    */
    xTaskCreate(LIDAR_task, "LIDAR_task", 4096, NULL, 5, NULL);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 4, NULL);
    xTaskCreate(IR_task, "ultrasonic_task", 2048, NULL, 4, NULL);
    xTaskCreate(PID_task, "PID_task", 2048, NULL, 3, NULL);
    xTaskCreate(movement, "movement", 4096, NULL, 5, NULL);
    xTaskCreate(steering, "steering", 4096, NULL, 5, NULL);
    xTaskCreate(calc_speed, "calc_speed", 4096, NULL, 6, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(alphanum_display, "alphanum_display", 2048, NULL, 4, NULL);
}
