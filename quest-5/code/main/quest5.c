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

#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 20  //Multisampling
#define E 2.718

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
#define HOST_IP_ADDR "192.168.1.111" //"192.168.1.139"
#define PORT 1234

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

struct  timeval {
     	int tv_sec;
     	int tv_usec;
};

#define TIMER_INTERVAL_SEC 0.100
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

#define DISTANCE_SET_POINT 0.50 // m
#define SPEED_SET_POINT 2 // m/s
#define K_p 50
#define K_i 0
#define K_d 0

#define GPIO_RED 12
#define GPIO_GREEN 27
#define GPIO_BLUE 15

// ultrasound defines
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   20          //Multisampling

// ultrasound variables
static float distance = 0;
static float distance2 = 0;
// Flag for dt
static int dt_complete = 0;

static float s_PID_output;
static float d_PID_output;

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
void calibrateESC();

#define MAX_RIGHT 700
#define MAX_LEFT 2000
#define MIDDLE 1300


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_3; //Ultrasonic right GPIO 36 A3
static const adc_channel_t channel2 = ADC_CHANNEL_0; //IR left GPIO 34 A4
static const adc_channel_t channel3 = ADC_CHANNEL_6; //ultrasonic left GPIO 39 A2
static const adc_channel_t channel4 = ADC_CHANNEL_4; //IR right GPIO 32
static const adc_channel_t channel5 = ADC_CHANNEL_5; //Speed Sensor GPIO 33

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static int timer;
static double speedC;
static int count;
static float rotations;
static float measured_speed_m_per_s;
static void calc_speed();
static int set_pwm_speed; //1400 neutral
static int set_pwm_steering; //1300 straight

static double US_left = 0;
static double US_right = 0;
static double IR_left = 0;
static double IR_right = 0;
static int timer;
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
  printf("Wrote 0 to 4 \n");
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
  printf("%d, %d\n", val1, val2);

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
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    i2c_cmd_link_delete(cmd2);

    if ((val1 & 1) == 0)
    {
      return 1;
    }
  }
}

void LIDAR_task()
{
    int i;
    int sum;
    while (1)
    {
        sum = 0;
        for (i = 0; i < 10; i++)
        {
            if (checkBit())
            {
                sum += get_Distance();
            }
            vTaskDelay(50 / portTICK_RATE_MS);
        }
        dist = (sum / 10.0) - 13;
        LIDAR_front = dist;
        // printf("Distance: %f\n", dist);
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
        adc1_config_channel_atten(channel2, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel2, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}


// returns distances in cm
// static double ultrasound_v_to_d(uint32_t reading)
// {
//     if (reading==0)
//     {
//         printf("returning 0");
//         return 0;
//     }
//     else
//     {
//         double dist = ((1 / 6.4 * (reading))-4.25)*.0254; // 6.4 mV per in
//         // uint32_t dist = (1 / 6.4 * (reading)); // 6.4 mV per in
//         return dist;
//     }
// }


// static void ultrasound_task()
// {
//     //Continuously sample ADC1
//     while (1) {
//       // #1
//         uint32_t adc_reading = 0;
//         //Multisampling
//         for (int i = 0; i < NO_OF_SAMPLES; i++) {
//             if (unit == ADC_UNIT_1) {
//                 adc_reading += adc1_get_raw((adc1_channel_t)channel2);
//             } else {
//                 int raw;
//                 adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
//                 adc_reading += raw;
//             }
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//         }
//         adc_reading /= NO_OF_SAMPLES;
//         //Convert adc_reading to voltage in mV
//         uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

//         // display voltage
//         distance = ultrasound_v_to_d(voltage);

//         printf("Raw: %d\tVoltage: %dmV\tDistance: %.2fm\n", adc_reading, voltage, distance);
//         //#2
//         uint32_t adc_reading2 = 0;
//         //Multisampling
//         for (int j = 0; j < NO_OF_SAMPLES; j++) {
//           //change this HUSSAIN
//             if (unit == ADC_UNIT_1) {
//                 adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
//             } else {
//                 int raw2;
//                 adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
//                 adc_reading2 += raw2;
//             }
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//         }
//         adc_reading2 /= NO_OF_SAMPLES;
//         //Convert adc_reading to voltage in mV
//         uint32_t voltage2 = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

//         // display voltage
//         distance2 = ultrasound_v_to_d(voltage2);

//         printf("Raw2: %d\tVoltage2: %dmV\tDistance2: %.2fm\n", adc_reading2, voltage2, distance2);
//     }
// }

static void ultrasound_task()
{
    uint32_t left, right, volt_right, volt_left;
    double dist_left, dist_right;
    while (1)
    {
        left = 0;
        right = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                right += adc1_get_raw((adc1_channel_t)channel4);
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

static void IR_task()
{
    uint32_t left, right, volt_right, volt_left;
    double dist_left, dist_right;
    int dist;
    double temp;
    while (1)
    {
        left = 0;
        right = 0;
        dist = 0;
        temp = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                right += adc1_get_raw((adc1_channel_t)channel1);
                left += adc1_get_raw((adc1_channel_t)channel3);
                vTaskDelay(50 / portTICK_RATE_MS);
            }
        }
        left /= NO_OF_SAMPLES;
        right /= NO_OF_SAMPLES;
        volt_right = esp_adc_cal_raw_to_voltage(right, adc_chars);
        volt_left = esp_adc_cal_raw_to_voltage(left, adc_chars);
        volt_right /= 1000;
        volt_left /= 1000;
        dist_right = ((1 / 6.4 * (volt_right)) - 4.25) * .0254;
        dist_left = ((1 / 6.4 * (volt_left)) - 4.25) * .0254;
        US_right = dist_right;
        US_left = dist_left;
    }
}

static void PID_task()
{
    float d_error, d_derivative, d_previous_error, d_integral;
	float s_error, s_derivative, s_previous_error, s_integral;
	float dt = 0.1; // 100 ms
    d_previous_error = 0;
    d_integral = 0;
	s_previous_error = 0;
	s_integral = 0;
    


    while(1)
    {
      if (dt_complete == 1) {
		  // distance PID
          d_error = DISTANCE_SET_POINT - measured_distance;
          d_integral = d_integral + d_error * dt;
          d_derivative = (d_error - d_previous_error) / dt;
          d_PID_output = K_p * d_error + K_i * d_integral + K_d * d_derivative;
          d_previous_error = d_error;

          if (d_error<-0.005)
          {
              gpio_set_level(GPIO_RED,1);
              gpio_set_level(GPIO_GREEN,0);
              gpio_set_level(GPIO_BLUE,0);
          }
          else if (d_error>0.005)
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

		  // speed PID
		  
		  s_error = SPEED_SET_POINT - measured_speed_m_per_s;
          s_integral = s_integral + s_error * dt;
          s_derivative = (s_error - s_previous_error) / dt;
          s_PID_output = K_p * s_error + K_i * s_integral + K_d * s_derivative;
          s_previous_error = s_error;

          if (s_error<-0.005)
          {
              gpio_set_level(GPIO_RED,1);
              gpio_set_level(GPIO_GREEN,0);
              gpio_set_level(GPIO_BLUE,0);
          }
          else if (s_error>0.005)
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

static void steering_PID_task() {
    float error, derivative, output, previous_error, integral, dt;
    previous_error = 0;
    integral = 0;
    dt = 0.1; // 100 ms

    //#define MAX_RIGHT 700
    //#define MAX_LEFT 2000
    //#define NEUTRAL 1300

    //IR_left, IR_right, US_left, US_right

    if(IR_left < 25 || US_left < 25)
    {

    }

    else()
    {}


    while(1)
    {
      if(IR_left < 25 || US_left < 25)
      {
        
      }

    else()
    {}

      else
      {
        set_pwm_steering = MIDDLE;
      }
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

        vTaskDelay(50 / portTICK_RATE_MS);
    }

}
//Movement,speed,steering
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static void control_pwm() {
	if (Lidar_front < DISTANCE_SET_POINT)
	{
		set_pwm_speed += d_PID_output;
	}
	else
	{
		set_pwm_speed += s_PID_output;
	}
}


static void calc_speed(){
  vTaskDelay(6000 / portTICK_PERIOD_MS);

  while(1){
    count = 0;
    int flag = 0;
    for (int j = 0; j < 100; j++){
      uint32_t adc_reading = 0;
          if (unit == ADC_UNIT_1)
          {
              adc_reading = adc1_get_raw((adc1_channel_t)channel5);
              vTaskDelay(10 / portTICK_PERIOD_MS);
          }
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
    measured_speed_m_per_s =  rotations * (62.0/100);
    printf(" speed: %f\n", measured_speed_m_per_s);
  }
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12); //Set GPIO 12 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 27); //Steering
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
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, set_pwm_speed); // Neutral signal in microseconds
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, set_pwm_steering); // Neutral signal in microseconds
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    while (1)
    {

        char addr_str[128];
        int addr_family;
        int ip_protocol;

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
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
            char buffer2[128];
            int status;
            status = sprintf(buffer2, "%d", start);
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
                if (rx_buffer[0] == '1')
                {
                    start = 1;
                }
                else
                {
                    start = 0;
                }
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
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

        displaybuffer[0] = alphafonttable[((int)dist) % 10] | (1 << 14);
        displaybuffer[1] = alphafonttable[(int)((dist)*10) % 10];   //
        displaybuffer[2] = alphafonttable[(int)((dist)*100) % 10];  //
        displaybuffer[2] = alphafonttable[(int)((dist)*1000) % 10]; //

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

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
    init();
    // Routine
    i2c_master_init();
    i2c_scanner();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
   /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    * Read "Establishing Wi-Fi or Ethernet Connection" section in
    * examples/protocols/README.md for more information about this function.
    */
    ESP_ERROR_CHECK(example_connect());
    xTaskCreate(LIDAR_task, "LIDAR_task", 4096, NULL, 5, NULL);
    xTaskCreate(ultrasound_task, "button_task", 2048, NULL, 4, NULL);
    xTaskCreate(IR_task, "button_task", 2048, NULL, 4, NULL);
    xTaskCreate(PID_task, "display_task", 2048, NULL,3, NULL);
    xTaskCreate(movement, "movement", 4096, NULL, 5, NULL);
    xTaskCreate(steering, "steering", 4096, NULL, 5, NULL);
    xTaskCreate(calc_speed, "calc_speed", 4096, NULL, 6, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(alphanum_display, "alphanum_display", 2048, NULL, 4, NULL);
}
