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

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

struct  timeval {
     	int tv_sec;
     	int tv_usec;
};

// LIDAR
#define SLAVE_ADDR (0x62) // 0x53
// #define RegisterMeasure (0x00) // Register to write to initiate ranging.
// #define MeasureValue (0x04)    // Value to initiate ranging.
// #define RegisterHighLowB 0x8f  // Register to get both High and Low bytes in 1 call.
//0xC4 write, 0xC5 read

static const char *TAG = "cmd_i2ctools";

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
static float distance = 0;
static float distance2 = 0;
// Flag for dt
int dt_complete = 0;

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond
void calibrateESC();

#define MAX_RIGHT 700
#define MAX_LEFT 2000
#define NEUTRAL 1300

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_3; //Thermistor GPIO 36 A3
static const adc_channel_t channel2 = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static int timer;
static double speedC;
static int count;
static float rotations;
static float speed_rpm;
static void calc_speed();

static float IR_left;
static float IR_right;
static float US_left;
static float US_right;
static float Lidar_front;

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
//LIDR
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
  while (1)
  {
    if (checkBit())
    {
      printf("Distance: %d\n", get_Distance());
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
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
      // #1
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
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
        //#2
        uint32_t adc_reading2 = 0;
        //Multisampling
        for (int j = 0; j < NO_OF_SAMPLES; j++) {
          //change this HUSSAIN
            if (unit == ADC_UNIT_1) {
                adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw2;
                adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
                adc_reading2 += raw2;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        adc_reading2 /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage2 = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        // display voltage
        distance2 = voltage_to_distance(voltage2);

        printf("Raw2: %d\tVoltage2: %dmV\tDistance2: %.2fm\n", adc_reading2, voltage2, distance2);
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
//Movement,speed,steering
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static void calc_speed(){
  vTaskDelay(6000 / portTICK_PERIOD_MS);

  while(1){
    count = 0;
    int flag = 0;
    for (int j = 0; j < 100; j++){
      uint32_t adc_reading = 0;
          if (unit == ADC_UNIT_1)
          {
              adc_reading = adc1_get_raw((adc1_channel_t)channel1);
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
    speed_rpm =  rotations * (62.0/100);
    printf(" speed: %f\n", speed_rpm);
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
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // Neutral signal in microseconds
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
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, NEUTRAL); // Neutral signal in microseconds
        vTaskDelay(4000 / portTICK_PERIOD_MS);
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
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    int S;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        // Setting scope_id to the connecting interface for correct routing if IPv6 Local Link supplied
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 2000000;

            socklen_t socklen = sizeof(source_addr);

            if(setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0){
            	perror("error time");
            	printf("errpr setsocket\n");
            	break;
            }

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len <= 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                /*
                	t = rx_buffer[0] -'0';
                	if(t==1){
						gpio_set_level(BLINK_GPIO, 1);
						ESP_LOGI(TAG,"LED on");
						S = 1;
						printf("State is %d\n",S);
					}
					else{
						gpio_set_level(BLINK_GPIO, 0);
						ESP_LOGI(TAG,"LED off");
						S = 0;
						printf("State is %d\n", S)
                	}
											*/
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
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
    xTaskCreate(PID_task, "display_task", 2048, NULL,3, NULL);
    xTaskCreate(movement, "movement", 4096, NULL, 5, NULL);
    xTaskCreate(steering, "steering", 4096, NULL, 5, NULL);
    xTaskCreate(calc_speed, "calc_speed", 4096, NULL, 6, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

}
