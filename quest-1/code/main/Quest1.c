/*Team-2-Cherian-Han-Valiuddin
    9/22/20 
    */
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
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//Reset Button Pin
#define BUTTON 39

//Global Feeding time in Seconds
#define FEEDING_TIME 21600 // in seconds

// 14-Segment Display
#define SLAVE_ADDR 0x70              // alphanumeric address
#define OSC 0x21                     // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01 // Display on cmd
#define HT16K33_BLINK_OFF 0          // Blink off cmd
#define HT16K33_BLINK_CMD 0x80       // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0  // Brightness cmd

// Master I2C
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

//Timer consts
#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_INTERVAL0_SEC (1.0)                    // interval for the timer
#define TEST_WITH_RELOAD 1                           // testing will be done without auto reload

//Servo defines
#define SERVO_MIN_PULSEWIDTH 480  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2600 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 15       //Maximum angle in degree upto which servo can rotate

typedef struct
{
  int type; // the type of timer's event
  int timer_group;
  int timer_idx;
  uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

//numeric alphafonttable
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
//global variables initialization
static int global_count = FEEDING_TIME;
static int button_pressed = 0;

//display buffer initialization
static uint16_t displaybuffer[4];

//initialize LED's for binary counter
static void led_init()
{

  //Set output LEDs
  gpio_pad_select_gpio(BUTTON);

  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
}

// Servo initilization
static void mcpwm_example_gpio_initialize(void)
{
  printf("initializing mcpwm servo control gpio......\n");
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12); //Set GPIO 12 as PWM0A, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 * @return- calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
  uint32_t cal_pulsewidth = 0;
  cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
  return cal_pulsewidth;
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init()
{
  displaybuffer[0] = 0b0000000000000000; // 0
  displaybuffer[1] = 0b0000000000000000; // 0
  displaybuffer[2] = 0b0000000000000000; // 0
  displaybuffer[3] = 0b0000000000000000; // 0
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
  // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
  if (err == ESP_OK)
  {
    printf("- initialized: yes\n\n");
  }

  // Dat in MSB mode
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
  printf("\n>> I2C scanning ...\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++)
  {
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
    printf("\n");
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
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
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
  i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
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
  i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

//Timer functions

void IRAM_ATTR timer_group0_isr(void *para)
{
  timer_spinlock_take(TIMER_GROUP_0);
  int timer_idx = (int)para;

  /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
  uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

  /* Prepare basic event data
       that will be then sent back to the main program task */
  timer_event_t evt;
  evt.timer_group = 0;
  evt.timer_idx = timer_idx;
  evt.timer_counter_value = timer_counter_value;

  /* Clear the interrupt
       and update the alarm time for the timer with without reload */
  if (timer_intr & TIMER_INTR_T0)
  {
    evt.type = TEST_WITH_RELOAD;
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_counter_value += (uint64_t)(TIMER_INTERVAL0_SEC * TIMER_SCALE);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
  }
  else
  {
    evt.type = -1; // not supported even type
  }

  /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

  /* Now just send the event data back to the main program task */
  xQueueSendFromISR(timer_queue, &evt, NULL);
  timer_spinlock_give(TIMER_GROUP_0);
}

static void example_tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_sec)
{
  /* Select and initialize basic parameters of the timer */
  timer_config_t config = {
      .divider = TIMER_DIVIDER,
      .counter_dir = TIMER_COUNT_UP,
      .counter_en = TIMER_PAUSE,
      .alarm_en = TIMER_ALARM_EN,
      .auto_reload = auto_reload,
  }; // default clock source is APB
  timer_init(TIMER_GROUP_0, timer_idx, &config);

  /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
  timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP_0, timer_idx);
  timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                     (void *)timer_idx, ESP_INTR_FLAG_IRAM, NULL);

  timer_start(TIMER_GROUP_0, timer_idx);
}
/////////////////////////////////////////////////////////////////////////////////
//helper function for clock
static void counter_using_clock(void *args)
{
  int hour;
  int min;
  int sec;
  while (1)
  {
    //caluclate hours minutes and seconds
    min = global_count / 60 % 60;
    hour = global_count / 3600 % 24;
    sec = global_count % 60;
    timer_event_t evt;
    xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    //button flag raised decrements count otherwise resets back to set time interval when global_count = 0
    if (button_pressed)
    {
      global_count -= 1;
    }
    if (global_count < 0)
    {
      global_count = FEEDING_TIME;
    }
    //printf("global_count: %d\n", global_count);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    printf("Time till next feeding: %d hrs: %d min: %d sec\n", hour, min, sec);
  }
}

//int direction, uint16_t displaybuffer[8]
static void print_counter()
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
    i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i = 0; i < 4; i++)
    {
      i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);
    //calculate hour and min
    int min = global_count / 60 % 60;
    int hour = global_count / 3600 % 24;
    //show hours.minutes otherwise on the last hour show minutes.sec
    if (global_count >= 3600)
    {
      displaybuffer[0] = alphafonttable[(hour) / 10 % 10]; //
      displaybuffer[1] = alphafonttable[(hour) % 10] | (1 << 14);
      displaybuffer[2] = alphafonttable[(min) / 10 % 10]; //
      displaybuffer[3] = alphafonttable[(min) % 10];      //
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    else
    {
      displaybuffer[0] = alphafonttable[(min) / 10 % 10];               //
      displaybuffer[1] = alphafonttable[(min) % 10];                    //
      displaybuffer[2] = alphafonttable[(global_count % 60) / 10 % 10]; //
      displaybuffer[3] = alphafonttable[(global_count % 60) % 10];      //
    }
  }
}

static void button_press()
{
  //set button flag to 1 if pressed and start clock
  while (1)
  {
    if (gpio_get_level(BUTTON))
    {
      global_count = FEEDING_TIME;
      button_pressed = 1;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void servo_control(void *arg)
{
  uint32_t angle, count;
  //1. mcpwm gpio initialization
  mcpwm_example_gpio_initialize();

  //2. initial mcpwm configuration
  //printf("Configuring Initial Parameters of mcpwm......\n");
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 200; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
  pwm_config.cmpr_a = 0;      //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;      //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
  while (1)
  {
     //if global_count is 0, servo swings 3 times in smooth manner
    if (global_count < 1)
    {
      for (int times = 0; times < 3; times++)
      {

        for (count = 0; count < SERVO_MAX_DEGREE; count++)
        {
          angle = servo_per_degree_init(count);
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
          vTaskDelay(10); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
        for (count = SERVO_MAX_DEGREE; count > 0; count--)
        {
          angle = servo_per_degree_init(count);
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
          vTaskDelay(10); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void app_main()
{
  //initializations
  i2c_example_master_init();
  i2c_scanner();
  led_init();
  timer_queue = xQueueCreate(10, sizeof(timer_event_t));
  example_tg0_timer_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL0_SEC);
  //start tasks
  xTaskCreate(button_press, "button_press", 1024, NULL, 5, NULL);
  xTaskCreate(print_counter, "print_counter", 2048, NULL, 4, NULL);
  xTaskCreate(counter_using_clock, "counter_using_clock", 4096, NULL, 6, NULL);
  xTaskCreate(servo_control, "servo_control", 4096, NULL, 5, NULL);
}
