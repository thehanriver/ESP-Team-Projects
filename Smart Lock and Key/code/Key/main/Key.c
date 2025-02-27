/* Infrared IR/UART beacons for crawler capture the flag!
   November 2019 -- Emily Lam

   RMT Pulse          -- pin 26 -- A0
   UART Transmitter   -- pin 25 -- A1
   UART Receiver      -- pin 34 wrong // 36 right-- A2 // A4

   Hardware interrupt/Button -- pin 4 -- A5
   ID Indicator       -- pin 13 -- Onboard LED

   Red LED            -- pin 15
   Green LED          -- pin 32
   Blue LED           -- Pin 14

   Features:
   - Sends UART payload -- | START | myColor | myID | Checksum? |
   - Outputs 38kHz using RMT for IR transmission
   - Onboard LED blinks device ID (myID)
   - Button press to change device ID
   - RGB LED shows traffic light state (red, green, blue)
   - Timer controls traffic light state (r - 10s, g - 10s, y - 2s)
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"

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

// ADXL343
#define SLAVE_ADDR ADXL343_ADDRESS // 0x53

// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 36 // A4 34 // A2 receiver
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1 4 //Button pin A5
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1

// LED Output pins definitions
#define LEDPIN 12

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_2_SEC (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload

// Default ID/color
#define ID 1

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char)ID;
int len_out = 6;

int Xint, Yint, Zint;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct
{
  int flag; // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "system"; // For debug logs
static void led_task();
// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para)
{

  // Prepare basic event data, aka set flag
  timer_event_t evt;
  evt.flag = 1;

  // // blue is shorter
  // if (myColor == 'G')
  // {
  //   timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
  // }
  // else
  // {
  //   timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
  // }

  // Clear the interrupt, Timer 0 in group 0
  TIMERG0.int_clr_timers.t0 = 1;

  // After the alarm triggers, we need to re-enable it to trigger it next time
  TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

  // Send the event data back to the main program task
  xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum
char genCheckSum(char *p, int len)
{
  char temp = 0;
  for (int i = 0; i < len; i++)
  {
    temp = temp ^ p[i];
  }
  // printf("%X\n",temp);

  return temp;
}
bool checkCheckSum(uint8_t *p, int len)
{
  char temp = (char)0;
  bool isValid;
  for (int i = 0; i < len - 1; i++)
  {
    temp = temp ^ p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len - 1])
  {
    isValid = true;
  }
  else
  {
    isValid = false;
  }
  return isValid;
}

//Accel inits

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

////////////////////////////////////////////////////////////////////////////////
// Init Functions //////////////////////////////////////////////////////////////
// RMT tx init
static void rmt_tx_init()
{
  rmt_config_t rmt_tx;
  rmt_tx.channel = RMT_TX_CHANNEL;
  rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
  rmt_tx.mem_block_num = 1;
  rmt_tx.clk_div = RMT_CLK_DIV;
  rmt_tx.tx_config.loop_en = false;
  rmt_tx.tx_config.carrier_duty_percent = 50;
  // Carrier Frequency of the IR receiver
  rmt_tx.tx_config.carrier_freq_hz = 38000;
  rmt_tx.tx_config.carrier_level = 1;
  rmt_tx.tx_config.carrier_en = 1;
  // Never idle -> aka ontinuous TX of 38kHz pulses
  rmt_tx.tx_config.idle_level = 1;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_tx.rmt_mode = 0;
  rmt_config(&rmt_tx);
  rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init()
{
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init()
{
  gpio_pad_select_gpio(LEDPIN);
  gpio_set_direction(LEDPIN, GPIO_MODE_OUTPUT);
}

// Button interrupt init
static void button_init()
{
  gpio_config_t io_conf;
  //interrupt of rising edge
  io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
  //bit mask of the pins, use GPIO4 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
  gpio_intr_enable(GPIO_INPUT_IO_1);
  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);
  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  //start gpio task
}

////////////////////////////////////////////////////////////////////////////////

// Tasks ///////////////////////////////////////////////////////////////////////
// Send task -- sends payload | Start | myID | Start | myID
static void send_task()
{
  // while (1)
  // {
  printf("Sent payload\n");
  char *data_out = (char *)malloc(len_out);
  xSemaphoreTake(mux, portMAX_DELAY);
  data_out[0] = start;
  data_out[1] = (char)(myID + 'j');
  data_out[2] = (char)(Xint + 'j');
  data_out[3] = (char)(Yint + 'j');
  data_out[4] = (char)(Zint + 'j');
  data_out[5] = genCheckSum(data_out, len_out - 1);
  printf("data: %s\n", data_out);
  ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

  uart_write_bytes(UART_NUM_1, data_out, len_out);
  xSemaphoreGive(mux);

  vTaskDelay(10 / portTICK_PERIOD_MS);
  // }
}

// Button task -- rotate through myIDs
void button_task()
{
  uint32_t io_num;
  while (1)
  {
    if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
    {
      printf("Button pressed.\n");
      send_task();
      led_task();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// LED task to light LED based on traffic state
void led_task()
{

  gpio_set_level(LEDPIN, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gpio_set_level(LEDPIN, 1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gpio_set_level(LEDPIN, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
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

// static int convert(int val)
// {
//   if (val > 9)
//     val = 9;
//   else if (val < 9 && val > 6)
//     val = 6;
//   else if (val < 6 && val > 3)
//     val = 3;
//   else if (val < 3 && val > 0)
//     val = 0;
//   else if (val < 0 && val > -3)
//     val = -3;
//   else if (val < -3 && val > -6)
//     val = -6;
//   else if (val < -6)
//     val = -9;

//   return val;
// }

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343()
{
  printf("\n>> Polling ADAXL343\n");
  while (1)
  {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    // calcRP(xVal, yVal, zVal);
    Xint = (int)xVal;
    Yint = (int)yVal;
    Zint = (int)zVal;

    printf("X: %d, Y: %d, Z: %d \n", Xint, Yint, Zint);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void app_main()
{

  // Mutex for current values when sending
  mux = xSemaphoreCreateMutex();

  // Create a FIFO queue for timer-based events
  timer_queue = xQueueCreate(10, sizeof(timer_event_t));

  // Create task to handle timer-based events
  //xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

  // Initialize all the things
  rmt_tx_init();
  uart_init();
  led_init();
  // Routine
  i2c_master_init();
  i2c_scanner();
  Accel_init();
  //alarm_init();
  button_init();

  // Create tasks for receive, send, set gpio, and button
  // Create task to poll ADXL343
  xTaskCreate(test_adxl343, "test_adxl343", 4096, NULL, 5, NULL);
  // xTaskCreate(recv_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
  // xTaskCreate(send_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
  // xTaskCreate(led_task, "set_traffic_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
  //xTaskCreate(id_task, "set_id_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(button_task, "button_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}
