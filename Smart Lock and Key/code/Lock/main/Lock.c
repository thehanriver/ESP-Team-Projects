//Quest 6
//Mario Han, Vivek Cherian, Hussain Valiuddin
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <lwip/netdb.h>
#include <sys/param.h>

#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/rmt.h"
#include "driver/adc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_types.h"
#include "esp_attr.h"
#include "esp_adc_cal.h"
#include "esp_vfs_dev.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "soc/rmt_reg.h"
#include "sdkconfig.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// LED Output pins definitions
#define LEDPIN 32
#define PASSWORDPIN 14

// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 4  // A5
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_BUTTON 36 //Button pin A4 //Button for changing vote
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_BUTTON

//Wifi defines
#define EXAMPLE_ESP_WIFI_SSID "Group_2"
#define EXAMPLE_ESP_WIFI_PASS "1GBSt0rage!"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_2_SEC (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload

//UDP
#define HOST_IP_ADDR "192.168.1.111" //"192.168.1.139"
#define PORT 1234

#define SERVO_MIN_PULSEWIDTH 700  //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2100 //Maximum pulse width in microsecond

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

static const uint16_t alphafonttable[128] = {

    0b0000000000000000, 0b0000000000000010, 0b0000000000000100,
    0b0000000000001000, 0b0000000000010000, 0b0000000000100000,
    0b0000000001000000, 0b0000000010000000, 0b0000000100000000,
    0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
    0b0001000000000000, 0b0010000000000000, 0b0100000000000000,
    0b1000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0001001011001001, 0b0001010111000000, 0b0001001011111001,
    0b0000000011100011, 0b0000010100110000, 0b0001001011001000,
    0b0011101000000000, 0b0001011100000000,
    0b0000000000000000, //
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0000000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000000000, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111};

static uint16_t displaybuffer[4];

//Wifi
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static const char *TAG = "Quest 6";
static const char *payload = "Default payload";

// static int timer;

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
static int set_password = 0;
int open_servo = 0;
int len_out = 6;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;

// System tags
static const char *TAG_SYSTEM = "system"; // For debug logs

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init()
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
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n\n");
    }

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

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
        printf("- No I2C devices found!"
               "\n");
    printf("\n");
}

uint16_t ASCII_to_Alphanum(uint8_t ascii_in)
{
    return (alphafonttable[ascii_in]);
};

////////////////////////////////////////////////////////////////////////////////

// Init Functions //////////////////////////////////////////////////////////////

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12); //Set GPIO 12 as PWM0A, to which servo is connected
}

//  For IR
// RMT tx init
static void
rmt_tx_init()
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
    gpio_pad_select_gpio(PASSWORDPIN);
    gpio_pad_select_gpio(GPIO_BUTTON);

    gpio_set_direction(LEDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PASSWORDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
}

// Button interrupt init
// static void button_init()
// {
//     gpio_config_t io_conf;
//     //interrupt of rising edge
//     io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
//     //bit mask of the pins, use GPIO4 here
//     io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
//     //set as input mode
//     io_conf.mode = GPIO_MODE_INPUT;
//     //enable pull-up mode
//     io_conf.pull_up_en = 1;
//     gpio_config(&io_conf);
//     gpio_intr_enable(GPIO_BUTTON);
//     //install gpio isr service
//     gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
//     //hook isr handler for specific gpio pin
//     gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void *)GPIO_BUTTON);
//     //create a queue to handle gpio event from isr
//     gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
//     //start gpio task
// }

static void mcpwm_task_init()
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
}

static void alphanum_init()
{
    displaybuffer[0] = 0b0000000000000000; // 0
    displaybuffer[1] = 0b0000000000000000; // 0
    displaybuffer[2] = 0b0000000000000000; // 0
    displaybuffer[3] = 0b0000000000000000; // 0
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

// Tasks ///////////////////////////////////////////////////////////////////////

void mcpwm_example_servo_control()
{
    uint32_t count;

    for (count = SERVO_MIN_PULSEWIDTH; count < SERVO_MAX_PULSEWIDTH; count += 50)
    {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    for (count = SERVO_MAX_PULSEWIDTH; count > SERVO_MIN_PULSEWIDTH; count -= 50)
    {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, count);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/* Client function, i.e. sending task */
static void udp_client_fn(int key, int x, int y, int z)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;

    char addr_str[128];
    int addr_family;
    int ip_protocol;

    gpio_set_level(LEDPIN, 1);

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
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    char buffer2[128];
    snprintf(buffer2, BUF_SIZE, "%d,%d,%d,%d,%d", set_password, key, x, y, z);
    payload = buffer2;
    printf("set_password = %d key = %d, x = %d, y = %d, z = %d \n", set_password, key, x, y, z);

    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
    ESP_LOGI(TAG, "Message sent");

    struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

    // Error occurred during receiving
    if (len < 0)
    {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    }
    // Data received
    else
    {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
        ESP_LOGI(TAG, "%s", rx_buffer);
        open_servo = atoi(rx_buffer);
        if (open_servo == 1)
        {
            printf("password correct!!\n");
            displaybuffer[0] = ASCII_to_Alphanum('O'); //
            displaybuffer[1] = ASCII_to_Alphanum('P'); //
            displaybuffer[2] = ASCII_to_Alphanum('E'); //
            displaybuffer[3] = ASCII_to_Alphanum('N'); //
            mcpwm_example_servo_control();
        }
        if (open_servo == 0)
        {
            printf("Wrong password!!\n");
            displaybuffer[0] = ASCII_to_Alphanum('N'); //
            displaybuffer[1] = ASCII_to_Alphanum('O'); //
            displaybuffer[2] = ASCII_to_Alphanum('P'); //
            displaybuffer[3] = ASCII_to_Alphanum('E'); //
        }
        if (open_servo == 2)
        {
            printf("Password Set!!!\n");
            displaybuffer[0] = ASCII_to_Alphanum('S'); //
            displaybuffer[1] = ASCII_to_Alphanum('E'); //
            displaybuffer[2] = ASCII_to_Alphanum('T'); //
            displaybuffer[3] = ASCII_to_Alphanum('.'); //
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    displaybuffer[0] = 0b0000000000000000; // 0
    displaybuffer[1] = 0b0000000000000000; // 0
    displaybuffer[2] = 0b0000000000000000; // 0
    displaybuffer[3] = 0b0000000000000000; // 0
    gpio_set_level(LEDPIN, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    shutdown(sock, 0);
    close(sock);
}

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
        // printf("- brightness: max \n");
    }

    // Initiallize characters to buffer
    // alphanum_init();
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

        // displaybuffer[0] = alphafonttable[]);
        // displaybuffer[1] = alphafonttable[];                                          //
        // displaybuffer[2] = alphafonttable[(int)((measured_speed_m_per_s)*100) % 10];  //
        // displaybuffer[3] = alphafonttable[(int)((measured_speed_m_per_s)*1000) % 10]; //

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Receives task -- looks for Start byte then stores received values
void ir_receive_task()
{
    // Buffer for input data
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len_in > 0)
        {
            if (data_in[0] == start)
            {
                printf("Recieved a signal");
                if (checkCheckSum(data_in, len_out))
                {
                    ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
                    printf("Received data_in from Key: %c\n", data_in[1]);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    int key = (int)(data_in[1] - 'j');   // convert char number to int number
                    int x_val = (int)(data_in[2] - 'j'); // convert char number to int number
                    int y_val = (int)(data_in[3] - 'j');
                    int z_val = (int)(data_in[4] - 'j');

                    printf("key = %d, x = %d, y = %d, z = %d \n", key, x_val, y_val, z_val);
                    udp_client_fn(key, x_val, y_val, z_val);
                    gpio_set_level(LEDPIN, 0);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

// void set_password_fn()
// {
//     uint32_t io_num;
//     while (1)
//     {
//         if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
//         {
//             set_password = 1;
//             gpio_set_level(PASSWORDPIN, 1);
//             printf("Button pressed.\n");
//             // ir_send_task();
//             vTaskDelay(3000 / portTICK_PERIOD_MS);
//             set_password = 0;
//             gpio_set_level(PASSWORDPIN, 0);
//         }
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
// }

static void button_press()
{
    while (1)
    {
        if (gpio_get_level(GPIO_BUTTON))
        {
            gpio_set_level(PASSWORDPIN, 1);
            set_password = 1;
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
        set_password = 0;
        gpio_set_level(PASSWORDPIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
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
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create task to handle timer-based events
    //xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initialize all the things
    rmt_tx_init();
    uart_init();
    led_init();
    //alarm_init();
    // button_init();
    mcpwm_task_init();

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,
                                        256, 0, 0, NULL, 0));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    i2c_example_master_init();
    i2c_scanner();
    alphanum_init();

    // Create Task to print out values received
    xTaskCreate(ir_receive_task, "uart_rx_task", 1024 * 4, NULL, 5, NULL);
    xTaskCreate(button_press, "button_press", 1024 * 2, NULL, 5, NULL);
    xTaskCreate(alphanum_display, "alphanum_display", 1024 * 2, NULL, 5, NULL);
}
