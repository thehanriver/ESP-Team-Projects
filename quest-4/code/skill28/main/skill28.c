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
#include "./ADXL343.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "soc/rmt_reg.h"
#include "sdkconfig.h"

//IP defines
#define HOST_IP_ADDR "192.168.1.111"
#define PORT 1234

//all esps and their IP
//create array ESP_IPs with the myid -1 as index
#define ESP1_IP // myID = 1
#define ESP2_IP // myID = 2
#define ESP3_IP // myID = 3
#define ESP4_IP // myID = 4
#define ESP5_IP // myID = 5
#define ESP6_IP // myID = 6
#define PORT 1234
#define maxID 6

// LED Output pins definitions
#define BLUEPIN 14
#define GREENPIN 32
#define REDPIN 15
#define ONBOARD 13

// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 36 // A4 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1 4 //Button pin A5
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1

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


// Default ID/color
#define ID 1
#define COLOR 'R'

// States
#define ELECTION 0
#define WAIT_OK 1
#define WAIT_WIN 2
#define LEADER 3
#define NOT_LEADER 4

// signals
#define OK_SIGNAL '0'
#define E_SIGNAL '1'
#define WIN_SIGNAL '2'

// Signal flag
static int OK = 0;
static int WIN = 0;
static int E = 0;



static int leaderID = -1;


//Wifi
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static const char *TAG = "Client";
static const char *payload = "Hello from the other siiiiiddddddeeeee";

static int timer;

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char)ID;
char myColor = (char)COLOR;
int len_out = 4;

static int myState = START;

//array of all IPs;
static char IPs[6][13] = {ESP1_IP, ESP2_IP, ESP3_IP, ESP4_IP, ESP5_IP, ESP6_IP};

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct
{
    int flag; // flag for enabling stuff in timer task
} timer_event_t;

// A simple structure to pass "events" to main task
typedef struct
{
    int from; // id that the message is from
    char message[3];
} message_t;

// System tags
static const char *TAG_SYSTEM = "system"; // For debug logs

// // Button interrupt handler -- add to queue
// static void IRAM_ATTR gpio_isr_handler(void *arg)
// {
//     uint32_t gpio_num = (uint32_t)arg;
//     xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para)
{

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // blue is shorter
    if (myColor == 'G')
    {
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
    }
    else
    {
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    }

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Utility  Functions //////////////////////////////////////////////////////////

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

// Init Functions //////////////////////////////////////////////////////////////

//For IR
// // RMT tx init
// static void rmt_tx_init()
// {
//     rmt_config_t rmt_tx;
//     rmt_tx.channel = RMT_TX_CHANNEL;
//     rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
//     rmt_tx.mem_block_num = 1;
//     rmt_tx.clk_div = RMT_CLK_DIV;
//     rmt_tx.tx_config.loop_en = false;
//     rmt_tx.tx_config.carrier_duty_percent = 50;
//     // Carrier Frequency of the IR receiver
//     rmt_tx.tx_config.carrier_freq_hz = 38000;
//     rmt_tx.tx_config.carrier_level = 1;
//     rmt_tx.tx_config.carrier_en = 1;
//     // Never idle -> aka ontinuous TX of 38kHz pulses
//     rmt_tx.tx_config.idle_level = 1;
//     rmt_tx.tx_config.idle_output_en = true;
//     rmt_tx.rmt_mode = 0;
//     rmt_config(&rmt_tx);
//     rmt_driver_install(rmt_tx.channel, 0, 0);
// }

// // Configure UART
// static void uart_init()
// {
//     // Basic configs
//     uart_config_t uart_config = {
//         .baud_rate = 1200, // Slow BAUD rate
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
//     uart_param_config(UART_NUM_1, &uart_config);

//     // Set UART pins using UART0 default pins
//     uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

//     // Reverse receive logic line
//     uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

//     // Install UART driver
//     uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
// }

// GPIO init for LEDs
static void led_init()
{
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

//Configure timer
static void alarm_init()
{
    // Select and initialize basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    
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
// // Send task -- sends payload | Start | myID | Start | myID
// static void send_task(char signal)
// {

//     printf("Sent payload\n");
//     char *data_out = (char *)malloc(len_out);
//     xSemaphoreTake(mux, portMAX_DELAY);
//     data_out[0] = start;
//     data_out[1] = signal; 
//     data_out[2] = (char)myID;
//     data_out[3] = genCheckSum(data_out, len_out - 1);
//     ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

//     uart_write_bytes(UART_NUM_1, data_out, len_out);
//     xSemaphoreGive(mux);

// }

// Receives task -- looks for Start byte then stores received values
void recv_task()
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
                if (checkCheckSum(data_in, len_out))
                {
                    ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
                    printf("Received data_in: %u\n", data_in[1]);
                    switch (data_in[1])
                    {
                    case 82:
                        myColor = 'R';
                        break;
                    case 71:
                        myColor = 'G';
                        break;
                    case 66:
                        myColor = 'B';
                        break;
                    }
                }
            }
        }
        else
        {
            //printf("Nothing received.\n");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

// Button task -- rotate through myIDs
void button_task()
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            xSemaphoreTake(mux, portMAX_DELAY);

            xSemaphoreGive(mux);
            printf("Button pressed.\n");
            send_task();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// LED task to light LED based on traffic state
void led_task()
{
    while (1)
    {
        switch (myState)
        {
        case START:
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
        case LEADER: // Green
            gpio_set_level(GREENPIN, 1);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
        case LEADERNT: // blue
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 1);
            // printf("Current state: %c\n",status);
            break;
        case ELECTION: // red
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 1);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
        case WAIT: // red & blue
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 1);
            gpio_set_level(BLUEPIN, 1);
            // printf("Current state: %c\n",status);
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Timer task -- R (10 seconds), G (10 seconds), Y (2 seconds)
// static void timer_evt_task(void *arg)
// {
//   while (1)
//   {
//     // Create dummy structure to store structure from queue
//     timer_event_t evt;

//     // Transfer from queue
//     xQueueReceive(timer_queue, &evt, portMAX_DELAY);

//     // Do something if triggered!
//     if (evt.flag == 1)
//     {
//       printf("Action!\n");
//       if (myColor == 'R')
//       {
//         myColor = 'G';
//       }
//       else if (myColor == 'G')
//       {
//         myColor = 'B';
//       }
//       else if (myColor == 'B')
//       {
//         myColor = 'R';
//       }
//     }
//   }
// }

// LED task to blink onboard LED based on ID
void id_task()
{
    while (1)
    {
        for (int i = 0; i < (int)myID; i++)
        {
            gpio_set_level(ONBOARD, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(ONBOARD, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//UDP client
static void udp_client_task(char[] esp_ip)
{
    char rx_buffer[4];
    char host_ip[] = esp_ip;
    int addr_family = 0;
    int ip_protocol = 0;


        char addr_str[128];
        int addr_family;
        int ip_protocol;

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(esp_ip);
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

        
        char buffer2[128];
        int status;
        status = sprintf(buffer2, "%d:%d", myID, signal);
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
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
            ESP_LOGI(TAG, "%s", rx_buffer);

            if (rx_buffer[2] == '0')
            {
                OK = 1;
            }
            else if (rx_buffer[2] == '1')
            {
                E = 1;
            }
            else if (rx_buffer[2] == '2')
            {
                leaderID = rx_buffer[0];
                WIN = 1;
            }
        } 

    if (sock != -1)
    {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}

void voting_fsm()
{
    int delay_count = 0;    // number of delays that has occurred; inc before each vTaskDelay, set to 0 when switching states
    int state = ELECTION;
    while (1)
    {
        switch (state)
        {
            case ELECTION:

                // send OK to lower ids
                int id = myID - 1;
                for (id; id>0; i--)
                {
                    send_signal(IPs[i-1],OK_SIGNAL);
                }

                // send E to higher ids
                id = myID+1;
                for (id; id<maxID; id++)
                {
                    send_signal(IPs[i-1],E_SIGNAL);
                }
                // switch to WAIT_OK state, reset delay count
                state = WAIT_OK;
                delay_count = 0; 
                break;

            case WAIT_OK:
                // send OK to lower ids while waiting for OK 
                int id = myID - 1;
                for (id; id>0; i--)
                {
                    send_signal(IPs[i-1],OK_SIGNAL);
                }

                // If OK is received, switch to WAIT_WIN
                if (OK = 1)
                {
                    OK = 0;
                    delay_count = 0;
                    state = WAIT_WIN;
                }

                // if timeout occurs, switch to LEADER
                if (delay_count == 10)  // set to appropriate value
                {
                    delay_count = 0;
                    state = LEADER
                }
                delay_count++;
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case WAIT_WIN:
                // if WIN is received, switch to NOT_LEADER
                if (WIN = 1)
                {
                    WIN=0; // reset
                    delay_count = 0;
                    state = NOT_LEADER;
                }
                // if timeout occurs, switch to ELECTION
                if (delay_count == 10)  // set to appropriate value
                {
                    delay_count = 0;
                    state = ELECTION
                }
                delay_count++;
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case LEADER:

                // Send WIN to lower
                int id = myID - 1;
                for (id; id>0; i--)
                {
                    send_signal(IPs[i-1],WIN_SIGNAL);
                }

                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case NOT_LEADER:

                // if WIN is received, reset and stay in NOT_LEADER
                if (WIN = 1)
                {
                    WIN = 0;
                }
                // if OK is received switch to WAIT_WIN
                else if (OK==1)
                {
                    delay_count = 0;
                    state = WAIT_WIN;
                }
                // if neither is received, check if second highest. If so, become LEADER. If not, move to ELECTION
                else if (myID == leaderID-1)
                {
                    delay_count = 0;
                    state = LEADER;
                }
                else
                {
                    delay_count = 0;
                    state = ELECTION;
                }


                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
        }

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

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Create task to handle timer-based events
    //xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initialize all the things
    rmt_tx_init();
    uart_init();
    led_init();
    //alarm_init();
    button_init();

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    // Create Task to print out values received
    xTaskCreate(recv_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(send_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(id_task, "set_id_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_task, "button_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}
