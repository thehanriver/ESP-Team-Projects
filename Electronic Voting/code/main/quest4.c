//Quest 4
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

//
//CHANGE ID HERE
//
// Default ID/color
#define FLASH_THIS_ID 6   //1,2,3,4,5
#define DEFAULT_COLOR '3' // RED
#define MAX_ID 6

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
#define UART_RX_GPIO_NUM 36 // A4
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1_VOTE 4  //Button pin A5 //Button for changing vote
#define GPIO_INPUT_IO_2_SEND 39 //Button pin A3 // Button for sending IR Task
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1_VOTE

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

// #define UDP_TIMEOUT_SECONDS 3
#define UDP_PORT 1111

// States
#define ELECTION 0
#define WAIT_OK 1
#define WAIT_WIN 2
#define LEADER 3
#define NOT_LEADER 4

// signals
#define OKIE_SIGNAL 0
#define E_SIGNAL 1
#define WIN_SIGNAL 2
#define R_SIGNAL 3
#define G_SIGNAL 4
#define B_SIGNAL 5

// Signal flag
static int OKIE = 0;
static int WIN = 0;
static int E = 0;

static int leaderID = -1;
static int state = NOT_LEADER;

//Wifi
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static const char *TAG = "Quest 4";

static int timer;
static int rec_vote = 1;

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
int myID = FLASH_THIS_ID;
char myVote = DEFAULT_COLOR; // myVote is a char number
int len_out = 4;

char ip_addrs[MAX_ID + 1][20] = {
    "192.168.1.131", // vpi
    "192.168.1.101", //ID 1 //COM4
    "192.168.1.102", //ID 2 //COM5
    "192.168.1.103", //ID 3 //COM6
    "192.168.1.104", //ID 4 //COM11
    "192.168.1.105", //ID 5 //COM10
    "192.168.1.106"  //ID 6 //COM9
};

int sockets[MAX_ID + 1];
struct sockaddr_in addrs[MAX_ID + 1];

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;

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

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// //ISR handler
// void IRAM_ATTR timer_group0_isr(void *para)
// {

//     // Prepare basic event data, aka set flag
//     timer_event_t evt;
//     evt.flag = 1;

//     // blue is shorter
//     if (myVote == 'G')
//     {
//         timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
//     }
//     else
//     {
//         timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
//     }

//     // Clear the interrupt, Timer 0 in group 0
//     TIMERG0.int_clr_timers.t0 = 1;

//     // After the alarm triggers, we need to re-enable it to trigger it next time
//     TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

//     // Send the event data back to the main program task
//     xQueueSendFromISR(timer_queue, &evt, NULL);
// }

static void udp_server_task(void *pvParameters);
static void udp_client_fn(int fromID, int targetID, int signal);

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
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_INPUT_IO_2_SEND, GPIO_MODE_INPUT);
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
    gpio_intr_enable(GPIO_INPUT_IO_1_VOTE);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1_VOTE, gpio_isr_handler, (void *)GPIO_INPUT_IO_1_VOTE);
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
/* Server task, i.e. receiving task */
static void udp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    //uint8_t* data_in = (uint8_t*)malloc(BUF_SIZE);
    //uint8_t* data_out = (uint8_t*)malloc(BUF_SIZE);
    //rx_buffer[len] = 0;
    char rx_buffer[128];
    printf("Started udp server\n");

    while (1)
    {

        /* Create socket */
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(UDP_PORT);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket in server: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        /* Bind to socket */
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, UDP_PORT %d", UDP_PORT);

        while (1)
        {

            // ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
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
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET)
                {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                }

                else if (source_addr.sin6_family == PF_INET6)
                {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                // ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                // ESP_LOGI(TAG, "%s", rx_buffer);
                // printf("0:%c 1:%c 2:%c", rx_buffer[0], rx_buffer[1], rx_buffer[2]);

                char resp[] = "ACK";
                int err = sendto(sock, resp, strlen(resp), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending in server: errno %d", errno);
                    break;
                }
                //printf("Recieved: %s\n", rx_buffer);
                if (rx_buffer[2] == '0')
                {
                    OKIE = 1;
                }
                else if (rx_buffer[2] == '1')
                {
                    E = 1;
                }
                else if (rx_buffer[2] == '2')
                {
                    // printf("rx_buffer: %d\n", (int)rx_buffer[0]);
                    leaderID = (int)rx_buffer[0];
                    WIN = 1;
                }
                else if (rx_buffer[2] == '3') // R_SIGNAL as character
                {
                    udp_client_fn(((int)rx_buffer[0] - 48), 0, R_SIGNAL);
                }
                else if (rx_buffer[2] == '4') // G_SIGNAL as character
                {
                    udp_client_fn(((int)rx_buffer[0] - 48), 0, G_SIGNAL);
                }
                else if (rx_buffer[2] == '5') // B_SIGNAL as character
                {
                    udp_client_fn(((int)rx_buffer[0] - 48), 0, B_SIGNAL);
                }
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    //free(data_in);
    //free(data_out);
    vTaskDelete(NULL);
}

/* Client function, i.e. sending task */
static void udp_client_fn(int fromID, int targetID, int signal)
{
    switch (signal)
    {
    case OKIE_SIGNAL:
        // printf("sending OKIE to %d\n", targetID);
        break;
    case E_SIGNAL:
        // printf("sending E to %d\n", targetID);
        break;
    case WIN_SIGNAL:
        // printf("sending WIN to %d\n", targetID);
        break;
    case R_SIGNAL:
        printf("sending R to %d\n", targetID);
        break;
    case G_SIGNAL:
        printf("sending G to %d\n", targetID);
        break;
    case B_SIGNAL:
        printf("sending B to %d\n", targetID);
        break;
    }
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    /* Skips configuring socket for itself */
    if (targetID == FLASH_THIS_ID)
    {
        return;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(ip_addrs[targetID]);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket in client: errno %d", errno);
        return;
    }
    //printf("Socket created, sending to %s:%d\n", ip_addrs[targetID], UDP_PORT);
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    char payload[BUF_SIZE];
    snprintf(payload, BUF_SIZE, "%d:%d", fromID, signal);

    if (signal > 2)
    {
        printf("payload %s \n", payload);
    }
    int err1 = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    if (err1 < 0)
    {
        ESP_LOGE(TAG, "Error occurred while making UDP socket non-blocking: errno %d", errno);
        shutdown(sock, 1);
        close(sock);
        return;
    }

    int err2 = sendto(sock, payload, BUF_SIZE, 0, (struct sockaddr *)&(dest_addr), sizeof(dest_addr));
    if (err2 < 0)
    {
        //ESP_LOGE(TAG, "Error occurred during sending in client: errno %d", errno);
        shutdown(sock, 1);
        close(sock);
        return;
    }
    shutdown(sock, 1);
    close(sock);
}

// Send task -- sends payload | Start | myVote | myID | checkSum
static void ir_send_task() // send myID and myVote
{

    printf("Sent payload\n");
    char *data_out = (char *)malloc(len_out);
    xSemaphoreTake(mux, portMAX_DELAY);
    data_out[0] = start;
    data_out[1] = (char)(myID + '0');
    data_out[2] = myVote;
    data_out[3] = genCheckSum(data_out, len_out - 1);
    ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

    uart_write_bytes(UART_NUM_1, data_out, len_out);
    xSemaphoreGive(mux);
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
                if (checkCheckSum(data_in, len_out))
                {
                    ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
                    printf("Received data_in from myid: %c  vote: %c\n", data_in[1], data_in[2]);
                    rec_vote = 0;
                    gpio_set_level(GREENPIN, 0);
                    gpio_set_level(REDPIN, 0);
                    gpio_set_level(BLUEPIN, 0);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    switch (data_in[2]) // fix to temporarily display received vote
                    {
                    case '3': // red
                        gpio_set_level(GREENPIN, 0);
                        gpio_set_level(REDPIN, 1);
                        gpio_set_level(BLUEPIN, 0);
                        // printf("Current state: %c\n",status);
                        break;
                    case '4': // Green
                        gpio_set_level(GREENPIN, 1);
                        gpio_set_level(REDPIN, 0);
                        gpio_set_level(BLUEPIN, 0);
                        // printf("Current state: %c\n",status);
                        break;
                    case '5': // blue
                        gpio_set_level(GREENPIN, 0);
                        gpio_set_level(REDPIN, 0);
                        gpio_set_level(BLUEPIN, 1);
                        // printf("Current state: %c\n",status);
                    }
                    int from_id = (int)(data_in[1] - '0'); // convert char number to int number
                    int vote = (int)(data_in[2] - '0');    // convert char number to int number
                    // printf("Before udp call: from_id %d, leaderID %d, vote %d", from_id, leaderID, vote);
                    int temp_leader_id = leaderID;
                    if (from_id > 10)
                    {
                        from_id -= 48;
                    }
                    if (leaderID > 10)
                    {
                        temp_leader_id -= 48;
                    }

                    if (state == LEADER)
                    {
                        udp_client_fn(from_id, 0, vote);
                    }
                    else
                    {
                        udp_client_fn(from_id, temp_leader_id, vote);
                    }
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    rec_vote = 1;
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
void change_vote()
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            xSemaphoreTake(mux, portMAX_DELAY);
            if (myVote == '3') //r
            {
                myVote = '4';
                printf("Went from red to green\n");
            }
            else if (myVote == '4') //g
            {
                myVote = '5';
                printf("Went from green to blue\n");
            }
            else if (myVote == '5') //b
            {
                myVote = '3';
                printf("Went from blue to red\n");
            }
            xSemaphoreGive(mux);
            printf("Button pressed.\n");
            // ir_send_task();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
static void send_vote()
{
    while (1)
    {
        if (!gpio_get_level(GPIO_INPUT_IO_2_SEND))
        {
            printf("Sending vote");
            ir_send_task();
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
// LED task to light LED based on vote
void led_task()
{
    while (1)
    {
        if (rec_vote)
        {
            switch (myVote)
            {
            case '3': // red
                gpio_set_level(GREENPIN, 0);
                gpio_set_level(REDPIN, 1);
                gpio_set_level(BLUEPIN, 0);
                // printf("Current state: %c\n",status);
                break;
            case '4': // Green
                gpio_set_level(GREENPIN, 1);
                gpio_set_level(REDPIN, 0);
                gpio_set_level(BLUEPIN, 0);
                // printf("Current state: %c\n",status);
                break;
            case '5': // blue
                gpio_set_level(GREENPIN, 0);
                gpio_set_level(REDPIN, 0);
                gpio_set_level(BLUEPIN, 1);
                // printf("Current state: %c\n",status);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

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

void switch_state(int *state_ptr, int to_state, int *delay_count_ptr)
{
    *state_ptr = to_state;
    *delay_count_ptr = 0;
    OKIE = 0;
    WIN = 0;
    E = 0;
}

void voting_fsm_task()
{
    int delay_count = 0; // number of delays that has occurred; inc before each vTaskDelay, set to 0 when switching states
    state = NOT_LEADER;
    int id; // used for loops
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Starting fsm...\n");
    while (1)
    {
        switch (state)
        {
        case ELECTION:

            // printf("entered ELECTION\n");

            // reset extraneous signals
            if (E)
            {
                E = 0; // don't print attention here because this is normal
            }
            if (OKIE) // switch to WAIT_WIN bc a higher esp is taking care of it
            {
                OKIE = 0;
                // printf("Attention: OKIE signal received in ELECTION state. Moving to WAIT_WIN\n");
                switch_state(&state, WAIT_WIN, &delay_count);
                break;
            }
            if (WIN)
            {
                WIN = 0;
                // printf("Attention: WIN signal received in ELECTION state. Moving to NOT_LEADER.\n");
                switch_state(&state, NOT_LEADER, &delay_count);
                break;
            }

            // send OKIE to lower ids
            for (id = myID - 1; id > 0; id--)
            {
                udp_client_fn(myID, id, OKIE_SIGNAL);
            }

            // send E to higher ids
            for (id = myID + 1; id <= MAX_ID; id++)
            {
                udp_client_fn(myID, id, E_SIGNAL);
            }

            // switch to WAIT_OK state, reset delay count
            switch_state(&state, WAIT_OK, &delay_count);
            break;

        case WAIT_OK:
            if (delay_count == 0)
            {
                //printf("entered WAIT_OK\n");
            }

            // reset extraneous signals
            if (E)
            {
                E = 0; // not printing attention cuz this is normal
            }
            if (WIN)
            {
                WIN = 0;
                // printf("Attention: WIN signal received in WAIT_OK state. Moving to NOT_LEADER.\n");
                switch_state(&state, NOT_LEADER, &delay_count);
                break;
            }

            // send OKIE to lower ids while waiting for OKIE
            for (id = myID - 1; id > 0; id--)
            {
                udp_client_fn(myID, id, OKIE_SIGNAL);
            }

            // If OKIE is received, switch to WAIT_WIN
            if (OKIE)
            {
                OKIE = 0;
                switch_state(&state, WAIT_WIN, &delay_count);
                break;
            }

            // if timeout occurs, switch to LEADER
            if (delay_count == 10) // set to appropriate value
            {
                switch_state(&state, LEADER, &delay_count);
                break;
            }
            delay_count++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;

        case WAIT_WIN:
            if (delay_count == 0)
            {
                // printf("entered WAIT_WIN\n");
            }

            // reset extraneous signals
            if (OKIE)
            {
                OKIE = 0; // don't print attention this is normal
            }

            // if E is received, another election was started, switch to ELECTION
            if (E)
            {
                E = 0;
                switch_state(&state, ELECTION, &delay_count);
                break;
            }

            // if WIN is received, switch to NOT_LEADER
            if (WIN)
            {
                WIN = 0; // reset
                switch_state(&state, NOT_LEADER, &delay_count);
                break;
            }
            // if timeout occurs in 20 delay_counts, switch to ELECTION
            if (delay_count == 20) // set to appropriate value
            {
                switch_state(&state, ELECTION, &delay_count);
                break;
            }
            delay_count++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;

        case LEADER:
            leaderID = myID;
            if (delay_count == 0)
            {
                // printf("entered LEADER\n");
            }

            // reset extraneous signals, and print 'error'
            if (E)
            {
                E = 0;
                // printf("Attention: E signal received in LEADER state\n");
            }

            if (WIN)
            {
                WIN = 0;
                // printf("Attention: WIN signal received in LEADER state. Moving to NOT_LEADER.\n");
                switch_state(&state, NOT_LEADER, &delay_count);
                break;
            }

            // if OKIE is received, it means a new ESP with a higher id entered, didn't received a WIN, so it started an election and sent OKIE's to lower id's
            // switch to WAIT_WIN
            if (OKIE)
            {
                OKIE = 0;
                switch_state(&state, WAIT_WIN, &delay_count);
                break;
            }

            // Send WIN to lower
            for (id = myID - 1; id > 0; id--)
            {
                udp_client_fn(myID, id, WIN_SIGNAL);
            }

            if (delay_count % 10 == 0)
            {
                // printf("I am the leader.\n");
            }
            delay_count++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;

        case NOT_LEADER:
            if (delay_count == 0)
            {
                // printf("entered NOT_LEADER\n");
            }

            if (E)
            {
                E = 0;
                switch_state(&state, ELECTION, &delay_count);
                break;
            }

            // if WIN is received, reset and stay in NOT_LEADER
            if (WIN)
            {
                WIN = 0;
                delay_count = 0;
                break; // because we stay in same state don't reset delay_count
            }
            // if OKIE is received switch to WAIT_WIN
            if (OKIE)
            {
                OKIE = 0;
                switch_state(&state, WAIT_WIN, &delay_count);
                break;
            }
            // if neither is received in 8 delay counts, the leader has died. check if second highest. If so, become LEADER. If not, move to ELECTION
            if (delay_count >= 8)
            {
                // set OKIE to lower id's
                for (id = myID - 1; id > 0; id--)
                {
                    udp_client_fn(myID, id, OKIE_SIGNAL);
                }
                if (myID == leaderID - 1)
                {
                    switch_state(&state, LEADER, &delay_count);
                    break;
                }
                else
                {
                    switch_state(&state, ELECTION, &delay_count);
                    break;
                }
            }

            delay_count++;
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

    // Create task to handle timer-based events
    //xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initialize all the things
    rmt_tx_init();
    uart_init();
    led_init();
    //alarm_init();
    button_init();

    // Create Task to print out values received
    xTaskCreate(ir_receive_task, "uart_rx_task", 1024 * 4, NULL, 5, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024 * 2, NULL, 3, NULL);
    xTaskCreate(id_task, "set_id_task", 1024 * 2, NULL, 5, NULL);
    xTaskCreate(change_vote, "change_vote", 1024 * 2, NULL, 5, NULL);
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    xTaskCreate(voting_fsm_task, "voting_fsm", 4096, NULL, 5, NULL);
    xTaskCreate(send_vote, "send_vote", 1024 * 2, NULL, 5, NULL);
}
