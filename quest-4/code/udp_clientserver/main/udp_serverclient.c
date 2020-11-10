#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "math.h"

// UDP
#define ID 1
#define DEVICE_NUM 3
#define BUF_SIZE (1024)

#define CLIENT_INPUT 15 //pin 15

// LED Output pins definitions
//#define BLUEPIN   14
//#define GREENPIN  32
//#define REDPIN    15

char ip_addrs[DEVICE_NUM + 1][20] = {
    "0",
    "192.168.1.106", //ID 1 //COM4
    "192.168.1.100", //ID 2 //COM5
    "192.168.1.130", //ID 3 //COM6
};

int port = 3333;

char start = 0x1B;
char myID = (char)ID;
static const char *TAG = "Quest 4";
static const char payload[35] = "Payload Message from ID 1 :";
static const char *resp = "This is response from ID 1 ";

int sockets[DEVICE_NUM + 1];
struct sockaddr_in addrs[DEVICE_NUM + 1];
//int current_state = RESET;
//int current_status = RESET_STATUS;
//int current_leader = 0;

// GPIO init for LEDs
/*
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
}
*/

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
        dest_addr_ip4->sin_port = htons(port);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        /* Bind to socket */
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", port);

        while (1)
        {

            ESP_LOGI(TAG, "Waiting for data");
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
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                int err = sendto(sock, resp, BUF_SIZE, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                printf("Response from %s: resp\n", addr_str);
                printf("%s\n", rx_buffer);
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

/* Client task, i.e. sending task */
static void udp_client_task()
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *data_out = (uint8_t *)malloc(BUF_SIZE);

    for (int i = 1; i <= DEVICE_NUM; i++)
    {

        /* Skips configuring socket for itself */
        if (i == ID)
            continue;

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(ip_addrs[i]);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        printf("Socket created, sending to %s:%d\n", ip_addrs[i], port);
        struct timeval read_timeout;
        read_timeout.tv_sec = 0;
        read_timeout.tv_usec = 100000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
        sockets[i] = sock;
        addrs[i] = dest_addr;
    }

    for (int j = 1; j <= DEVICE_NUM; j++)
    {
        if (j != ID)
        {
            int err = sendto(sockets[j], payload, BUF_SIZE, 0, (struct sockaddr *)&(addrs[j]), sizeof(addrs[j]));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        }
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    //free(data_out);
    //free(data_in);
    vTaskDelete(NULL);
}

static void Toggle_client()
{
    while (1)
    {
        if (gpio_get_level(CLIENT_INPUT))
        {
            printf("Button pressed\n");
            udp_client_task();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    /* Set up UDP */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    //led_init();

    gpio_set_direction(CLIENT_INPUT, GPIO_MODE_INPUT);

    //xTaskCreate(led, "led", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    //xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    xTaskCreate(Toggle_client, "Toggle_client", 4096, NULL, 7, NULL);

    //xTaskCreate(logic_task, "logic_task", 4096, NULL, configMAX_PRIORITIES - 4, NULL);

    //alarm_init();
}