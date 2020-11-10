/*
Authored by Vivek Cherian
09-22-2020
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>
#include "esp_vfs_dev.h"

#define BLINK_GPIO 13
#define ECHO_UART_PORT_NUM      0
#define BUF_SIZE (1024)

void app_main()
{
    gpio_reset_pin(BLINK_GPIO);

    gpio_pad_select_gpio(BLINK_GPIO);

    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BLINK_GPIO);

    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,256, 0, 0, NULL, 0) );

    esp_vfs_dev_uart_use_driver(UART_NUM_0);


    int mode = 0;
    char c;
    int num=0;
    int len=0;
    int led_status=0;
    char input[BUF_SIZE];
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while(1)
    {

        switch(mode)
        {
            case 0:
                printf("Press 't' to toggle LED. Press 's' to switch modes.\n");
                while(1)
                {
                    
                    len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
                    if (len>0)
                    {
                        if (*data=='s')
                        {
                            printf("Switching modes...\n\n");
                            mode = (mode+1)%3;
                            break;
                        }
                        else if (*data=='t')
                        {
                            led_status^=1;
                            gpio_set_level(BLINK_GPIO,led_status);
                        }
                    }
                }
                
                
                break;
            case 1:
                while(1)
                {
                    printf("Input characters and hit 'Enter' to echo to console. Type 's' and hit 'Enter' to switch modes.\n");
                    scanf("%[^\n]s",input);
                    while((c = getchar()) != '\n' && c != EOF); // flush rest of input
                    if (input[0]=='s' && input[1]=='\0')
                    {
                        printf("Switching modes...\n\n");
                        mode = (mode+1)%3;
                        break;
                    }
                    else if (input[0]!='\n')
                    {
                        printf("Echo: %s\n",input);
                    }
                }
                
                break;
            case 2:
                while(1)
                {
                    printf("Enter a decimal number to convert to hex. Type 's' and hit 'Enter' to switch modes.\n");
                    scanf("%s",input);
                    if (input[0]=='s' && input[1]=='\0')
                    {
                        printf("Switching modes...\n\n");
                        mode = (mode+1)%3;
                        break;
                    }
                    else
                    {
                        int i=0, invalid = 0;
                        for (i=0; i<strlen(input); i++)
                        {
                            if (input[i]<48 || input[i]>57)
                            {
                                invalid=1;
                            }
                        }
                        if (invalid==1)
                        {
                            printf("Received invalid input '%s'. Please enter an integer.\n",input);
                        }              
                        else 
                        {
                            sscanf(input, "%d",&num);
                            printf("Recieved %s. In hex: 0x%x\n",input, num);
                        }
                    }
                }
                
                break;
        }
    }
    
}