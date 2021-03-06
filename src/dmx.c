//#include <stdint.h>
//#include <stdio.h>
#include <string.h>
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include <dmx.h>

#define DMX_SERIAL_INPUT_PIN    16          // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   17          // pin for dmx tx
#define DMX_UART_NUM            UART_NUM_2  // dmx uart

#define HEALTHY_TIME            500         // timeout in ms 

#define BUF_SIZE                1024        //  buffer size for rx events

#define DMX_IDLE                    0
#define DMX_BREAK                   1
#define DMX_DATA                    2
#define DMX_BREAK_480               3


QueueHandle_t dmx_rx_queue;

SemaphoreHandle_t sync_dmx;

uint8_t dmx_state = DMX_IDLE;

uint16_t current_rx_addr = 0;

long last_dmx_packet = 0;

uint8_t dmx_data[1024];

int curbuf = 0;

void Initialize()
{
    // configure UART for DMX
    uart_config_t uart_config =
    {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(DMX_UART_NUM, &uart_config);

    // Set pins for UART
    uart_set_pin(DMX_UART_NUM, DMX_SERIAL_OUTPUT_PIN, DMX_SERIAL_INPUT_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // install queue
    uart_driver_install(DMX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &dmx_rx_queue, 0);

    // create receive task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    // create mutex for syncronisation
    sync_dmx = xSemaphoreCreateMutex();
}

uint8_t Read(uint16_t channel)
{
    // restrict acces to dmx array to valid values
    if(channel < 1)
    {
        channel = 1;
    }
    else if(channel > 512)
    {
        channel = 512;
    }
    // take data threadsafe from array and return
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
    uint8_t tmp_dmx = dmx_data[channel];
    xSemaphoreGive(sync_dmx);
    return tmp_dmx;
}

uint8_t IsHealthy()
{
    // get timestamp of last received packet
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
    long dmx_timeout = last_dmx_packet;
    xSemaphoreGive(sync_dmx);
    // check if elapsed time < defined timeout
    //if(esp_timer_get_time() - dmx_timeout < HEALTHY_TIME)
    if(xTaskGetTickCount() - dmx_timeout < HEALTHY_TIME)
    {
        return 1;
    }
    return 0;
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    int read_bytes = 0;
    for(;;)
    {
        // wait for data in the dmx_queue
        if(xQueueReceive(dmx_rx_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            //memset(dtmp, 0, (int)BUF_SIZE);
            bzero(dtmp, BUF_SIZE);
            //printf("%d,%d\n",event.type, event.size);
            switch(event.type)
            {
                case UART_DATA:
                    //printf("%d\n", event.size);
                    //uart_get_buffered_data_len(DMX_UART_NUM, (size_t*)&curbuf);
                    //printf("UART_Break curbuf: %d\n", curbuf);

                    // read the received data
                    uart_read_bytes(DMX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    // check if break detected
                    if(dmx_state == DMX_BREAK)
                    {
                        // if not 0, then RDM or custom protocol
                        if(dtmp[0] == 0)
                        {
                            dmx_state = DMX_DATA;
                            // reset dmx adress to 0
                            current_rx_addr = 0;
                            xSemaphoreTake(sync_dmx, portMAX_DELAY);
                            // store received timestamp
                            last_dmx_packet = esp_timer_get_time();
                            xSemaphoreGive(sync_dmx);
                        }
                    }
                    // check if in data receive mode, or get remaining
                    if((dmx_state == DMX_DATA) || (dmx_state == DMX_BREAK_480))
                    {
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
                        // copy received bytes to dmx data array
                        for(int i = 0; i < event.size; i++)
                        {
                        dmx_data[current_rx_addr++] = dtmp[i];
                        }
                        xSemaphoreGive(sync_dmx);
                        if(dmx_state == DMX_BREAK_480)
                        {
                            dmx_state = DMX_DATA;
                        }
                    }
                    //printf("curren_rx_addr: %d\n", current_rx_addr);
                    //printf("-----------\n");
                break;
                case UART_BREAK:
                    //if current address less than 512
                    //assume more left in buffer
                    //else flush and start from
                    // clear queue und flush received bytes
                    if(current_rx_addr <512 )
                    {
                        dmx_state = DMX_BREAK_480;
                    }else
                    {
                        uart_flush_input(DMX_UART_NUM);
                        xQueueReset(dmx_rx_queue);
                        dmx_state = DMX_BREAK;
                    }
		        break;                
                case UART_FRAME_ERR:
                    printf("frame\n");
                    break;
                case UART_PARITY_ERR:
                    printf("parity\n");
                    break;
                case UART_BUFFER_FULL:
                    printf("buff\n");
                    break;
                case UART_FIFO_OVF:
                    printf("fifo\n");
                    break;
                default:
                // error recevied, going to idle mode
                uart_flush_input(DMX_UART_NUM);
                xQueueReset(dmx_rx_queue);
                dmx_state = DMX_IDLE;
                break;
            }
        }
    }
}
