/* Uart Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"

/**
 * This is a example which echos any data it receives on UART back to the sender using RS485 interface in half duplex mode.
*/
#define TAG "RS485_ECHO_APP"

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
#define ECHO_TEST_TXD   (CONFIG_ECHO_UART_TXD)
#define ECHO_TEST_RXD   (CONFIG_ECHO_UART_RXD)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (CONFIG_ECHO_UART_RTS)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)

#define BUF_SIZE        (127)
#define BAUD_RATE       (CONFIG_ECHO_UART_BAUD_RATE)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART_PORT          (CONFIG_ECHO_UART_PORT_NUM)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
// Function to process received packet
#define PACKET_START_1 0x5A
#define PACKET_START_2 0xA5
void processPacket(unsigned char* uart_data,unsigned int data_length)
{
   // int data_length = sizeof(uart_data);
    int i = 0;

    while (i < data_length) {
        if ((uart_data[i] == 0x5a) && (uart_data[i + 1] == 0xa5)) {
            switch (uart_data[i + 2]) {
                case 0x05: // Data Length = 5
                {
                    switch (uart_data[i + 3]) {
                        case 0x82: // Command = Display Write
                        {
                            switch (uart_data[i + 4]) {
                                case 0x20: // Humidity
                                    printf("Measured Humidity = %u\n", (((unsigned int)uart_data[i + 6]) << 8) | uart_data[i + 7]);
                                    i += 7;
                                    break;
                                case 0x10: // Temperature
                                    printf("Measured Temperature = %u\n", (((unsigned int)uart_data[i + 6]) << 8) | uart_data[i + 7]);
                                    i += 7;
                                    break;
                                case 0x15: // CO2
                                    printf("Measured CO2 = %u\n", (((unsigned int)uart_data[i + 6]) << 8) | uart_data[i + 7]);
                                    i += 7;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                        case 0x83: // Command = Display Read
                        {
                            switch (uart_data[i + 4]) {
                                case 0x25: // Preset Temperature
                                    printf("Preset Temperature = %u\n", (((unsigned int)uart_data[i + 6]) << 8) | uart_data[i + 7]);
                                    i += 7;
                                    break;
                                case 0x30: // Preset CO2
                                    printf("Preset CO2 = %u\n", (((unsigned int)uart_data[i + 6]) << 8) | uart_data[i + 7]);
                                    i += 7;
                                    break;
                                case 0x35: // Preset Humidity
                                    printf("Preset Humidity = %u\n", (((unsigned int)uart_data[i + 6]) << 8) | uart_data[i + 7]);
                                    i += 7;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                    }
                    break;
                }
                case 0x06: // Data Length = 6
                {
                    switch (uart_data[i + 3]) {
                        case 0x83: // Command = Display Read
                        {
                            switch (uart_data[i + 4]) {
                                case 0x25: // Preset Temperature
                                    printf("Preset Temperature = %u\n", (((unsigned int)uart_data[i + 7]) << 8) | uart_data[i + 8]);
                                    i += 8;
                                    break;
                                case 0x30: // Preset CO2
                                    printf("Preset CO2 = %u\n", (((unsigned int)uart_data[i + 7]) << 8) | uart_data[i + 8]);
                                    i += 8;
                                    break;
                                case 0x35: // Preset Humidity
                                    printf("Preset Humidity = %u\n", (((unsigned int)uart_data[i + 7]) << 8) | uart_data[i + 8]);
                                    i += 8;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }
        i++;
    }
}
void processPacket1(const uint8_t* packet) {
    // Check packet markers
	/////////////
//	{5A}{A5}{05}{82}{20}{00}{05}{51}{00}==>Command to update  Measured humidity=1361 from 0x2000, 0x5aa5 is header and 0x05 is the number of bytes, 0x82 is update command
//	{A5}{03}{82}{4F}{4B}==> acknowledge from the Display
//	{5A}{A5}{05}{82}{10}{00}{00}{19}{00} ==> Command to update Measured temperature=25 from 0x10000, 0x5aa5 is header and 0x05 is the number of bytes, 0x82 is update command
//	{A5}{03}{82}{4F}{4B} ==> acknowledge from the Display
//	{5A}{A5}{05}{82}{15}{00}{00}{34}{00}==> Measured co2=52 from 0x1500, 0x5aa5 is header and 0x05 is the number of bytes, 0x82 is update command
//	{A5}{03}{82}{4F}{4B}==>acknowledge from the Display
//	{5A}{A5}{04}{83}{25}{00}{01}==>Command to read the display preset temperature value from 0x2500 ,  0x5aa5 is header and 0x05 is the number of bytes, 0x82 is update command
//	{5A}{A5}{06}{83}{25}{00}{01}{00}{16} ==> response from the display that the temperature is 22, 0x5aa5 is header and 0x05 is the number of bytes, 0x82 is update command
//	{5A}{A5}{04}{83}{30}{00}{01} ==>Command to read the display preset co2 value from 0x3000
//	{5A}{A5}{06}{83}{30}{00}{01}{00}{14} ==>response from the display that the co2 is 20.
//	{5A}{A5}{04}{83}{35}{00}{01} ==> Command to read the display preset humidity value from 0x3500
//	{5A}{A5}{06}{83}{35}{00}{01}{4E}{20}==> response from the display that the humidity is 20000 ppm.
    if (packet[0] == PACKET_START_1 && packet[1] == PACKET_START_2) {
        // Extract the data based on the packet type (packet[4])
        switch (packet[4]) {
            case 0x20: // CO2 packet
                // Combine the bytes to get the CO2 value
                uint16_t co2 = (packet[6] << 8) | packet[7];
                printf("CO2=%u\n", co2);
                // Now you have the CO2 value (co2)
                // Do something with it
                break;

            case 0x10: // Temperature packet
                // Combine the bytes to get the temperature value
                int16_t temperature = (packet[6] << 8) | packet[7];
                printf("Temp=%d\n", temperature);
                // Now you have the temperature value (temperature)
                // Do something with it
                break;

            case 0x15: // Humidity packet
                // Combine the bytes to get the humidity value
                uint16_t humidity = (packet[6] << 8) | packet[7];
                printf("humidity=%d\n", humidity);
                // Now you have the humidity value (humidity)
                // Do something with it
                break;
            case 0x25: // Temperature Pre_set packet
                         // Combine the bytes to get the temperature pre_set value
            	 if(packet[2]==6){
                         int16_t temperature_pre_set = (packet[7] << 8) | packet[8];
                         printf("Pre_set_Temperature=%d\n", temperature_pre_set);
            	 }
                         // Now you have the temperature pre_set value (temperature_pre_set)
                         // Do something with it
                         break;

                     case 0x30: // CO2 Pre_set packet
                         // Combine the bytes to get the CO2 pre_set value
                    	 if(packet[2]==6){
                         uint16_t co2_pre_set = (packet[7] << 8) | packet[8];
                         printf("Pre_set_Co2=%u\n", co2_pre_set);
                    	 }
                         // Now you have the CO2 pre_set value (co2_pre_set)
                         // Do something with it
                         break;

                     case 0x35: // Humidity Pre_set packet
                         // Combine the bytes to get the humidity pre_set value
                    	 if(packet[2]==6){
                         uint16_t humidity_pre_set = (packet[7] << 8) | packet[8];
                         printf("Pre_set_Humidity=%d\n", humidity_pre_set);
                    	 }
                         // Now you have the humidity pre_set value (humidity_pre_set)
                         // Do something with it
                         break;

            default:
                // Invalid packet type, ignore or handle error
                break;
        }
    }
}

static void echo_send(const int port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

// An example of echo test with hardware flow control on UART
static void echo_task(void *arg)
{
    const int uart_num = ECHO_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));

    // Allocate buffers for UART
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
  ///  echo_send(uart_num, "Start RS485 UART test.\r\n", 24);

    while(1) {
        //Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);

        //Write data back to UART
        if (len > 0) {
           /// echo_send(uart_num, "\r\n", 2);
            ///char prefix[] = "RS485 Received: [";
          ///  echo_send(uart_num, prefix, (sizeof(prefix) - 1));
            ESP_LOGI(TAG, "Received %u bytes:", len);
            printf("[ ");
            for (int i = 0; i < len; i++) {
                printf("0x%.2X ", (uint8_t)data[i]);
             ///   echo_send(uart_num, (const char*)&data[i], 1);
                // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
                if (data[i] == '\r') {
                  ///  echo_send(uart_num, "\n", 1);
                }
            }
            printf("] \n");
            // Inside the main loop or the task that handles received UART data
            // data is the buffer containing the received packet (e.g., A9 4B DF 7E FF DB FF FF)
            processPacket(data,len);

         ///   echo_send(uart_num, "]\r\n", 3);
        } else {
            // Echo a "." to show we are alive while we wait for input
           /// echo_send(uart_num, ".", 1);
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    //A uart read/write example without event queue;
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
}
