/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
//firmware version is added and 0x99 is added
#include <stdio.h>  // printf

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "gpio.h"
#include "uart.h"

/**
 * TO USE CONSOLE OUTPUT (PRINTF) IF NOT PRESENT ON YOUR PLATFORM
 */
// Macro to pack a 4-byte value into a buffer in big-endian format
#define PACK_4BYTE_VALUE(buffer, index, value) do { \
    buffer[index] = (value >> 24) & 0xFF; \
    buffer[index + 1] = (value >> 16) & 0xFF; \
    buffer[index + 2] = (value >> 8) & 0xFF; \
    buffer[index + 3] = value & 0xFF; \
} while (0)

#define TEMPERATURE_INDEX 2
#define HUMIDITY_INDEX 6

//static uint8_t TxData[4]= {0x07,0x04,0x03,0x05};
#define SENSIRION_I2C_HAL_SLEEP_MS 500
#define DEFAULT_DELAY_FACTOR 50
#define BUFFER_SIZE 12
char bufferText[100]; //
unsigned char buffer[BUFFER_SIZE];
#define RESPONSE_DELAY 500
void setbuffers(char *s, char t, int x) {
	int i = 0;
	for (i = 0; i < x; i++)
		s[i] = t;
}

// Define the packet types
#define PACKET_START_1 0x5A
#define PACKET_START_2 0xA5

#define SENSOR_STATE 0
#define DISPLAY_STATE 1
#define IDLE_STATE 2

// Function to send a packet through UART
void sendPacket(uint8_t command, uint16_t data) {
	// Create the buffer
	uint8_t buffer[8];

	// Prepare the packet
	buffer[0] = PACKET_START_1;
	buffer[1] = PACKET_START_2;
	buffer[2] = 0x05;
	buffer[3] = 0x82;
	buffer[4] = command;
	buffer[5] = 0x00;
	buffer[6] = (data >> 8) & 0xFF;
	buffer[7] = data & 0xFF;

	// Send the packet through UART
	UART_Send(buffer, sizeof(buffer));
	HAL_Delay(RESPONSE_DELAY);
}

uint32_t i = 0;
#define FIRMWAREVERSION "STM32_TEMP_HUM_CO2_VER1_REV1\n"
int app_main(void) {
	int16_t error = 0;
	uint16_t co2;
	int32_t temperature;
	int32_t humidity;

	UART_Init();
	UART_Send(FIRMWAREVERSION, sizeof(FIRMWAREVERSION));
	sensirion_i2c_hal_init();

	// Clean up potential SCD40 states
	scd4x_wake_up();
	scd4x_stop_periodic_measurement();
	scd4x_reinit();

	uint16_t serial_0;
	uint16_t serial_1;
	uint16_t serial_2;
	error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
	if (error) {

	} else {

	}

	// Start Measurement

	error = scd4x_start_periodic_measurement();
	if (error) {

	}

	for (;;) {

		// Read Measurement
		sensirion_i2c_hal_sleep_usec(100000);
		//sensirion_i2c_hal_sleep_usec(100000);
		//HAL_Delay(SENSIRION_I2C_HAL_SLEEP_MS);
		HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin | TX_EN_Pin, GPIO_PIN_SET);
		bool data_ready_flag = false;
		error = scd4x_get_data_ready_flag(&data_ready_flag);
		if (error) {
			continue;
		}
		if (!data_ready_flag) {
			continue;
		}

		error = scd4x_read_measurement(&co2, &temperature, &humidity);
		if (error) {

		} else if (co2 == 0) {

		} else {

			// Assuming 2 bytes for CO2, 4 bytes for temperature, and 4 bytes for humidity
			//co2/=100;//ansari
			// Pack the data into the buffer (big-endian format for simplicity)

			// Now, send the buffer through UART using HAL_UART_Transmit_IT
			HAL_GPIO_WritePin(GPIOA, TX_EN_Pin, GPIO_PIN_RESET);
			switch (i) {
			case SENSOR_STATE: {
				//HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin | TX_EN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, GPIO_PIN_RESET);
///////////////////////////////////////co2
				HAL_Delay(RESPONSE_DELAY);
				buffer[0] = 0x5a;
				buffer[1] = 0xa5;
				buffer[2] = 0x05;
				buffer[3] = 0x82;

				buffer[4] = 0x20;
				buffer[5] = 0x0;
				buffer[6] = (co2 >> 8) & 0xFF;
				buffer[7] = co2 & 0xFF;

				UART_Send(buffer, 8);

				//UART_Send(buffer,sizeof(buffer));
				HAL_Delay(RESPONSE_DELAY);
////////////////////////////////////////////temperature
				//5A A5 05 82 10 00 00 65

				buffer[0] = 0x5a;
				buffer[1] = 0xa5;
				buffer[2] = 0x05;
				buffer[3] = 0x82;

				buffer[4] = 0x10;
				buffer[5] = 0x0;
				buffer[6] = (temperature >> 8) & 0xFF;
				buffer[7] = temperature & 0xFF;

				UART_Send(buffer, 8);         //sizeof(buffer));
				HAL_Delay(RESPONSE_DELAY);

///////////////////////////////////////HUMIDITY

				buffer[0] = 0x5a;
				buffer[1] = 0xa5;
				buffer[2] = 0x05;
				buffer[3] = 0x82;

				buffer[4] = 0x15;
				buffer[5] = 0x0;
				buffer[6] = (humidity >> 8) & 0xFF;  // Humidity - Low-high byte
				buffer[7] = humidity & 0xFF;

				UART_Send(buffer, 8);
				//////////////////
				HAL_Delay(RESPONSE_DELAY);
				///FOLLOWING IS TE SECTION OF CODE TO READ THE SET VALUES OF TEMPERATURE,CO2 AND HUMIDITY..
////5A A5 03 83 25 00== GET TEMPERATURE
				i++;
				break;
			}

			case DISPLAY_STATE: {
				HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, GPIO_PIN_SET);
				buffer[0] = 0x5a;
				buffer[1] = 0xa5;
				buffer[2] = 0x04;
				buffer[3] = 0x83;
				buffer[4] = 0x25;
				buffer[5] = 0x00;
				buffer[6] = 0x01;
				UART_Send(buffer, 7);
				//expected reply=>{5A}{A5}{06}{83}{25}{00}{01}{00}{16}=0x16=22 temp
				/////////////////////////////////////////

				HAL_Delay(RESPONSE_DELAY);
				//5A A5 03 83 30 00== GET co2
				buffer[0] = 0x5a;
				buffer[1] = 0xa5;
				buffer[2] = 0x04;
				buffer[3] = 0x83;
				buffer[4] = 0x30;
				buffer[5] = 0x00;
				buffer[6] = 0x01;
				UART_Send(buffer, 7);
				//expected reply=>{5A}{A5}{06}{83}{30}{00}{01}{00}{14}///0x14=20 co2
				/////////////////////////////////////////

				HAL_Delay(RESPONSE_DELAY);
				//5A A5 03 83 35 00== GET co2
				buffer[0] = 0x5a;
				buffer[1] = 0xa5;
				buffer[2] = 0x04;
				buffer[3] = 0x83;
				buffer[4] = 0x35;
				buffer[5] = 0x00;
				buffer[6] = 0x01;
				//expected reply=>	{5A}{A5}{06}{83}{35}{00}{01}{4E}{20}==>4e20=20000
				UART_Send(buffer, 7);
				HAL_Delay(RESPONSE_DELAY);
				i++;
				break;
			}
			case IDLE_STATE: {
				HAL_Delay(RESPONSE_DELAY/20);
								buffer[0] = 0x99;
								buffer[1] = 0x99;
								buffer[2] = 0x99;
								buffer[3] = 0x99;
								buffer[4] = 0x99;
								buffer[5] = 0x99;
								UART_Send(buffer, 6);
								HAL_Delay(RESPONSE_DELAY/5);
												HAL_GPIO_WritePin(GPIOA, TX_EN_Pin, GPIO_PIN_RESET);
				for (int j = 0; j < DEFAULT_DELAY_FACTOR; j++)
							{
							HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, GPIO_PIN_RESET);
							HAL_Delay(RESPONSE_DELAY/8);
							HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, GPIO_PIN_SET);
							HAL_Delay(RESPONSE_DELAY/16);
							}


				HAL_Delay(RESPONSE_DELAY);
				HAL_Delay(RESPONSE_DELAY);
				i++;
				break;
			}
			default: {
				HAL_GPIO_WritePin(GPIOA, TX_EN_Pin, GPIO_PIN_RESET);
				for (int j = 0; j < DEFAULT_DELAY_FACTOR; j++)
				{
				HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, GPIO_PIN_RESET);
				HAL_Delay(RESPONSE_DELAY/4);
				HAL_GPIO_WritePin(GPIOA, LED_OUT_Pin, GPIO_PIN_SET);
				HAL_Delay(RESPONSE_DELAY/2);
				}
				i = 0;
			}

			}

		}
	}

	return 0;
}
