#ifndef NEO_6M_H
#define NEO_6M_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<stdint.h>

#include<cutlery/dpipe.h>

typedef struct neo_6m_data neo_6m_data;
struct neo_6m_data
{
	double latitude;
	double longitude;
	double altitude;
};

#define BUFFER_BYTES 2048

typedef struct neo_6m neo_6m;
struct neo_6m
{
	uint8_t received_byte; // per byte receive buffer

	neo_6m_data gps_data;

	uint32_t last_read_in_millis;

	dpipe unparsed_bytes;

	uint8_t unparsed_bytes_buffer[BUFFER_BYTES];

	UART_HandleTypeDef* huart;
};

void init_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart);

// to be called asynchronously in interrupt
void accept_byte_for_neo_6m(neo_6m* mod_neo_6m);

// if there is new data available it is all parsed and latest one is returned
neo_6m_data get_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart_temp, int* new_data_arrived);

#endif