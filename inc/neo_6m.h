#ifndef NEO_6M_H
#define NEO_6M_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<stdint.h>

#include<cutlery/dpipe.h>

typedef struct neo_6m_data neo_6m_data;
struct neo_6m_data
{
	// position data
	double latitude;  // in degreen (-90 to +90)
	double longitude; // in degrees (-180 to 180)

	// altitude data
	double altitude;  // in meters

	// velocity data
	double speed;     // in meters/second
	double heading;   // in degreed (0 to 360) from north in clockwise direction
};

#include<cutlery/value_arraylist.h>

data_definitions_value_arraylist(buffer_read_sizes, uint32_t)
declarations_value_arraylist(buffer_read_sizes, uint32_t, static inline) // last parameter can be empty
#define EXPANSION_FACTOR 1.5
function_definitions_value_arraylist(buffer_read_sizes, uint32_t, static inline) // last parameter can be empty

#define BUFFER_BYTES 256

typedef struct neo_6m neo_6m;
struct neo_6m
{
	uint8_t received_byte; // per byte receive buffer

	neo_6m_data gps_data;

	uint32_t last_read_in_millis;

	dpipe unparsed_bytes;
	buffer_read_sizes unparsed_sentence_sizes;

	uint8_t unparsed_bytes_buffer[BUFFER_BYTES];
	uint32_t unparsed_sentence_sizes_buffer[BUFFER_BYTES / 3]; // over capacity

	UART_HandleTypeDef* huart;
};

void init_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart);

// to be called asynchronously in interrupt
void accept_byte_for_neo_6m(neo_6m* mod_neo_6m);

// if there is new data available it is all parsed and latest one is returned
neo_6m_data get_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart_temp, int* new_position_arrived, int* new_altitude_arrived, int* new_velocity_arrived);

#endif