#ifndef MS5611_H
#define MS5611_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<cutlery/dpipe.h>

typedef enum ms5611_state ms5611_state;
enum ms5611_state
{
	MS5611_NOT_STARTED = 0, // initial state
	MS5611_READ_QUEUED = 1,
	MS5611_READ_IN_PROGRESS = 2,
	MS5611_READ_COMPLETE = 3,
	MS5611_WAIT_FOR_PERIOD = 4,
};

typedef struct ms5611 ms5611;
struct ms5611
{
	I2C_HandleTypeDef* hi2c;

	uint8_t i2c_addr;

	ms5611_state state; // current sensor state

	dpipe* i2c_queue; // disable interrupts before accessing this

	uint16_t C[6]; // PROM data

	uint32_t last_read_in_millis;

	uint32_t read_period_in_millis; // period in millis to be allowed to read the next record

	uint8_t read_D1[3];
	uint8_t read_D2[3];

	double data;
};

// initialize the module and makes sure it is present on the bus
// returns true if present on the bus
int init_ms5611(ms5611* mod_baro, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t period_in_millis);

// call on i2c txcplt
void maybe_data_ready_ms5611(ms5611* mod_baro);

// returns only if new data is available, else returns NULL
// pushes a new read to dpipe's queue if period has elapsed
// if the top of the dpipe's queue is it's own request, it schedules its IO over state machine 
double get_ms5611(ms5611* mod_baro, int* new_data_arrived);


#endif