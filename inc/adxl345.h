#ifndef ADXL345_H
#define ADXL345_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<quatro3d/quatro.h>
#include<cutlery/dpipe.h>

typedef enum adxl345_state adxl345_state;
enum adxl345_state
{
	ADXL345_NOT_STARTED, // initial state
	ADXL345_READ_QUEUED,
	ADXL345_READ_IN_PROGRESS,
	ADXL345_READ_COMPLETE,
	ADXL345_WAIT_FOR_PERIOD,
};

typedef struct adxl345 adxl345;
struct adxl345
{
	I2C_HandleTypeDef* hi2c;

	uint8_t i2c_addr;

	adxl345_state state; // current sensor state

	dpipe* i2c_queue; // disable interrupts before accessing this

	uint32_t last_read_in_millis;

	uint32_t read_period_in_millis; // period in millis to be allowed to read the next record

	uint8_t read_buffer[2 * 3 * 2];

	vector data;
};

// initialize the module and makes sure it is present on the bus
// returns true if present on the bus
int init_adxl345(adxl345* mod_accl, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t range_in_gs, uint32_t period_in_millis);

// call on i2c txcplt
void maybe_data_ready_adxl345(adxl345* mod_accl);

// returns only if new data is available, else returns NULL
// pushes a new read to dpipe's queue if period has elapsed
// if the top of the dpipe's queue is it's own request, it schedules its IO over state machine 
vector get_adxl345(adxl345* mod_accl, int* new_data_arrived);

#endif