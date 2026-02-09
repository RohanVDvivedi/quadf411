#ifndef ITG3205_H
#define ITG3205_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<quatro3d/quatro.h>
#include<cutlery/dpipe.h>

typedef enum itg3205_state itg3205_state;
enum itg3205_state
{
	ITG3205_NOT_STARTED = 0, // initial state
	ITG3205_READ_QUEUED = 1,
	ITG3205_READ_IN_PROGRESS = 2,
	ITG3205_READ_COMPLETE = 3,
	ITG3205_WAIT_FOR_PERIOD = 4,
};

typedef struct itg3205 itg3205;
struct itg3205
{
	I2C_HandleTypeDef* hi2c;

	uint8_t i2c_addr;

	itg3205_state state; // current sensor state

	dpipe* i2c_queue; // disable interrupts before accessing this

	uint32_t last_read_in_millis;

	uint32_t read_period_in_millis; // period in millis to be allowed to read the next record

	uint8_t read_buffer[2 * 3 * 2];

	vector data;
};

// initialize the module and makes sure it is present on the bus
// returns true if present on the bus
int init_itg3205(itg3205* mod_gyro, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t period_in_millis);

// call on i2c txcplt
int maybe_data_ready_itg3205(itg3205* mod_gyro);

// returns only if new data is available, else returns NULL
// pushes a new read to dpipe's queue if period has elapsed
// if the top of the dpipe's queue is it's own request, it schedules its IO over state machine 
vector get_itg3205(itg3205* mod_gyro, int* new_data_arrived);

#endif