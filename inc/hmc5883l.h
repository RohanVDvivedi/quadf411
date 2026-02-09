#ifndef HMC5883L_H
#define HMC5883L_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<quatro3d/quatro.h>
#include<cutlery/dpipe.h>

typedef enum hmc5883l_state hmc5883l_state;
enum hmc5883l_state
{
	HMC5883L_NOT_STARTED = 0, // initial state
	HMC5883L_READ_QUEUED = 1,
	HMC5883L_READ_IN_PROGRESS = 2,
	HMC5883L_READ_COMPLETE = 3,
	HMC5883L_WAIT_FOR_PERIOD = 4,
};

typedef struct hmc5883l hmc5883l;
struct hmc5883l
{
	I2C_HandleTypeDef* hi2c;

	uint8_t i2c_addr;

	hmc5883l_state state; // current sensor state

	dpipe* i2c_queue; // disable interrupts before accessing this

	uint32_t last_read_in_millis;

	uint32_t read_period_in_millis; // period in millis to be allowed to read the next record

	uint8_t read_buffer[2 * 3 * 2];

	vector data;
};

// initialize the module and makes sure it is present on the bus
// returns true if present on the bus
int init_hmc5883l(hmc5883l* mod_magn, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t period_in_millis);

// call on i2c txcplt
int maybe_data_ready_hmc5883l(hmc5883l* mod_magn);

// returns only if new data is available, else returns NULL
// pushes a new read to dpipe's queue if period has elapsed
// if the top of the dpipe's queue is it's own request, it schedules its IO over state machine 
vector get_hmc5883l(hmc5883l* mod_magn, int* new_data_arrived);

#endif