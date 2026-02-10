#include<ms5611.h>

int init_ms5611(ms5611* mod_baro, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue)
{
	mod_baro->hi2c = hi2c;
	mod_baro->i2c_addr = i2c_addr;
	mod_baro->state = MS5611_NOT_STARTED;
	mod_baro->state_data_type = 1;
	mod_baro->i2c_queue = i2c_queue;
	mod_baro->last_read_in_millis = HAL_GetTick();
	mod_baro->read_period_in_millis = 10;

	if(HAL_I2C_IsDeviceReady(hi2c, i2c_addr << 1, 3, 100) != HAL_OK)
		return 0;

	// read prom
	for(int i = 1; i <= 6; i++)
	{
		uint8_t buffer[2];
		HAL_I2C_Mem_Read(hi2c, (mod_baro->i2c_addr) << 1, 0xa0 + i * 2, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);
		mod_baro->C[i] = (((uint16_t)buffer[0]) << 8) | ((uint16_t)buffer[1]);
	}

	return 1;
}

int maybe_data_ready_ms5611(ms5611* mod_baro)
{
	if(mod_baro->state != MS5611_COMMAND_IN_PROGRESS && mod_baro->state != MS5611_READ_IN_PROGRESS)
		return 0;
	uint8_t buffer[1];
	cy_uint bytes_read = peek_from_dpipe(mod_baro->i2c_queue, buffer, 1, ALL_OR_NONE);
	if(bytes_read > 0 && buffer[0] == mod_baro->i2c_addr)
	{
		read_from_dpipe(mod_baro->i2c_queue, buffer, 1, ALL_OR_NONE);
		mod_baro->state++; // change to it's corresponding completed state
		return 1;
	}
	return 0;
}

double get_ms5611(ms5611* mod_baro, int* new_data_arrived)
{
	switch(mod_baro->state)
	{
		case MS5611_NOT_STARTED :
		{
			// queue read request to dpipe
			{
				__disable_irq();
					write_to_dpipe(mod_baro->i2c_queue, &(mod_baro->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_baro->state = MS5611_COMMAND_QUEUED;
				mod_baro->state_data_type = 1;
			}

			break;
		}
		case MS5611_COMMAND_QUEUED :
		{
			uint8_t buffer[1];
			__disable_irq();
				cy_uint bytes_read = peek_from_dpipe(mod_baro->i2c_queue, buffer, 1, ALL_OR_NONE);
			__enable_irq();
			if(bytes_read > 0 && buffer[0] == mod_baro->i2c_addr)
			{
				// we are at the top of the queue, so schedule this read
				HAL_I2C_Master_Transmit_IT(mod_baro->hi2c, (mod_baro->i2c_addr) << 1, ((uint8_t[]){0x08 + ((0x03 + mod_baro->state_data_type) << 4)}), 1);
				mod_baro->state = MS5611_COMMAND_IN_PROGRESS;
			}

			break;
		}
		case MS5611_COMMAND_IN_PROGRESS :
		{
			// do nothing
			break;
		}
		case MS5611_COMMAND_COMPLETED :
		{
			// record the millis when last read was done by the user
			mod_baro->last_read_in_millis = HAL_GetTick();

			mod_baro->state = MS5611_WAIT_FOR_PERIOD;
			break;
		}
		case MS5611_WAIT_FOR_PERIOD :
		{
			// queue read request to dpipe, if the period to wait for has elapsed
			if(HAL_GetTick() >= mod_baro->last_read_in_millis + mod_baro->read_period_in_millis)
			{
				__disable_irq();
					write_to_dpipe(mod_baro->i2c_queue, &(mod_baro->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_baro->state = MS5611_READ_QUEUED;
			}
			break;
		}
		case MS5611_READ_QUEUED :
		{
			uint8_t buffer[1];
			__disable_irq();
				cy_uint bytes_read = peek_from_dpipe(mod_baro->i2c_queue, buffer, 1, ALL_OR_NONE);
			__enable_irq();
			if(bytes_read > 0 && buffer[0] == mod_baro->i2c_addr)
			{
				HAL_I2C_Master_Receive_IT(mod_baro->hi2c, (mod_baro->i2c_addr) << 1, &(mod_baro->read_buffer_D[mod_baro->state_data_type][0]), 3);
				mod_baro->state = MS5611_READ_IN_PROGRESS;
			}
			break;
		}
		case MS5611_READ_IN_PROGRESS :
		{
			// do nothing
			break;
		}
		case MS5611_READ_COMPLETED :
		{
			// queue read request to dpipe
			{
				__disable_irq();
					write_to_dpipe(mod_baro->i2c_queue, &(mod_baro->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_baro->state = MS5611_COMMAND_QUEUED;
				mod_baro->state_data_type = (3 - mod_baro->state_data_type);
			}
			break;
		}
	}

	(*new_data_arrived) = 0;
	return 0;
}