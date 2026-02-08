#include<adxl345.h>

int init_adxl345(adxl345* mod_accl, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t range_in_gs, uint32_t period_in_millis);

void maybe_data_ready_adxl345(adxl345* mod_accl)
{
	uint8_t buffer[1];
	cy_uint bytes_read = peek_from_dpipe(mod_accl->i2c_queue, buffer, 1, ALL_OR_NONE);
	if(bytes_read > 0 && buffer[0] == mod_accl->i2c_addr)
	{
		read_from_dpipe(mod_accl->i2c_queue, buffer, 1, ALL_OR_NONE);
		mod_accl->state = ADXL345_READ_COMPLETE;
	}
}

vector get_adxl345(adxl345* mod_accl, int* new_data_arrived)
{
	switch(mod_accl->state)
	{
		case ADXL345_NOT_STARTED :
		{
			// queue read request to dpipe
			{
				__disable_irq();
					write_to_dpipe(mod_accl->i2c_queue, &(mod_accl->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_accl->state = ADXL345_READ_QUEUED;
			}

			break;
		}
		case ADXL345_READ_QUEUED :
		{
			uint8_t buffer[1];
			__disable_irq();
				cy_uint bytes_read = peek_from_dpipe(mod_accl->i2c_queue, buffer, 1, ALL_OR_NONE);
			__enable_irq();
			if(bytes_read > 0 && buffer[0] == mod_accl->i2c_addr)
			{
				// we are at the top of the queue, so schedule this read
				HAL_I2C_Mem_Read_IT(
					mod_accl->hi2c,
					(mod_accl->i2c_addr) << 1,
					0x32, // register address
					I2C_MEMADD_SIZE_8BIT,
					mod_accl->read_buffer,
					6
				);
				mod_accl->state = ADXL345_READ_IN_PROGRESS;
			}

			break;
		}
		case ADXL345_READ_IN_PROGRESS : // user application can not do anything there, this transition is done by the maybe_data_ready_adxl345()
		{
			break;
		}
		case ADXL345_READ_COMPLETE :
		{
			// transform read_buffer -> data in vector
			mod_accl->data.xi = (int16_t)((((int16_t)(mod_accl->read_buffer[1]))<<8) | ((int16_t)(mod_accl->read_buffer[0])));
			mod_accl->data.yj = (int16_t)((((int16_t)(mod_accl->read_buffer[3]))<<8) | ((int16_t)(mod_accl->read_buffer[2])));
			mod_accl->data.zk = (int16_t)((((int16_t)(mod_accl->read_buffer[5]))<<8) | ((int16_t)(mod_accl->read_buffer[4])));

			// record the millis when last read was done by the user
			mod_accl->last_read_in_millis = HAL_GetTick();

			// transition state to waiting
			mod_accl->state = ADXL345_WAIT_FOR_PERIOD;

			// prepare return values and return
			(*new_data_arrived) = 1;
			return mod_accl->data;
		}
		case ADXL345_WAIT_FOR_PERIOD :
		{
			if(HAL_GetTick() >= mod_accl->last_read_in_millis + mod_accl->read_period_in_millis)
			{
				// queue read request to dpipe
				__disable_irq();
					write_to_dpipe(mod_accl->i2c_queue, &(mod_accl->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_accl->state = ADXL345_READ_QUEUED;
			}

			break;
		}
	}

	(*new_data_arrived) = 0;
	return (vector){};
}