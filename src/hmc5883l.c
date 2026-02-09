#include<hmc5883l.h>

int init_hmc5883l(hmc5883l* mod_magn, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t period_in_millis)
{
	mod_magn->hi2c = hi2c;
	mod_magn->i2c_addr = i2c_addr;
	mod_magn->state = HMC5883L_NOT_STARTED;
	mod_magn->i2c_queue = i2c_queue;
	mod_magn->last_read_in_millis = HAL_GetTick();
	mod_magn->read_period_in_millis = period_in_millis;

	if(HAL_I2C_IsDeviceReady(hi2c, i2c_addr << 1, 3, 100) != HAL_OK)
		return 0;

	// sample averaging, and output rate
	HAL_I2C_Mem_Write(hi2c, (mod_magn->i2c_addr) << 1, 0x00, I2C_MEMADD_SIZE_8BIT, ((uint8_t[]){0x78}), 1, HAL_MAX_DELAY);

	// range and gain setting, to suit earth
	HAL_I2C_Mem_Write(hi2c, (mod_magn->i2c_addr) << 1, 0x01, I2C_MEMADD_SIZE_8BIT, ((uint8_t[]){0x20}), 1, HAL_MAX_DELAY);

	// continuous sampling mode
	HAL_I2C_Mem_Write(hi2c, (mod_magn->i2c_addr) << 1, 0x02, I2C_MEMADD_SIZE_8BIT, ((uint8_t[]){0x00}), 1, HAL_MAX_DELAY);

	return 1;
}

int maybe_data_ready_hmc5883l(hmc5883l* mod_magn)
{
	uint8_t buffer[1];
	cy_uint bytes_read = peek_from_dpipe(mod_magn->i2c_queue, buffer, 1, ALL_OR_NONE);
	if(bytes_read > 0 && buffer[0] == mod_magn->i2c_addr)
	{
		read_from_dpipe(mod_magn->i2c_queue, buffer, 1, ALL_OR_NONE);
		mod_magn->state = HMC5883L_READ_COMPLETE;
		return 1;
	}
	return 0;
}

vector get_hmc5883l(hmc5883l* mod_magn, int* new_data_arrived)
{
	switch(mod_magn->state)
	{
		case HMC5883L_NOT_STARTED :
		{
			// queue read request to dpipe
			{
				__disable_irq();
					write_to_dpipe(mod_magn->i2c_queue, &(mod_magn->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_magn->state = HMC5883L_READ_QUEUED;
			}

			break;
		}
		case HMC5883L_READ_QUEUED :
		{
			uint8_t buffer[1];
			__disable_irq();
				cy_uint bytes_read = peek_from_dpipe(mod_magn->i2c_queue, buffer, 1, ALL_OR_NONE);
			__enable_irq();
			if(bytes_read > 0 && buffer[0] == mod_magn->i2c_addr)
			{
				// we are at the top of the queue, so schedule this read
				HAL_I2C_Mem_Read_IT(
					mod_magn->hi2c,
					(mod_magn->i2c_addr) << 1,
					0x03, // register address
					I2C_MEMADD_SIZE_8BIT,
					mod_magn->read_buffer,
					6
				);
				mod_magn->state = HMC5883L_READ_IN_PROGRESS;
			}

			break;
		}
		case HMC5883L_READ_IN_PROGRESS : // user application can not do anything there, this transition is done by the maybe_data_ready_hmc5883l()
		{
			break;
		}
		case HMC5883L_READ_COMPLETE :
		{
			// transform read_buffer -> data in vector
			// note: the byte ordering is not x,y,z
			mod_magn->data.xi = (int16_t)((((int16_t)(mod_magn->read_buffer[0]))<<8) | ((int16_t)(mod_magn->read_buffer[1])));
			mod_magn->data.zk = (int16_t)((((int16_t)(mod_magn->read_buffer[2]))<<8) | ((int16_t)(mod_magn->read_buffer[3])));
			mod_magn->data.yj = (int16_t)((((int16_t)(mod_magn->read_buffer[4]))<<8) | ((int16_t)(mod_magn->read_buffer[5])));

			// record the millis when last read was done by the user
			mod_magn->last_read_in_millis = HAL_GetTick();

			// transition state to waiting
			mod_magn->state = HMC5883L_WAIT_FOR_PERIOD;

			// prepare return values and return
			(*new_data_arrived) = 1;
			return mod_magn->data;
		}
		case HMC5883L_WAIT_FOR_PERIOD :
		{
			if(HAL_GetTick() >= mod_magn->last_read_in_millis + mod_magn->read_period_in_millis)
			{
				// queue read request to dpipe
				__disable_irq();
					write_to_dpipe(mod_magn->i2c_queue, &(mod_magn->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_magn->state = HMC5883L_READ_QUEUED;
			}

			break;
		}
	}

	(*new_data_arrived) = 0;
	return (vector){};
}