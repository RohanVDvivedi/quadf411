#include<itg3205.h>

int init_itg3205(itg3205* mod_gyro, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue, uint32_t period_in_millis)
{
	mod_gyro->hi2c = hi2c;
	mod_gyro->i2c_addr = i2c_addr;
	mod_gyro->state = ITG3205_NOT_STARTED;
	mod_gyro->i2c_queue = i2c_queue;
	mod_gyro->last_read_in_millis = HAL_GetTick();
	mod_gyro->read_period_in_millis = period_in_millis;

	if(HAL_I2C_IsDeviceReady(hi2c, i2c_addr << 1, 3, 100) != HAL_OK)
		return 0;

	// run the sensor at full speed, regardless of our reading rate
	HAL_I2C_Mem_Write(hi2c, (mod_gyro->i2c_addr) << 1, 0x16, I2C_MEMADD_SIZE_8BIT, ((uint8_t[]){0x18}), 1, HAL_MAX_DELAY);

	// put it to externally sample as fast as possible
	HAL_I2C_Mem_Write(hi2c, (mod_gyro->i2c_addr) << 1, 0x15, I2C_MEMADD_SIZE_8BIT, ((uint8_t[]){0x00}), 1, HAL_MAX_DELAY);

	// turn it on with some axis gyro to provide it with clock
	HAL_I2C_Mem_Write(hi2c, (mod_gyro->i2c_addr) << 1, 0x3e, I2C_MEMADD_SIZE_8BIT, ((uint8_t[]){0x01}), 1, HAL_MAX_DELAY);

	return 1;
}

int maybe_data_ready_itg3205(itg3205* mod_gyro)
{
	if(mod_gyro->state != ITG3205_READ_IN_PROGRESS)
		return 0;
	uint8_t buffer[1];
	cy_uint bytes_read = peek_from_dpipe(mod_gyro->i2c_queue, buffer, 1, ALL_OR_NONE);
	if(bytes_read > 0 && buffer[0] == mod_gyro->i2c_addr)
	{
		read_from_dpipe(mod_gyro->i2c_queue, buffer, 1, ALL_OR_NONE);
		mod_gyro->state = ITG3205_READ_COMPLETE;
		return 1;
	}
	return 0;
}

vector get_itg3205(itg3205* mod_gyro, int* new_data_arrived)
{
	switch(mod_gyro->state)
	{
		case ITG3205_NOT_STARTED :
		{
			// queue read request to dpipe
			{
				__disable_irq();
					write_to_dpipe(mod_gyro->i2c_queue, &(mod_gyro->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_gyro->state = ITG3205_READ_QUEUED;
			}

			break;
		}
		case ITG3205_READ_QUEUED :
		{
			uint8_t buffer[1];
			__disable_irq();
				cy_uint bytes_read = peek_from_dpipe(mod_gyro->i2c_queue, buffer, 1, ALL_OR_NONE);
			__enable_irq();
			if(bytes_read > 0 && buffer[0] == mod_gyro->i2c_addr)
			{
				// we are at the top of the queue, so schedule this read
				HAL_I2C_Mem_Read_IT(
					mod_gyro->hi2c,
					(mod_gyro->i2c_addr) << 1,
					0x1d, // register address
					I2C_MEMADD_SIZE_8BIT,
					mod_gyro->read_buffer,
					6
				);
				mod_gyro->state = ITG3205_READ_IN_PROGRESS;
			}

			break;
		}
		case ITG3205_READ_IN_PROGRESS : // user application can not do anything there, this transition is done by the maybe_data_ready_itg3205()
		{
			break;
		}
		case ITG3205_READ_COMPLETE :
		{
			// transform read_buffer -> data in vector
			mod_gyro->data.xi = (int16_t)((((int16_t)(mod_gyro->read_buffer[0]))<<8) | ((int16_t)(mod_gyro->read_buffer[1])));
			mod_gyro->data.yj = (int16_t)((((int16_t)(mod_gyro->read_buffer[2]))<<8) | ((int16_t)(mod_gyro->read_buffer[3])));
			mod_gyro->data.zk = (int16_t)((((int16_t)(mod_gyro->read_buffer[4]))<<8) | ((int16_t)(mod_gyro->read_buffer[5])));

			// record the millis when last read was done by the user
			mod_gyro->last_read_in_millis = HAL_GetTick();

			// transition state to waiting
			mod_gyro->state = ITG3205_WAIT_FOR_PERIOD;

			// prepare return values and return
			(*new_data_arrived) = 1;
			return mod_gyro->data;
		}
		case ITG3205_WAIT_FOR_PERIOD :
		{
			if(HAL_GetTick() >= mod_gyro->last_read_in_millis + mod_gyro->read_period_in_millis)
			{
				// queue read request to dpipe
				__disable_irq();
					write_to_dpipe(mod_gyro->i2c_queue, &(mod_gyro->i2c_addr), 1, ALL_OR_NONE);
				__enable_irq();
				mod_gyro->state = ITG3205_READ_QUEUED;
			}

			break;
		}
	}

	(*new_data_arrived) = 0;
	return (vector){};
}