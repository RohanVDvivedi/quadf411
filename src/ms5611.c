#include<ms5611.h>

#include<math.h>

int init_ms5611(ms5611* mod_baro, I2C_HandleTypeDef* hi2c, uint8_t i2c_addr, dpipe* i2c_queue)
{
	mod_baro->hi2c = hi2c;
	mod_baro->i2c_addr = i2c_addr;
	mod_baro->state = MS5611_NOT_STARTED;
	mod_baro->state_data_type = 1;
	mod_baro->i2c_queue = i2c_queue;
	mod_baro->last_read_in_millis = HAL_GetTick();
	mod_baro->read_period_in_millis = 10;

	mod_baro->commands[1] = 0x48;
	mod_baro->commands[2] = 0x58;

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

static int32_t ms5611_pressure_pa(uint16_t *C, uint32_t D1, uint32_t D2, int32_t *temp_centi)
{
	int32_t dT = (int32_t)D2 - ((int32_t)C[5] << 8);
	int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / (1LL << 23);

	int64_t OFF  = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT) / (1LL << 7);
	int64_t SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT) / (1LL << 8);

	int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

	if(TEMP < 2000)
	{
		T2 = ((int64_t)dT * dT) >> 31;
		OFF2 = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) >> 1;
		SENS2 = 5 * ((int64_t)(TEMP - 2000) * (TEMP - 2000)) >> 2;

		if (TEMP < -1500)
		{
			OFF2  += 7 * ((int64_t)(TEMP + 1500) * (TEMP + 1500));
			SENS2 += 11 * ((int64_t)(TEMP + 1500) * (TEMP + 1500)) >> 1;
		}
	}

	TEMP -= T2;
	OFF  -= OFF2;
	SENS -= SENS2;

	int32_t P = (((int64_t)D1 * SENS) >> 21) - OFF;
	P >>= 15;

	if(temp_centi)
		*temp_centi = TEMP;

	return P;
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
				HAL_I2C_Master_Transmit_IT(mod_baro->hi2c, (mod_baro->i2c_addr) << 1, mod_baro->commands + mod_baro->state_data_type, 1);
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
				HAL_I2C_Mem_Read_IT(
					mod_baro->hi2c,
					(mod_baro->i2c_addr) << 1,
					0x00, // register address
					I2C_MEMADD_SIZE_8BIT,
					&(mod_baro->read_buffer_D[mod_baro->state_data_type][0]),
					3
				);
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

			uint32_t D[3];
			D[1] = (((uint32_t)(mod_baro->read_buffer_D[1][0])) << 16) | (((uint32_t)(mod_baro->read_buffer_D[1][1])) << 8) | ((uint32_t)(mod_baro->read_buffer_D[1][2]));
			D[2] = (((uint32_t)(mod_baro->read_buffer_D[2][0])) << 16) | (((uint32_t)(mod_baro->read_buffer_D[2][1])) << 8) | ((uint32_t)(mod_baro->read_buffer_D[2][2]));

			int32_t temperature;
			int32_t pressure = ms5611_pressure_pa(mod_baro->C, D[1], D[2], &temperature);

			mod_baro->data = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.190294957));

			(*new_data_arrived) = 1;
			return mod_baro->data;
		}
	}

	(*new_data_arrived) = 0;
	return 0;
}