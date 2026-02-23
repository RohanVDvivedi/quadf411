#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<stdio.h>
#include<string.h>
#include<math.h>

#include<cutlery/dpipe.h>

#include<cutlery/dpipe.h>

#include<adxl345.h>
#include<itg3205.h>
#include<hmc5883l.h>
#include<ms5611.h>

adxl345 mod_accl;
itg3205 mod_gyro;
hmc5883l mod_magn;
ms5611 mod_baro;

void SysTick_Handler(void)
{
	HAL_IncTick();
}

UART_HandleTypeDef huart1;

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

volatile uint8_t uart_tx_ready = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
		uart_tx_ready = 1;
}

I2C_HandleTypeDef hi2c1;

void I2C1_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(&hi2c1);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		if(maybe_data_ready_adxl345(&mod_accl) || maybe_data_ready_itg3205(&mod_gyro) || maybe_data_ready_hmc5883l(&mod_magn) || maybe_data_ready_ms5611(&mod_baro))
		{
			asm("");
		}
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		if(maybe_data_ready_ms5611(&mod_baro))
		{
			asm("");
		}
	}
}

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(UART_HandleTypeDef* huart1);
static void I2C1_Init(I2C_HandleTypeDef* hi2c1);

int main(void)
{
	HAL_Init();

	// setup clock to run at highest frequency
	SystemClock_Config();

	// setup LED pin as output
	GPIO_Init();

	// setup UART at baud of 115200
	UART1_Init(&huart1);

	// setup I2C at baud of 100000
	I2C1_Init(&hi2c1);

	#define I2C_SENSOR_QUEUE_CAPACITY 128
	uint8_t i2c_sensor_queue_buffer[I2C_SENSOR_QUEUE_CAPACITY];
	dpipe i2c_sensor_queue;
	initialize_dpipe_with_memory(&i2c_sensor_queue, I2C_SENSOR_QUEUE_CAPACITY, i2c_sensor_queue_buffer);

	int failed = 0;

	if(!init_adxl345(&mod_accl, &hi2c1, 0x53, &i2c_sensor_queue, 4)) // collect samples every 4 millis -> 250 Hz
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)("could not init adxl345\n"), strlen("could not init adxl345\n"), HAL_MAX_DELAY);
		failed = 1;
	}

	if(!init_itg3205(&mod_gyro, &hi2c1, 0x68, &i2c_sensor_queue, 2)) // collect samples every 2 millis -> 500 Hz
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)("could not init itg3205\n"), strlen("could not init itg3205\n"), HAL_MAX_DELAY);
		failed = 1;
	}

	if(!init_hmc5883l(&mod_magn, &hi2c1, 0x1e, &i2c_sensor_queue, 15)) // collect samples every 15 millis -> 66 Hz
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)("could not init hmc5883l\n"), strlen("could not init hmc5883l\n"), HAL_MAX_DELAY);
		failed = 1;
	}

	if(!init_ms5611(&mod_baro, &hi2c1, 0x77, &i2c_sensor_queue)) // collect samples every 10 millis -> 100 Hz
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)("could not init ms5611\n"), strlen("could not init ms5611\n"), HAL_MAX_DELAY);
		failed = 1;
	}

	if(failed)
		while(1);

	/*const char* i2c_devices[] = {
		"adxl345",
		"itg3205",
		"hmc5883l",
		"ms5611"
	};

	const uint8_t i2c_addresses[] = {
		0x53,
		0x68,
		0x1e,
		0x77
	};*/

	int is_valid_boot_time_accl_data = 0;
	vector boot_time_accl_data = {};
	vector average_gyro_data = {};
	#define GYRO_AVERAGE_SAMPLES 200
	uint32_t gyro_init_samples = 0;
	int is_valid_boot_time_magn_data = 0;
	vector boot_time_magn_data = {};

	vector accl_data = {}; uint32_t dt_accl = 0; uint32_t t_accl = 0;
	vector gyro_data = {}; uint32_t dt_gyro = 0; uint32_t t_gyro = 0;
	vector magn_data = {}; uint32_t dt_magn = 0; uint32_t t_magn = 0;
	double baro_data = 0;  uint32_t dt_baro = 0; uint32_t t_baro = 0;

	// absolute pitch and roll in degrees
	double abs_pitch = 0;
	double abs_roll = 0;

	uint32_t last_print_at = HAL_GetTick();
	uint32_t print_period = 1000; // print every 100 millis

	int accl_samples = 0;
	int gyro_samples = 0;
	int magn_samples = 0;
	int baro_samples = 0;

	while(1)
	{
		int new_data_arrived;

		new_data_arrived = 0;
		vector _accl_data = get_adxl345(&mod_accl, &new_data_arrived);
		if(new_data_arrived)
		{
			accl_samples++;

			if(t_accl != 0)
				dt_accl = (HAL_GetTick() - t_accl);

			if(!is_valid_boot_time_accl_data)
			{
				is_valid_boot_time_accl_data = (HAL_GetTick() > 100); // wait for 100 millis
				boot_time_accl_data = vector_mul_scalar(_accl_data, 4.0/1000.0); // convert to number of g-s of acceleration
			}
			else
			{
				accl_data = vector_mul_scalar(_accl_data, 4.0/1000.0); // convert to number of g-s of acceleration

				// use accl_data
				{
					vector boot_time_ay = vector_unit_dir(NULL, vector_perpendicular_component(NULL, boot_time_accl_data, unit_vector_y_axis));
					vector curr_ay = vector_unit_dir(NULL, vector_perpendicular_component(NULL, accl_data, unit_vector_y_axis));
					float _abs_pitch = angle_between_2_vectors(unit_vector_y_axis, curr_ay, boot_time_ay);
					if(!isnan(_abs_pitch))
						abs_pitch = 0.98 * abs_pitch + 0.02 * (_abs_pitch * 180.0 / M_PI);
				}
				{
					vector boot_time_ax = vector_unit_dir(NULL, vector_perpendicular_component(NULL, boot_time_accl_data, unit_vector_x_axis));
					vector curr_ax = vector_unit_dir(NULL, vector_perpendicular_component(NULL, accl_data, unit_vector_x_axis));
					float _abs_roll = angle_between_2_vectors(unit_vector_x_axis, curr_ax, boot_time_ax);
					if(!isnan(_abs_roll))
						abs_roll = 0.98 * abs_roll + 0.02 * (_abs_roll * 180.0 / M_PI);
				}
			}

			// get time for the accl_data
			t_accl = HAL_GetTick();
		}

		new_data_arrived = 0;
		vector _gyro_data = get_itg3205(&mod_gyro, &new_data_arrived);
		if(new_data_arrived)
		{
			gyro_samples++;

			if(t_gyro != 0)
				dt_gyro = (HAL_GetTick() - t_gyro);

			if(gyro_init_samples < GYRO_AVERAGE_SAMPLES)
			{
				average_gyro_data = vector_sum(average_gyro_data, _gyro_data);
				gyro_init_samples++;
				if(gyro_init_samples >= GYRO_AVERAGE_SAMPLES)
					average_gyro_data = vector_mul_scalar(average_gyro_data, 1.0 / gyro_init_samples);
			}
			else
			{
				gyro_data = vector_mul_scalar(vector_sub(_gyro_data, average_gyro_data), 1.0/14.375); // convert to degrees per second

				abs_pitch += (gyro_data.yj * (dt_gyro / 1000.0));
				if(abs_pitch > 180)
					abs_pitch -= 360;
				else if(abs_pitch < -180)
					abs_pitch += 360;
				abs_roll += (gyro_data.xi * (dt_gyro / 1000.0));
				if(abs_roll > 180)
					abs_roll -= 360;
				else if(abs_roll < -180)
					abs_roll += 360;
			}

			// get time for the gyro_data
			t_gyro = HAL_GetTick();
		}

		new_data_arrived = 0;
		vector _magn_data = get_hmc5883l(&mod_magn, &new_data_arrived);
		if(new_data_arrived)
		{
			magn_samples++;

			if(t_magn != 0)
				dt_magn = (HAL_GetTick() - t_magn);

			if(!is_valid_boot_time_magn_data)
			{
				is_valid_boot_time_magn_data = 1;
				boot_time_magn_data = vector_mul_scalar(_magn_data, 0.92 / 1000.0); // convert to the range in Gauss
			}
			else
			{
				magn_data = vector_mul_scalar(_magn_data, 0.92 / 1000.0); // convert to the range in Gauss

				// use magn_data
			}

			// get time for the magn_data
			t_magn = HAL_GetTick();
		}

		new_data_arrived = 0;
		double _baro_data = get_ms5611(&mod_baro, &new_data_arrived);
		if(new_data_arrived)
		{
			baro_samples++;

			if(t_baro != 0)
				dt_baro = (HAL_GetTick() - t_baro);

			baro_data = _baro_data; // convert to the CM

			// get time for the baro_data
			t_baro = HAL_GetTick();
		}

		if(HAL_GetTick() >= last_print_at + print_period)
		{
			char buffer[300];
			/*sprintf(buffer, "ax=%f, ay=%f, az=%f, a_samples = %d, gx=%f, gy=%f, gz=%f, g_samples=%d, mx=%f, my=%f, mz=%f, m_samples=%d, z_pos = %f, b_samples=%d\n", accl_data.xi, accl_data.yj, accl_data.zk, accl_samples, gyro_data.xi, gyro_data.yj, gyro_data.zk, gyro_samples, magn_data.xi, magn_data.yj, magn_data.zk, magn_samples, baro_data, baro_samples);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);*/
			sprintf(buffer, "abs_pitch=%f \t abs_roll=%f\n", abs_pitch, abs_roll);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			last_print_at = HAL_GetTick();
			accl_samples = 0;
			gyro_samples = 0;
			magn_samples = 0;
			baro_samples = 0;
		}
	}

	/*for(int i = 0; i < sizeof(i2c_addresses)/sizeof(i2c_addresses[0]); i++)
	{
		char buffer[100];
		if(HAL_I2C_IsDeviceReady(&hi2c1, i2c_addresses[i] << 1, 3, 10) == HAL_OK)
			sprintf(buffer, "%s @ %hhx : detected\n", i2c_devices[i], i2c_addresses[i]);
		else
			sprintf(buffer, "%s @ %hhx : absent\n", i2c_devices[i], i2c_addresses[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	}

	while(1);*/

	/*while(uart_tx_ready == 0);
	uart_tx_ready = 0;
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)"S", 1);

	while(uart_tx_ready == 0);
	uart_tx_ready = 0;
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)"T", 1);

	while(uart_tx_ready == 0);
	uart_tx_ready = 0;
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)"S\n", 2);

	uint32_t delay_ms = 2000;
	while(1)
	{
		// toggle LED
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		// print hello world on UART
		if(uart_tx_ready)
		{
			uart_tx_ready = 0;  // mark busy
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)"Hello world!!\n", 14);
		}

		HAL_Delay(delay_ms);
	}*/
}

/* ---------------- clock ---------------- */

static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef osc = {0};
	RCC_ClkInitTypeDef clk = {0};

	// Enable power control clock
	__HAL_RCC_PWR_CLK_ENABLE();

	// Configure voltage scaling for max frequency
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	// HSE + PLL @ 100 MHz
	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState       = RCC_HSE_ON;
	osc.PLL.PLLState   = RCC_PLL_ON;
	osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	osc.PLL.PLLM       = 25;
	osc.PLL.PLLN       = 400;
	osc.PLL.PLLP       = RCC_PLLP_DIV4;   // 100 MHz
	osc.PLL.PLLQ       = 8;

	if (HAL_RCC_OscConfig(&osc) != HAL_OK)
	{
		__disable_irq();
		while (1);
	}

	// Bus clocks
	clk.ClockType = RCC_CLOCKTYPE_SYSCLK |
					RCC_CLOCKTYPE_HCLK   |
					RCC_CLOCKTYPE_PCLK1  |
					RCC_CLOCKTYPE_PCLK2;

	clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	clk.APB1CLKDivider = RCC_HCLK_DIV2;   // max 50 MHz
	clk.APB2CLKDivider = RCC_HCLK_DIV1;   // max 100 MHz

	if (HAL_RCC_ClockConfig(&clk, FLASH_ACR_LATENCY_3WS) != HAL_OK)
	{
		__disable_irq();
		while (1);
	}
}

/* ---------------- GPIO ---------------- */

static void GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin   = GPIO_PIN_13;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOC, &gpio);
}

/* ---------------- UART ---------------- */

static void UART1_Init(UART_HandleTypeDef* huart1)
{
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_NOPULL;
	gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &gpio);

	huart1->Instance          = USART1;
	huart1->Init.BaudRate     = 115200;
	huart1->Init.WordLength   = UART_WORDLENGTH_8B;
	huart1->Init.StopBits     = UART_STOPBITS_1;
	huart1->Init.Parity       = UART_PARITY_NONE;
	huart1->Init.Mode         = UART_MODE_TX_RX;
	huart1->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart1->Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(huart1);

	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ---------------- I2C ---------------- */

static void I2C1_Init(I2C_HandleTypeDef* hi2c1)
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
	gpio.Mode      = GPIO_MODE_AF_OD;
	gpio.Pull      = GPIO_PULLUP;
	gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &gpio);

	hi2c1->Instance = I2C1;
	hi2c1->Init.ClockSpeed = 100000;
	hi2c1->Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1->Init.OwnAddress1 = 0;
	hi2c1->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1->Init.OwnAddress2 = 0;
	hi2c1->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	HAL_I2C_Init(hi2c1);

	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}
