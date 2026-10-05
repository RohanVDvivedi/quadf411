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
#include<fs_i6_ibus_receiver.h>
#include<neo_6m.h>

adxl345 mod_accl;
itg3205 mod_gyro;
hmc5883l mod_magn;
ms5611 mod_baro;
fs_i6_ibus mod_fs_i6_ibus;
neo_6m mod_neo_6m;

void SysTick_Handler(void)
{
	HAL_IncTick();
}

UART_HandleTypeDef huart1;

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

UART_HandleTypeDef huart2;

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

UART_HandleTypeDef huart6;

void USART6_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart6);
}

volatile uint8_t uart_tx_ready = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
		uart_tx_ready = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == mod_fs_i6_ibus.huart)
		accept_byte_for_fs_i6_ibus(&mod_fs_i6_ibus);
	if(huart == mod_neo_6m.huart)
		accept_byte_for_neo_6m(&mod_neo_6m);
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

TIM_HandleTypeDef htim3; // for motors 1, 2, 3, 4
TIM_HandleTypeDef htim4; // for motors 5, 6

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(UART_HandleTypeDef* huart1);
static void UART2_Init(UART_HandleTypeDef* huart2);
static void UART6_Init(UART_HandleTypeDef* huart6);
static void I2C1_Init(I2C_HandleTypeDef* hi2c1);

static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

char debug_buffer[512];

int write_board_motor_pwn(int motor_no, uint16_t pulse_width_in_us) // for escs write 1000 to 2000 values only here
{
	switch(motor_no)
	{
		case 1 : __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width_in_us); break;
		case 2 : __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_width_in_us); break;
		case 3 : __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_width_in_us); break;
		case 4 : __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_width_in_us); break;
		case 5 : __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_width_in_us); break;
		case 6 : __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_width_in_us); break;
		default : return 0;
	}
	return 1;
}

int main(void)
{
	HAL_Init();

	// setup clock to run at highest frequency
	SystemClock_Config();

	// 100 ms delay to let the power lines get stable
	HAL_Delay(100);

	// setup LED pin as output
	GPIO_Init();

	// setup UART at baud of 115200
	UART1_Init(&huart1);
	UART2_Init(&huart2);
	UART6_Init(&huart6);

	// setup I2C at baud of 100000
	I2C1_Init(&hi2c1);

	// setup timers for motors
	MX_TIM3_Init();
	MX_TIM4_Init();

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

	init_fs_i6_ibus_receiver(&mod_fs_i6_ibus, &huart2);

	init_neo_6m(&mod_neo_6m, &huart6);

	if(failed)
		while(1);

	int is_valid_boot_time_accl_data = 0;
	vector boot_time_accl_data = {};
	vector average_gyro_data = {};
	#define GYRO_AVERAGE_SAMPLES 500
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

	int receiver_channels_data_valid = 0;
	fs_i6_data receiver_channels_data = {};

	int gps_data_valid = 0;
	neo_6m_data gps_data = {};

	uint32_t last_print_at = HAL_GetTick();
	uint32_t print_period = 1000; // print every 1000 millis

	int accl_samples = 0;
	int gyro_samples = 0;
	int magn_samples = 0;
	int baro_samples = 0;

	// right before the controller is ready then start att motors
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

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
				is_valid_boot_time_accl_data = (HAL_GetTick() > 300); // wait for 300 millis
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

		new_data_arrived = 0;
		fs_i6_data _receiver_channels_data = get_fs_i6_ibus(&mod_fs_i6_ibus, &new_data_arrived);
		if(new_data_arrived)
		{
			receiver_channels_data = _receiver_channels_data;
			receiver_channels_data_valid = 1;

			// write directly to motor 5 and 6
			write_board_motor_pwn(5, receiver_channels_data.channels[0]);
			write_board_motor_pwn(6, receiver_channels_data.channels[1]);
		}

		new_data_arrived = 0;
		neo_6m_data _gps_data = get_neo_6m(&mod_neo_6m, &huart1, &new_data_arrived);
		if(new_data_arrived)
		{
			gps_data = _gps_data;
			gps_data_valid = 1;
		}

		if(HAL_GetTick() >= last_print_at + print_period)
		{
			sprintf(debug_buffer, "ax=%f, ay=%f, az=%f, a_samples = %d, gx=%f, gy=%f, gz=%f, g_samples=%d, mx=%f, my=%f, mz=%f, m_samples=%d, z_pos = %f, b_samples=%d\n", accl_data.xi, accl_data.yj, accl_data.zk, accl_samples, gyro_data.xi, gyro_data.yj, gyro_data.zk, gyro_samples, magn_data.xi, magn_data.yj, magn_data.zk, magn_samples, baro_data, baro_samples);
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer));
			/*sprintf(debug_buffer, "abs_pitch=%f \t abs_roll=%f\n", abs_pitch, abs_roll);
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer));*/
			/*if(receiver_channels_data_valid)
			{
				sprintf(debug_buffer, "receiver_data[0]=%hu \t receiver_data[1]=%hu \t receiver_data[2]=%hu \t receiver_data[3]=%hu\n", receiver_channels_data.channels[0], receiver_channels_data.channels[1], receiver_channels_data.channels[2], receiver_channels_data.channels[3]);
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer));
			}*/
			last_print_at = HAL_GetTick();
			accl_samples = 0;
			gyro_samples = 0;
			magn_samples = 0;
			baro_samples = 0;
		}
	}
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

	if(HAL_RCC_OscConfig(&osc) != HAL_OK)
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

	if(HAL_RCC_ClockConfig(&clk, FLASH_ACR_LATENCY_3WS) != HAL_OK)
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

static void UART2_Init(UART_HandleTypeDef* huart2)
{
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_NOPULL;
	gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &gpio);

	huart2->Instance          = USART2;
	huart2->Init.BaudRate     = 115200;
	huart2->Init.WordLength   = UART_WORDLENGTH_8B;
	huart2->Init.StopBits     = UART_STOPBITS_1;
	huart2->Init.Parity       = UART_PARITY_NONE;
	huart2->Init.Mode         = UART_MODE_TX_RX;
	huart2->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart2->Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(huart2);

	HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void UART6_Init(UART_HandleTypeDef* huart6)
{
	__HAL_RCC_USART6_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_NOPULL;
	gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOA, &gpio);

	huart6->Instance          = USART6;
	huart6->Init.BaudRate     = 9600;
	huart6->Init.WordLength   = UART_WORDLENGTH_8B;
	huart6->Init.StopBits     = UART_STOPBITS_1;
	huart6->Init.Parity       = UART_PARITY_NONE;
	huart6->Init.Mode         = UART_MODE_TX_RX;
	huart6->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart6->Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(huart6);

	HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART6_IRQn);
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
	hi2c1->Init.ClockSpeed = 400000;
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

/* ---------------- TIMers for motors ---------------- */

#define TIM_PRESCALER   (100u - 1) 			// 1 MHz clock in to the timers
#define TIM_PERIOD      (20000u - 1u) 		// 20,000 total pulses in one period

#define MOTOR_PWM_MIN    1000 // initial pwm valkue

static void MX_TIM3_Init(void)
{
	/* ---- 1. Enable clocks first ---- */
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* ---- 2. Configure GPIO ---- */
	/*
	 * PB0  → TIM3_CH3  AF2
	 * PB1  → TIM3_CH4  AF2
	 * PB4  → TIM3_CH1  AF2
	 * PB5  → TIM3_CH2  AF2
	 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 |
	                            GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* ---- 3. Configure timer ---- */
	htim3.Instance               = TIM3;
	htim3.Init.Prescaler         = TIM_PRESCALER;
	htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim3.Init.Period            = TIM_PERIOD;
	htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if(HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		__disable_irq();
		while (1);
	}

	/* ---- 4. Configure channels ---- */
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode     = TIM_OCMODE_PWM1;
	sConfigOC.Pulse      = MOTOR_PWM_MIN;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if(HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) // motor 1
	{
		__disable_irq();
		while (1);
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) // motor 2
	{
		__disable_irq();
		while (1);
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) // motor 3
	{
		__disable_irq();
		while (1);
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) // motor 4
	{
		__disable_irq();
		while (1);
	}
}

static void MX_TIM4_Init(void)
{
	/* ---- 1. Enable clocks first ---- */
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();   /* likely already enabled; safe to call again */

	/* ---- 2. Configure GPIO ---- */
	/*
	 * PB6  → TIM4_CH1  AF2
	 * PB7  → TIM4_CH2  AF2
	 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* ---- 3. Configure timer ---- */
	htim4.Instance               = TIM4;
	htim4.Init.Prescaler         = TIM_PRESCALER;
	htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim4.Init.Period            = TIM_PERIOD;
	htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if(HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		__disable_irq();
		while (1);
	}

	/* ---- 4. Configure channels ---- */
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode       = TIM_OCMODE_PWM1;
	sConfigOC.Pulse        = MOTOR_PWM_MIN;
	sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

	if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) // motor 5
	{
		__disable_irq();
		while (1);
	}
	if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) // motor 6
	{
		__disable_irq();
		while (1);
	}
}
