#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<stdio.h>
#include<string.h>

#include<cutlery/dpipe.h>

#include<adxl345.h>

adxl345 mod_accl;

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

volatile uint8_t i2c_tx_ready = 1;
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		maybe_data_ready_adxl345(&mod_accl);
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

	if(!init_adxl345(&mod_accl, &hi2c1, 0x53, &i2c_sensor_queue, 5)) // collect samples every 5 millis
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)("could not init adxl345"), strlen("could not init adxl345"), HAL_MAX_DELAY);
		while(1);
	}

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

	vector accl_data = {};

	uint32_t last_print_at = HAL_GetTick();
	uint32_t print_period = 100; // print every 100 millis

	while(1)
	{
		int new_data_arrived = 0;

		vector _accl_data = get_adxl345(&mod_accl, &new_data_arrived);
		if(new_data_arrived)
			accl_data = _accl_data;

		if(last_print_at + print_period >= HAL_GetTick())
		{
			char buffer[100];
			sprintf(buffer, "ax=%f, ay=%f, az=%f\n", accl_data.xi, accl_data.yj, accl_data.zk);
			HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
			last_print_at = HAL_GetTick();
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