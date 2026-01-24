#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<hello_world_from_uart.h>

void SysTick_Handler(void)
{
	HAL_IncTick();
}

UART_HandleTypeDef huart2;

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART2_Init(UART_HandleTypeDef* huart2);

int main(void)
{
	HAL_Init();

	// setup clock to run at highest frequency
	SystemClock_Config();

	// setup LED pin as output
	GPIO_Init();

	// setup UART at baud of 115200
	UART2_Init(&huart2);

	uint32_t delay_ms = 2000;
	while(1)
	{
		// toggle LED
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		// print hello world on UART, blockingly
		print_hello_world_from_uart(&huart2);

		// diminishing delay by a fraction of 0.93, and reset delay if the delay is lesser than 10 ms
		float new_delay_ms = (float)(delay_ms * 0.8f);
		if (new_delay_ms < 70.5)
			delay_ms = 2000;
		else
			delay_ms = new_delay_ms;

		HAL_Delay(delay_ms);
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