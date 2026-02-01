#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<string.h>

void SysTick_Handler(void)
{
	HAL_IncTick();
}

UART_HandleTypeDef huart1;

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

// Callback when IT transmit completes
volatile uint8_t uart_tx_ready = 1;  // simple flag
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart_tx_ready = 1;  // mark ready
    }
}

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(UART_HandleTypeDef* huart1);

int main(void)
{
	HAL_Init();

	// setup clock to run at highest frequency
	SystemClock_Config();

	// setup LED pin as output
	GPIO_Init();

	// setup UART at baud of 115200
	UART1_Init(&huart1);

	while(uart_tx_ready == 0);
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