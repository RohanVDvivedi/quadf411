#include<hello_world_from_uart.h>

#include<capp/capp.h>

#include<string.h>

char buffer[100];

void print_hello_world_from_uart(UART_HandleTypeDef* huart)
{
	static int i = 0;
	static char msg[] = "Hello, world!";

	temp t;
	init_temp(&t, i, msg);
	sprintf(buffer, "%s, %d\r\n", t.str, t.s);
	i++;

	if(t.s & 1) // if odd do interrupt transfer
	{
		HAL_UART_Transmit_IT(
			huart,                  /* UART handle */
			(uint8_t *)buffer,      /* data buffer */
			strlen(buffer)          /* length (exclude '\0') */
		);
	}
	else
	{
		HAL_UART_Transmit(
			huart,                  /* UART handle */
			(uint8_t *)buffer,      /* data buffer */
			strlen(buffer),         /* length (exclude '\0') */
			HAL_MAX_DELAY
		);
	}
}