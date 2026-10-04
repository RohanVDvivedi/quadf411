#include<neo_6m.h>

void init_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart)
{
	mod_neo_6m->gps_data = (neo_6m_data){};

	mod_neo_6m->last_read_in_millis = HAL_GetTick();

	initialize_dpipe_with_memory(&(mod_neo_6m->unparsed_bytes), BUFFER_BYTES, mod_neo_6m->unparsed_bytes_buffer);

	mod_neo_6m->huart = huart;

	HAL_UART_Receive_IT(mod_neo_6m->huart, &(mod_neo_6m->received_byte), 1);
}

// to be called asynchronously in interrupt
void accept_byte_for_neo_6m(neo_6m* mod_neo_6m)
{
	if(is_full_dpipe(&(mod_neo_6m->unparsed_bytes)))
		discard_from_dpipe(&(mod_neo_6m->unparsed_bytes), 1);

	write_to_dpipe(&(mod_neo_6m->unparsed_bytes), &(mod_neo_6m->received_byte), 1, ALL_OR_NONE);

	HAL_UART_Receive_IT(mod_neo_6m->huart, &(mod_neo_6m->received_byte), 1);
}

// if there is new data available it is all parsed and latest one is returned
static char debug_buffer[100];
neo_6m_data get_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart_temp, int* new_data_arrived)
{
	(*new_data_arrived) = 0;

	// disable interrupts for stable reading from dpipe's buffer
	__disable_irq();

	if(get_bytes_readable_in_dpipe(&(mod_neo_6m->unparsed_bytes)) >= 100 && HAL_GetTick() > mod_neo_6m->last_read_in_millis + 500UL)
	{
		(*new_data_arrived) = 1;
		mod_neo_6m->last_read_in_millis = HAL_GetTick();
		uint32_t debug_bytes = read_from_dpipe(&(mod_neo_6m->unparsed_bytes), debug_buffer, sizeof(debug_buffer), PARTIAL_ALLOWED);
		if(debug_bytes > 0)
			HAL_UART_Transmit_IT(huart_temp, (uint8_t*)debug_buffer, debug_bytes);
	}

	// enable interrupts back
	__enable_irq();

	return mod_neo_6m->gps_data;
}