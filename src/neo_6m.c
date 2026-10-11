#include<neo_6m.h>

#include<string.h>
#include<stdio.h>

void init_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart)
{
	mod_neo_6m->gps_data = (neo_6m_data){};

	mod_neo_6m->last_read_in_millis = HAL_GetTick();

	initialize_dpipe_with_memory(&(mod_neo_6m->unparsed_bytes), BUFFER_BYTES, mod_neo_6m->unparsed_bytes_buffer);
	initialize_buffer_read_sizes_with_memory(&(mod_neo_6m->unparsed_sentence_sizes), sizeof(mod_neo_6m->unparsed_sentence_sizes_buffer) / sizeof(uint32_t), mod_neo_6m->unparsed_sentence_sizes_buffer);

	mod_neo_6m->huart = huart;

	HAL_UART_Receive_IT(mod_neo_6m->huart, &(mod_neo_6m->received_byte), 1);
}

// to be called asynchronously in interrupt
void accept_byte_for_neo_6m(neo_6m* mod_neo_6m)
{
	if(is_full_dpipe(&(mod_neo_6m->unparsed_bytes)))
	{
		if(is_empty_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes))) // single statement too big, very likely garbage, discard everything, and start fresh
		{
			remove_all_from_dpipe(&(mod_neo_6m->unparsed_bytes));
			remove_all_from_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
		}
		else // discard the oldest statement and it's size
		{
			uint32_t oldest_statement_length = *get_front_of_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
			pop_front_from_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
			discard_from_dpipe(&(mod_neo_6m->unparsed_bytes), oldest_statement_length);
		}
	}

	write_to_dpipe(&(mod_neo_6m->unparsed_bytes), &(mod_neo_6m->received_byte), 1, ALL_OR_NONE);

	// check tail
	{
		char last_2_bytes[2];
		if(2 == get_back_of_dpipe(&(mod_neo_6m->unparsed_bytes), last_2_bytes, 2, ALL_OR_NONE) && 0 == memcmp(last_2_bytes, "\r\n", 2)) // if this is true we are at the end of the statement, i.e. we just read the last byte of the sentence
		{
			// total bytes in unparsed_dpipe - total length of all known sentences
			uint32_t last_sentence_size = get_bytes_readable_in_dpipe(&(mod_neo_6m->unparsed_bytes));
			for(uint32_t i = 0; i < get_element_count_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes)); i++)
				last_sentence_size -= (*get_from_front_of_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes), i));
			push_back_to_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes), &last_sentence_size);
		}
	}

	HAL_UART_Receive_IT(mod_neo_6m->huart, &(mod_neo_6m->received_byte), 1);
}

// if there is new data available it is all parsed and latest one is returned
static char debug_buffer[128];
neo_6m_data get_neo_6m(neo_6m* mod_neo_6m, UART_HandleTypeDef* huart_temp, int* new_data_arrived)
{
	(*new_data_arrived) = 0;

	// disable interrupts for stable reading from dpipe's buffer
	__disable_irq();

	while(!is_empty_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes)))
	{
		uint32_t sentence_length = *get_front_of_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
		char sentence[128];

		// too big sentence just skip it
		if(sentence_length > 100)
		{
			pop_front_from_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
			discard_from_dpipe(&(mod_neo_6m->unparsed_bytes), sentence_length);
			continue;
		}

		read_from_dpipe(&(mod_neo_6m->unparsed_bytes), sentence, sentence_length, ALL_OR_NONE);
		pop_front_from_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
		sentence[sentence_length] = '\0';

		// now the sentence and sentence length is ready to be parsed

		uint32_t debug_data_size = sprintf(debug_buffer, "<< -- %.*s -- >>\n", (int)(sentence_length-2), sentence); // -2 for skipping \r\n in the log

		(*new_data_arrived) = 1;
		mod_neo_6m->last_read_in_millis = HAL_GetTick();

		if(debug_data_size > 0)
			HAL_UART_Transmit_IT(huart_temp, (uint8_t*)debug_buffer, debug_data_size);
	}

	// enable interrupts back
	__enable_irq();

	return mod_neo_6m->gps_data;
}