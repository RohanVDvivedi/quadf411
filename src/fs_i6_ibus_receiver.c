#include<fs_i6_ibus_receiver.h>

void init_fs_i6_ibus_receiver(fs_i6_ibus* mod_fsi6, UART_HandleTypeDef* huart)
{
	mod_fsi6->channels_data = (fs_i6_data){.has_valid_data = 0};

	initialize_dpipe_with_memory(&(mod_fsi6->unparsed_bytes), BUFFER_BYTES, mod_fsi6->unparsed_bytes_buffer);

	mod_fsi6->huart = huart;

	HAL_UART_Receive_IT(mod_fsi6->huart, &(mod_fsi6->received_byte), 1);
}

void accept_byte_for_fs_i6_ibus(fs_i6_ibus* mod_fsi6)
{
	if(is_full_dpipe(&(mod_fsi6->unparsed_bytes)))
		discard_from_dpipe(&(mod_fsi6->unparsed_bytes), 1);

	write_to_dpipe(&(mod_fsi6->unparsed_bytes), &(mod_fsi6->received_byte), 1, ALL_OR_NONE);

	HAL_UART_Receive_IT(mod_fsi6->huart, &(mod_fsi6->received_byte), 1);
}

fs_i6_data fetch_latest_fs_i6_ibus(fs_i6_ibus* mod_fsi6)
{
	// disable interrupts for stable reading from dpipe's buffer
	__disable_irq();

	while(get_bytes_readable_in_dpipe(&(mod_fsi6->unparsed_bytes)) >= BYTES_PER_PACKET)
	{
		uint8_t prefix[2];
		peek_from_dpipe(&(mod_fsi6->unparsed_bytes), prefix, sizeof(prefix), ALL_OR_NONE);
		if(prefix[0] != 0x20 || prefix[1] != 0x40)
		{
			discard_from_dpipe(&(mod_fsi6->unparsed_bytes), 1);
			continue;
		}

		uint8_t payload[BYTES_PER_PACKET];
		peek_from_dpipe(&(mod_fsi6->unparsed_bytes), payload, sizeof(payload), ALL_OR_NONE);

		// check checksum
		uint16_t checksum = 0xFFFF;
		for(uint32_t i = 0; i < sizeof(payload) - 2; i++)
			checksum -= payload[i];
		if(checksum != (((uint16_t)payload[sizeof(payload)-2]) | (((uint16_t)payload[sizeof(payload)-1])<<8)))
		{
			discard_from_dpipe(&(mod_fsi6->unparsed_bytes), 1);
			continue;
		}

		// discard confirmed valid bytes in the buffer
		discard_from_dpipe(&(mod_fsi6->unparsed_bytes), BYTES_PER_PACKET);

		// enable interrupts back
		__enable_irq();

		// populate the payload
		mod_fsi6->channels_data.has_valid_data = 1;
		for(uint32_t i = 0; i < CHANNELS_COUNT; i++)
			mod_fsi6->channels_data.channels[i] = (((uint16_t)payload[2*i+2]) | (((uint16_t)payload[2*i+3])<<8));
		mod_fsi6->channels_data.timestamp_in_millis = HAL_GetTick();

		// disable interrupts for stable reading from dpipe's buffer
		__disable_irq();
	}

	// enable interrupts back
	__enable_irq();

	return mod_fsi6->channels_data;
}