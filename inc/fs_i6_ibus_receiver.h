#ifndef FS_I6_IBUS_RECEIVER_H
#define FS_I6_IBUS_RECEIVER_H

#include"stm32f4xx.h"               // device header
#include"stm32f4xx_hal.h"           // main HAL header

#include<stdint.h>

#include<cutlery/dpipe.h>

#define CHANNELS_COUNT 14

typedef struct fs_i6_data fs_i6_data;
struct fs_i6_data
{
	unsigned int has_valid_data:1;

	uint32_t timestamp_in_millis;

	uint16_t channels[CHANNELS_COUNT];
};

#define BYTES_PER_PACKET (2 + (CHANNELS_COUNT * 2) + 2)

#define BUFFER_BYTES (BYTES_PER_PACKET * 2)

typedef struct fs_i6_ibus fs_i6_ibus;
struct fs_i6_ibus
{
	uint8_t received_byte; // per byte receive buffer

	fs_i6_data channels_data;

	dpipe unparsed_bytes;

	uint8_t unparsed_bytes_buffer[BUFFER_BYTES];

	UART_HandleTypeDef* huart;
};

void init_fs_i6_ibus_receiver(fs_i6_ibus* mod_fsi6, UART_HandleTypeDef* huart);

// to be called asynchnously in interrupt
void accept_byte_for_fs_i6_ibus(fs_i6_ibus* mod_fsi6);

// disables interrupt to get the valid snapshot
fs_i6_data fetch_latest_fs_i6_ibus(fs_i6_ibus* mod_fsi6);

#endif