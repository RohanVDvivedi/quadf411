#include<neo_6m.h>

#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<stddef.h>
#include<math.h>

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

	// check tail if we received a \n
	if(mod_neo_6m->received_byte == '\n')
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

static void neo6m_parse(char* sentence, uint32_t sentence_length, neo_6m_data* gps_data, int* new_position_arrived, int* new_altitude_arrived, int* new_velocity_arrived);

// if there is new data available it is all parsed and latest one is returned
neo_6m_data get_neo_6m(neo_6m* mod_neo_6m, int* new_position_arrived, int* new_altitude_arrived, int* new_velocity_arrived)
{
	(*new_position_arrived) = 0;
	(*new_altitude_arrived) = 0;
	(*new_velocity_arrived) = 0;

	// disable interrupts for stable reading from dpipe's buffer
	__disable_irq();

	while(!is_empty_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes)))
	{
		uint32_t sentence_length = *get_front_of_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
		char sentence[128];

		// too big/short sentence just skip it
		if(sentence_length > 100 || sentence_length < 3)
		{
			pop_front_from_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
			discard_from_dpipe(&(mod_neo_6m->unparsed_bytes), sentence_length);
			continue;
		}

		read_from_dpipe(&(mod_neo_6m->unparsed_bytes), sentence, sentence_length, ALL_OR_NONE);
		pop_front_from_buffer_read_sizes(&(mod_neo_6m->unparsed_sentence_sizes));
		sentence[sentence_length] = '\0';

		// sentence must start with $ and end with \r\n
		if(sentence[0] != '$' || sentence[sentence_length-2] != '\r' || sentence[sentence_length-1] != '\n')
			continue;

		// now the sentence and sentence length is ready to be parsed

		// enable interrupts just for parsing
		__enable_irq();

			// actual parsing
			neo6m_parse(sentence, sentence_length, &(mod_neo_6m->gps_data), new_position_arrived, new_altitude_arrived, new_velocity_arrived);

		// enable interrupts to possibly parse the next sentence
		__disable_irq();

		if((*new_position_arrived) || (*new_altitude_arrived) || (*new_velocity_arrived))
			mod_neo_6m->last_read_in_millis = HAL_GetTick();
	}

	// enable interrupts back
	__enable_irq();

	return mod_neo_6m->gps_data;
}

// GPS neo-6m data parser from single sentences

static int hex_byte(const char *s)
{
	int hi, lo;

	if (s == NULL || s[0] == '\0' || s[1] == '\0')
		return -1;

	if      (s[0] >= '0' && s[0] <= '9') hi = s[0] - '0';
	else if (s[0] >= 'A' && s[0] <= 'F') hi = s[0] - 'A' + 10;
	else if (s[0] >= 'a' && s[0] <= 'f') hi = s[0] - 'a' + 10;
	else return -1;

	if      (s[1] >= '0' && s[1] <= '9') lo = s[1] - '0';
	else if (s[1] >= 'A' && s[1] <= 'F') lo = s[1] - 'A' + 10;
	else if (s[1] >= 'a' && s[1] <= 'f') lo = s[1] - 'a' + 10;
	else return -1;

	return hi * 16 + lo;
}


/* Validate the NMEA XOR checksum. */

static bool nmea_checksum_valid(const char *s)
{
	if(s == NULL || s[0] != '$')
		return false;

	const char *star = strchr(s, '*');

	if(star == NULL || star[1] == '\0' || star[2] == '\0')
		return false;

	/* Checksum must be followed only by CR/LF or string termination. */
	const char *end = star + 3;

	while(*end == '\r' || *end == '\n')
		++end;

	if(*end != '\0')
		return false;

	int expected = hex_byte(star + 1);

	if(expected < 0)
		return false;

	unsigned char actual = 0;

	for(const char *p = s + 1; p < star; ++p)
		actual ^= (unsigned char)*p;

	return actual == (unsigned char)expected;
}


/*
 * Split comma-separated fields in place.
 * Empty fields are preserved.
 */
static int nmea_split(char *s, char *fields[], int max_fields)
{
	int count = 0;

	if(max_fields <= 0)
		return -1;

	char *p = s;

	while(count < max_fields)
	{
		fields[count++] = p;

		char *comma = strchr(p, ',');

		if(comma == NULL)
			break;

		*comma = '\0';
		p = comma + 1;
	}

	return count;
}


/* Parse a finite floating-point number without accepting trailing junk. */

static bool parse_double(const char *s, double *value)
{
	if(s == NULL || *s == '\0' || value == NULL)
		return false;

	char *end;
	double v = strtod(s, &end);

	if(end == s || *end != '\0' || !isfinite(v))
		return false;

	*value = v;
	return true;
}


/* Parse a non-negative integer. */

static bool parse_int(const char *s, int *value)
{
	if (s == NULL || *s == '\0' || value == NULL)
		return false;

	char *end;
	long v = strtol(s, &end, 10);

	if (end == s || *end != '\0' || v < 0 || v > 1000000)
		return false;

	*value = (int)v;
	return true;
}


/*
 * Convert NMEA coordinates:
 *
 * Latitude:  ddmm.mmmm
 * Longitude: dddmm.mmmm
 *
 * Hemisphere: N, S, E, or W.
 */
static bool nmea_coordinate(const char *s, char hemisphere, bool is_latitude, double *result)
{
	double raw;

	if(!parse_double(s, &raw) || raw < 0.0)
		return false;

	int degrees = (int)(raw / 100.0);
	double minutes = raw - (double)degrees * 100.0;

	double limit = is_latitude ? 90.0 : 180.0;

	if(minutes < 0.0 || minutes >= 60.0 || (double)degrees > limit || ((double)degrees == limit && minutes != 0.0))
		return false;

	if (is_latitude)
	{
		if(hemisphere != 'N' && hemisphere != 'S')
			return false;
	}
	else
	{
		if(hemisphere != 'E' && hemisphere != 'W')
			return false;
	}

	double value = (double)degrees + (minutes / 60.0);

	if(hemisphere == 'S' || hemisphere == 'W')
		value = -value;

	(*result) = value;
	return true;
}


/*
 * Parse one complete NMEA sentence.
 *
 * Returns true if the sentence was recognized and contained
 * valid data for at least one of the requested data categories.
 *
 * All output flags are reset to zero on every call.
 *
 * Supported:
 *   RMC: position, speed, heading
 *   GGA: position, altitude
 *
 * Invalid-fix RMC/GGA sentences are rejected.
 * Unsupported sentence types are ignored.
 *
 * The caller must pass a writable-independent, null-terminated
 * complete sentence. The input is copied before modification.
 */
#define NEO6M_MAX_FIELDS 30
static void neo6m_parse(char* sentence, uint32_t sentence_length, neo_6m_data* gps_data, int* new_position_arrived, int* new_altitude_arrived, int* new_velocity_arrived)
{
	/* Remove CR/LF from the end. */
	sentence_length -= 2;
	sentence[sentence_length] = '\0';

	if(!nmea_checksum_valid(sentence))
		return ;

	/* Remove the checksum suffix. */
	{
		char *star = strchr(sentence, '*');
		if (star == NULL)
			return ;

		sentence_length = star - sentence;
		sentence[sentence_length] = '\0';
	}

	char* fields[NEO6M_MAX_FIELDS];

	int count = nmea_split(sentence, fields, NEO6M_MAX_FIELDS);
	if (count < 1 || strlen(fields[0]) < 6)
		return ;

	/*
	* Accept different NMEA talker IDs, e.g.:
	* $GPRMC, $GNRMC, $GPGGA, $GNGGA.
	*
	* Sentence type is the final three characters of the ID.
	*/
	const char *id = fields[0];
	size_t id_len = strlen(id);
	if (id[0] != '$' || id_len != 6)
		return ;

	const char *type = id + 3;

	/* ---------------- RMC ---------------- */

	if (strcmp(type, "RMC") == 0)
	{
		if(count < 10)
			return ;

		/*
		* RMC status:
		* A = valid
		* V = invalid
		*/
		if(strcmp(fields[2], "A") != 0)
			return ;

		/*
		* If an NMEA mode field is present, N means no fix.
		* Older NMEA versions may omit this field.
		*/
		if(count > 12 && fields[12][0] != '\0' && fields[12][0] == 'N')
			return ;

		double latitude, longitude;

		if(!nmea_coordinate(fields[3], fields[4][0], true, &latitude) || !nmea_coordinate(fields[5], fields[6][0], false, &longitude))
			return ;

		gps_data->latitude = latitude;
		gps_data->longitude = longitude;
		(*new_position_arrived) = 1;

		/*
		* RMC speed is in knots.
		* Course is clockwise from true north.
		*/
		double speed_knots, heading;

		if(!parse_double(fields[7], &speed_knots) || !parse_double(fields[8], &heading) || speed_knots < 0.0 || heading < 0.0 || heading > 360.0)
			return ;

		gps_data->speed = speed_knots * 0.514444;
		gps_data->heading = heading;
		(*new_velocity_arrived) = 1;

		return ;
	}

	/* ---------------- GGA ---------------- */

	if (strcmp(type, "GGA") == 0)
	{
		if(count < 11)
			return ;

		int fix_quality;

		if(!parse_int(fields[6], &fix_quality) || (fix_quality != 1 && fix_quality != 2))
			return ;

		double latitude, longitude, altitude;

		if(!nmea_coordinate(fields[2], fields[3][0], true, &latitude) || !nmea_coordinate(fields[4], fields[5][0], false, &longitude) || !parse_double(fields[9], &altitude) || strcmp(fields[10], "M") != 0)
			return ;

		gps_data->latitude = latitude;
		gps_data->longitude = longitude;
		(*new_position_arrived) = 1;

		gps_data->altitude = altitude;

		(*new_altitude_arrived) = 1;

		return ;
	}
}