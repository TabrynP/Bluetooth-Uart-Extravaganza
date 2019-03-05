/********************************************************************************
*	Tabryn Palmer																																	*
*	11/26/18																																			*
*	ECE 415 - Subcutaneous Continuous Glucose Monitor															*
*																																								*
*	Data types and functions for transferring of glucose monitor data over UART.  *
* Contains platform independent data structure for syncing and verifying				*
* CGM data.																																			*
*																																								*
********************************************************************************/

#ifndef glucose_packet_h
#define glucose_packet_h

#include <string.h>
#include <stdlib.h>

#include "stm32l4xx_hal.h"

// Sync bytes, will signify to the host machine that glucose data is incoming.
#define SYNC_BYTES (uint8_t*){'R', 'O', 'B'}

// Upper and lower bounds for acceptable glucose voltage reading.
#define DATA_UPPER 3000
#define DATA_LOWER 2000

// Bit masks for parsing a 32-bit value into four 8-bit values.
#define MSB 0xFF000000
#define UPPER_MIDDLE 0x00FF0000
#define LOWER_MIDDLE 0x0000FF00
#define LSB 0x000000FF

// Logical shift values for 32-bit parsing.
#define MSB_shift 24
#define UM_shift 16
#define LM_shift 8
#define LSB_shift 0

// Actual content of a packet of glucose monitor data. Contains sync bytes,
// checksum, and pointer to data buffer.
typedef struct glucose_packet_t 
{
	uint8_t sync_bytes[3];
	uint8_t* data;
	uint32_t checksum;
	
}GlucosePacket;

// Data format of glucose monitor data.
typedef struct glucose_data_t 
{
	uint8_t glucose_msb;
	uint8_t glucose_upper_middle;
	uint8_t glucose_lower_middle;
	uint8_t glucose_lsb;
}GlucoseData;

// Ensures the glucose data is in the proper format before passing it to the
// GlucoseData structure.
uint8_t* check_glucose_data(uint32_t);

// Takes formatted ADC glucose data, breaks it into four significate bytes
// and passes them to the glucose data structure.
GlucoseData pack_glucose_data(uint32_t);

// Prepares a data buffer for uart transmission based on passed glucose data
uint8_t* prepare_data_buffer(GlucoseData);

// Takes formatted glucose data and packs it into a glucose packet.
GlucosePacket pack_glucose_packet(uint8_t*);

// Creates buffer for uart transmission based on data buffer and glucose packet
uint8_t* create_uart_buffer(GlucosePacket);

#endif