#include "glucose_packet.h"

//uint8_t* check_glucose_data(uint32_t data) 
//{
//	if(data <= DATA_UPPER && data >= DATA_LOWER)
//	{
//		return (uint8_t*)"OK";
//	}
//	else
//	{
//		return (uint8_t*)"NO";
//	}
//}

GlucoseData pack_glucose_data(uint32_t glucose_data)
{
	GlucoseData data_packet;
	//if (strcmp((const char*)glucose_validity, "OK"))
	//{
		data_packet.glucose_msb = (glucose_data & MSB) >> MSB_shift;
		data_packet.glucose_upper_middle = (glucose_data & UPPER_MIDDLE) >> UM_shift;
		data_packet.glucose_lower_middle = (glucose_data & LOWER_MIDDLE) >> LM_shift;
		data_packet.glucose_lsb = (glucose_data & LSB) >> LSB_shift;
		return data_packet;
		//exit(0);
	//}
	//else
	//{
		exit(-1);
	//}
}

uint8_t* prepare_data_buffer(GlucoseData data)
{
	uint8_t *data_buffer = malloc(4 * sizeof(uint8_t));
	data_buffer[0] = data.glucose_msb;
	data_buffer[1] = data.glucose_upper_middle;
	data_buffer[2] = data.glucose_lower_middle;
	data_buffer[3] = data.glucose_lsb;
	return data_buffer;
}

GlucosePacket pack_glucose_packet(uint8_t* data_buffer)
{
	GlucosePacket packet;
	packet.sync_bytes[0] = 'R';
	packet.sync_bytes[1] = 'O';
	packet.sync_bytes[2] = 'B';
	packet.data = malloc(sizeof(data_buffer));
	packet.data = data_buffer;
	packet.checksum = 0; //placeholder
	return packet;
}

uint8_t* create_uart_buffer(GlucosePacket packet)
{
	uint8_t* uart_buffer = malloc(7 * sizeof(uint8_t));
	uart_buffer[0] = packet.sync_bytes[0];
	uart_buffer[1] = packet.sync_bytes[1];
	uart_buffer[2] = packet.sync_bytes[2];
	uart_buffer[3] = packet.data[0];
	uart_buffer[4] = packet.data[1];
	uart_buffer[5] = packet.data[2];
	uart_buffer[6] = packet.data[3];
	return uart_buffer;
}

void free_buffers(uint8_t* buffer)
{
	free(buffer);
}
