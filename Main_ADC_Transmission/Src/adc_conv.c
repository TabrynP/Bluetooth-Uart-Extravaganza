#include "adc_conv.h"

uint32_t process_ADC_result(uint32_t ADC_VAL)
{
	float out_float = ADC_VAL * VOLTS_PER_BIT;
	uint32_t output_voltage = (uint32_t)(out_float * 1000);
	out_float = out_float * 1000;
	uint32_t output = (uint32_t)out_float;
	
	return output;
}