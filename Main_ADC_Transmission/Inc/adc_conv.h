#ifndef adc_conv_h
#define adc_conv_h

#include "main.h"

#define MAX_ADC_VOLTS 3.11f
#define MAX_ADC_BITS 4095.0f
#define VOLTS_PER_BIT (MAX_ADC_VOLTS/MAX_ADC_BITS)

uint32_t process_ADC_result(uint32_t);

#endif