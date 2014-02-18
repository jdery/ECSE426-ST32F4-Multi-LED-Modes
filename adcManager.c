/**
 * @file adc.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date November 9th 2013
 * @brief Manages all the ADC stuff with the temperature sensor.
*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "adcManager.h"
#include "movingAverageFilter.h"

/**
 * Global variables related to the ADC
*/
uint32_t temperatureInCelsius;
uint16_t ADC_output;

/**
 * Variables defined (Datasheet p.134) in the temperature formula (Reference Manual p.230) user to convert the
 * output from the ADC to actual temperature.
*/
const float avg_Slope = 0.0025;
const float v25 = 0.76;
const uint32_t samplingTime = 15;
const uint32_t celsiusBase = 25;

/**
 * Returns the value in celsius based on the 12-bit input (taken from the ADC).
 * This formula can be found in the reference manual at page 230.
*/
uint32_t getCelsius(uint16_t adcIn) {
	return (((3*((float)adcIn / 4096) - v25) / avg_Slope) + celsiusBase);
}

/**
 * Configures the ADC on channel 16 so we can get data from the temperature sensor.
*/
void configureADC() {	
	ADC_InitTypeDef adc_init_s;
	ADC_CommonInitTypeDef adc_common_init_s;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	adc_common_init_s.ADC_Mode = ADC_Mode_Independent;
	// Sets the frequency of the ADC, smallest freq available.
	adc_common_init_s.ADC_Prescaler = ADC_Prescaler_Div2;
	// Smallest possible delay.
	adc_common_init_s.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	// Initializes the ADC with the specified parameters in adc_common_init_s.
	ADC_CommonInit(&adc_common_init_s);
	
	// Sets the ADC resolution to 12 bits.
	adc_init_s.ADC_Resolution = ADC_Resolution_12b;
	// The conversion is going to be on only one channel, so we set it to DISABLE.
	adc_init_s.ADC_ScanConvMode = DISABLE;
	
	//adc_init_s.ADC_ContinuousConvMode = DISABLE;
	adc_init_s.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc_init_s.ADC_DataAlign = ADC_DataAlign_Right;
	//adc_init_s.ADC_NbrOfConversion = 1;
	
	ADC_Init(ADC1, &adc_init_s);
	ADC_Cmd(ADC1, ENABLE);
	
	// Wake up the temperature sensor on the board (else we won't be able to use it).
	ADC_TempSensorVrefintCmd(ENABLE);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_480Cycles);
	 
	// Starts the ADC convertion for ADC1.
	ADC_SoftwareStartConv(ADC1);
	// We enable the convertion in a continuous mode in order to prevent the "ADC_SoftwareStartConv()"
	// to be called on each time we want to read from the ADC.
	ADC_ContinuousModeCmd(ADC1, ENABLE);
}

/**
 * Reads the temp sensor with ADC, make it a useful value (deg C), add to the filter, see if the temp is changing, update leds
*/
void acquireADCValue() {
	// The while loop is present because we need the data to be ready before reading it.
	// When the data is ready to be read, the flag will be set for ADC1.
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	// Since we know that the data is ready, we can reset the flag for ADC1.
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	// Gets the value from the ADC.
	ADC_output = ADC_GetConversionValue(ADC1);
	// Converts the reading from the ADC to actual degree celsius.
	temperatureInCelsius = getCelsius(ADC_output);
	// Adds the new temperature to the moving average filter.
	addToTemperatureBuffer(temperatureInCelsius);
}