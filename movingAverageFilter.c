/**
 * @file movingAverageFilter.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date October 27th 2013
 * @brief Manages the Low-Pass filters for the 3-axis tilt angles.
*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "movingAverageFilter.h"
#include "cmsis_os.h"

/**
 * Defines the private function of the C file.
*/
void computeXAcceleration();
void computeYAcceleration();
void computeZAcceleration();
void computeTemperature();
void setXValue(int xValue);
void setYValue(int yValue);
void setZValue(int zValue);
void setTemperature(uint32_t averageTemperature);

/**
 * Defines the buffer size used for the moving average filter.
*/
const uint8_t bufferDepth = 10;
const uint8_t temperatureBufferDepth = 30;

/**
 * Defines the buffers used for all axis.
*/
double xAxisBuffer[bufferDepth];
double yAxisBuffer[bufferDepth];
double zAxisBuffer[bufferDepth];
double temperatureBuffer[temperatureBufferDepth];

/**
 * Defines the pointers of the circular buffers used for the moving average filter.
*/
uint8_t xBufferPointer = 0;
uint8_t yBufferPointer = 0;
uint8_t zBufferPointer = 0;
uint8_t temperatureBufferPointer = 0;

/**
 * Defines the average accelerations computed with the aid of their respective buffers.
*/
int averageXAcceleration = 0;
int averageYAcceleration = 0;
int averageZAcceleration = 0;
uint32_t averageTemperature = 0;

/**
 * Defines the different Mutex used in the Moving Average filter for the different components.
*/
osMutexId xAcceleration_mutex;
osMutexDef(xAcceleration_mutex);
osMutexId yAcceleration_mutex;
osMutexDef(yAcceleration_mutex);
osMutexId zAcceleration_mutex;
osMutexDef(zAcceleration_mutex);
osMutexId temperature_mutex;
osMutexDef(temperature_mutex);

/**
 * Initializes all the mutex.
*/
void initializeFilter() {
	xAcceleration_mutex = osMutexCreate(osMutex(xAcceleration_mutex));
	yAcceleration_mutex = osMutexCreate(osMutex(yAcceleration_mutex));
	zAcceleration_mutex = osMutexCreate(osMutex(zAcceleration_mutex));
	temperature_mutex = osMutexCreate(osMutex(temperature_mutex));
}

/**
 * Adds a 
 *
 * @param The 
*/
void addToTemperatureBuffer(uint32_t temperatureInCelcius) {	
	// Adds the temperature at the end of the circular buffer.
	temperatureBuffer[temperatureBufferPointer] = temperatureInCelcius;
	
	// Increments the pointer in the circular buffer and sets it back to 0 if it overbounds.
	if (temperatureBufferPointer == (temperatureBufferDepth-1)) {
		temperatureBufferPointer = 0;
	} else {
		temperatureBufferPointer++;
	}
	
	computeTemperature();
}

/**
 * Computes the average temperature based on the values stored in the buffer.
*/
void computeTemperature() {
	int sum = 0;
	uint8_t i = 0;
	for (i = 0; i < temperatureBufferDepth; i++){
		sum += temperatureBuffer[i];
	}
	int averageTemperature = sum / temperatureBufferDepth;
	
	setTemperature(averageTemperature);
}

/**
 * Sets the temperature average.
 * 
 * @param The temperature average.
*/
void setTemperature(uint32_t temperature) {
	osMutexWait(temperature_mutex, osWaitForever);
	averageTemperature = temperature;
	osMutexRelease(temperature_mutex);
}

/**
 * Returns the temperature average.
 *
 * @return The temperature average.
*/
uint32_t getTemperatureAverage() {
	osMutexWait(temperature_mutex, osWaitForever);
	uint32_t temp = averageTemperature;
	osMutexRelease(temperature_mutex);
	return temp;
}

/**
 * Adds an acceleration reading the the X-axis buffer.
 *
 * @param The acceleration to be added.
*/
void addToXBuffer(int angle){
	// Adds the angle at the end of the circular buffer.
	xAxisBuffer[xBufferPointer] = angle;
	
	// Increments the pointer in the circular buffer and sets it back to 0 if it overbounds.
	if (xBufferPointer == (bufferDepth-1)) {
		xBufferPointer = 0;
	} else {
		xBufferPointer++;
	}
	
	computeXAcceleration();
}

/**
 * Computes the average acceleration based on the values stored in the buffer.
*/
void computeXAcceleration() {
	int sum = 0;
	uint8_t i = 0;
	for (i = 0; i < bufferDepth; i++){
		sum += xAxisBuffer[i];
	}
	int averageXAcceleration = sum / bufferDepth;
	
	setXValue(averageXAcceleration);
}

/**
 * Sets the averageXAcceleration.
 * 
 * @param The averageXAcceleration.
*/
void setXValue(int xValue) {
	osMutexWait(xAcceleration_mutex, osWaitForever);
	averageXAcceleration = xValue;
	osMutexRelease(xAcceleration_mutex);
}

/**
 * Returns the average value of the moving average filter for the X-axis.
 *
 * @return The average acceleration on the X-axis.
*/
int getXValue(){
	osMutexWait(xAcceleration_mutex, osWaitForever);
	int temp = averageXAcceleration;
	osMutexRelease(xAcceleration_mutex);
	return temp;
}

/**
 * Adds an acceleration reading the the Y-axis buffer.
 *
 * @param The acceleration to be added.
*/
void addToYBuffer(int angle){
	// Adds the angle at the end of the circular buffer.
	yAxisBuffer[yBufferPointer] = angle;
	
	// Increments the pointer in the circular buffer and sets it back to 0 if it overbounds.
	if (yBufferPointer == (bufferDepth-1)) {
		yBufferPointer = 0;
	} else {
		yBufferPointer++;
	}
	
	computeYAcceleration();
}

/**
 * Computes the average acceleration based on the values stored in the buffer.
*/
void computeYAcceleration() {
	int sum = 0;
	uint8_t i = 0;
	for (i = 0; i < bufferDepth; i++){
		sum += yAxisBuffer[i];
	}
	int averageYAcceleration = sum / bufferDepth;
	
	setYValue(averageYAcceleration);
}

/**
 * Sets the averageYAcceleration.
 * 
 * @param The averageYAcceleration.
*/
void setYValue(int yValue) {
	osMutexWait(yAcceleration_mutex, osWaitForever);
	averageYAcceleration = yValue;
	osMutexRelease(yAcceleration_mutex);
}

/**
 * Returns the average value of the moving average filter for the Y-axis.
 *
 * @return The average acceleration on the Y-axis.
*/
int getYValue(){
	osMutexWait(yAcceleration_mutex, osWaitForever);
	int temp = averageYAcceleration;
	osMutexRelease(yAcceleration_mutex);
	return temp;
}

/**
 * Adds an acceleration reading the the Z-axis buffer.
 *
 * @param The acceleration to be added.
*/
void addToZBuffer(int angle){
	// Adds the angle at the end of the circular buffer.
	zAxisBuffer[zBufferPointer] = angle;
	
	// Increments the pointer in the circular buffer and sets it back to 0 if it overbounds.
	if (zBufferPointer == (bufferDepth-1)) {
		zBufferPointer = 0;
	} else {
		zBufferPointer++;
	}
	
	computeZAcceleration();
}

/**
 * Computes the average acceleration based on the values stored in the buffer.
*/
void computeZAcceleration() {
	int sum = 0;
	uint8_t i = 0;
	for (i = 0; i < bufferDepth; i++){
		sum += zAxisBuffer[i];
	}
	int averageZAcceleration = sum / bufferDepth;
	
	setZValue(averageZAcceleration);
}

/**
 * Sets the averageZAcceleration.
 * 
 * @param The averageZAcceleration.
*/
void setZValue(int zValue) {
	osMutexWait(zAcceleration_mutex, osWaitForever);
	averageZAcceleration = zValue;
	osMutexRelease(zAcceleration_mutex);
}

/**
 * Returns the average value of the moving average filter for the Z-axis.
 *
 * @return The average acceleration on the Z-axis.
*/
int getZValue(){
	osMutexWait(zAcceleration_mutex, osWaitForever);
	int temp = averageZAcceleration;
	osMutexRelease(zAcceleration_mutex);
	return temp;
}
