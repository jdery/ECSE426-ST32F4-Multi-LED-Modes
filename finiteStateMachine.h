#ifndef FINITESTATEMACHINE_H
#define FINITESTATEMACHINE_H

#include "stm32f4xx.h"

typedef enum {ACCELEROMETER_NORMAL_STATE, ACCELEROMETER_PWM_STATE, TEMPERATURE_NORMAL_STATE, TEMPERATURE_PWM_STATE} fsmState;
typedef enum {TOP, RIGHT, BOTTOM, LEFT} activeLED;

void initializeFSM();
void switchCurrentState();
void switchInternalState();
void processCurrentState();

#endif