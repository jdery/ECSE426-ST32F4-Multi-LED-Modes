#ifndef ACCELEROMETERMANAGER_H
#define ACCELEROMETERMANAGER_H

#include "stm32f4xx.h"

void initializeAccelerometer();
void getAngles(int* allAxisAngles);
void fetchAllAxisAccelerations();
void activateSingleTapDetectionInterrupt();
void activateButtonInterrupt();

#endif