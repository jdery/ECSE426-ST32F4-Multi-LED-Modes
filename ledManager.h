#ifndef LEDMANAGER_H
#define LEDMANAGER_H

#include "stm32f4xx.h"

void initializeLED();
void processLEDStates();
void invertLED12();
void invertLED13();
void invertLED14();
void invertLED15();
void turnOffLED12();
void turnOffLED13();
void turnOffLED14();
void turnOffLED15();
void turnOnLED12();
void turnOnLED13();
void turnOnLED14();
void turnOnLED15();
void enableGPIOForFrequencyVerification();
void inverseGPIOPinForFrequencyVerification();

#endif