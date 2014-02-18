/**
 * @file ledManager.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date October 31th 2013
 * @brief Manages all the LED functionalities.
*/

#include <stdio.h>
#include "ledManager.h"

// Defines the boolean used to control the state of the LEDs.
uint8_t isLed12On = 0;
uint8_t isLed13On = 0;
uint8_t isLed14On = 0;
uint8_t isLed15On = 0;
uint8_t isPinEnabled = 0;

/**
 * Turns the state of LED 12 to off.
*/
void turnOffLED12() {
	isLed12On = 0;
}

/**
 * Turns the state of LED 13 to off.
*/
void turnOffLED13() {
	isLed13On = 0;
}

/**
 * Turns the state of LED 14 to off.
*/
void turnOffLED14() {
	isLed14On = 0;
}

/**
 * Turns the state of LED 15 to off.
*/
void turnOffLED15() {
	isLed15On = 0;
}

/**
 * Turns the state of LED 12 to off.
*/
void turnOnLED12() {
	isLed12On = 1;
}

/**
 * Turns the state of LED 13 to off.
*/
void turnOnLED13() {
	isLed13On = 1;
}

/**
 * Turns the state of LED 14 to off.
*/
void turnOnLED14() {
	isLed14On = 1;
}

/**
 * Turns the state of LED 15 to off.
*/
void turnOnLED15() {
	isLed15On = 1;
}

/**
 * Inverts the state of LED 12.
*/
void invertLED12() {
	if (isLed12On) {
		isLed12On = 0;
	} else {
		isLed12On = 1;
	}
}

/**
 * Inverts the state of LED 13.
*/
void invertLED13() {
	if (isLed13On) {
		isLed13On = 0;
	} else {
		isLed13On = 1;
	}
}

/**
 * Inverts the state of LED 14.
*/
void invertLED14() {
	if (isLed14On) {
		isLed14On = 0;
	} else {
		isLed14On = 1;
	}
}

/**
 * Inverts the state of LED 15.
*/
void invertLED15() {
	if (isLed15On) {
		isLed15On = 0;
	} else {
		isLed15On = 1;
	}
}

/**
 * Process the states of the LEDs. If an LED is set to be on, it turns that LED on. The inverse happen when it is set to be false.
*/
void processLEDStates() {
	// In here we are playing with the TIM4 Capture Compare register because this is where the value of the 
	// Duty Cycle is stored. 
	
	if (isLed12On) {
		TIM4->CCR1 = 1000;
	} else {
		TIM4->CCR1 = 0;
	}
	
	if (isLed13On) {
		TIM4->CCR2 = 1000;
	} else {
		TIM4->CCR2 = 0;
	}
	
	if (isLed14On) {
		TIM4->CCR3 = 1000;
	} else {
		TIM4->CCR3 = 0;
	}
	
	if (isLed15On) {
		TIM4->CCR4 = 1000;
	} else {
		TIM4->CCR4 = 0;
	}
}

/**
 * Enables the GPIO pin to be used to confirm that the interrupt frequency is 25 Hz.
*/
void enableGPIOForFrequencyVerification() {
	GPIO_InitTypeDef gpio_init_s;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_StructInit(&gpio_init_s);
  gpio_init_s.GPIO_Pin = GPIO_Pin_12;
  gpio_init_s.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init_s.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init_s.GPIO_OType = GPIO_OType_PP;
  gpio_init_s.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &gpio_init_s);
}

/**
 * Inverses the GPIO pin state used to validate the frequency.
*/
void inverseGPIOPinForFrequencyVerification() {
	if (isPinEnabled) {
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
		isPinEnabled = 0;
	} else {
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
		isPinEnabled = 1;
	}
}