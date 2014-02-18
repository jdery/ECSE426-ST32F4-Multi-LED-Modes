/**
 * @file timerManager.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date October 25th 2013
 * @brief Manages all the interactions with the on-board TIM4 timer's peripheral.
*/

#include <stdio.h>
#include "accelerometerManager.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "timerManager.h"

void activateTIM5Interrupt();
void activateTIM3Interrupt();
void activateTIM2Interrupt();

/**
 * Initializes the on-board TIM3 timer at a frequency of 25 Hz, as per lab requirement.
*/
void initializeTIM3Timer() {
	// To have information about the Nested Vectored Interrupt Controller go to Reference Manual page 195.
	// TIM2 to TIM5 main features explained in Reference Manual at page 361.
	// To have information about how the hardware interrupts work on the STM32f4 go to http://visualgdb.com/tutorials/arm/stm32/timers/
	// Another example available here http://amarkham.com/?p=29
	
	activateTIM3Interrupt();
	
	// Enables the clock for the TIM3 timer so we can use it.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// Creates the Structure that will contain all the desired parameters.
	TIM_TimeBaseInitTypeDef timer3InitParameters; 
	// The TIM_Prescaler multiplied with the Clock Period will be the time between each incrementation of the counter used to generate interrupts.
	// The total time between interrupts will be given by (TIM_Prescaler*TIM_Period*(Clock Period)) which leads to a frequency at which the 
	// interrupts will happen to be equal to: Clock Frequency/(TIM_Prescaler*TIM_Period) that is given in the Lab Experiment requirements.
	// This value has been taken arbitrarily, and TIM_Period was adjusted according to it.
	// When defining the prescaler remember that the speed is 42 Mhz (as presented on the STM32F40x Block Diagram schematic).
	timer3InitParameters.TIM_Prescaler = 40000;
	// Sets to up because we want it to count from 0 to the specified value (and generate an interrupt when the desired value is reached).
	timer3InitParameters.TIM_CounterMode = TIM_CounterMode_Up;
	// This is the upper bound of the counter. When it reaches this value it will generate an interrupt and reset itself to 0.
	timer3InitParameters.TIM_Period = 42;
	timer3InitParameters.TIM_ClockDivision = 0;
	// Initializes the TIM3 timer with the specified parameters.
	TIM_TimeBaseInit(TIM3, &timer3InitParameters);
	// Enable the interrupt generation for the TIM3 timer.
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	// Activates the TIM3 timer.
	TIM_Cmd(TIM3, ENABLE);
}

/**
 * Setup the interrupt on the Nested Vectored Interrupt Controller for the TIM3 timer.
*/
void activateTIM3Interrupt() {
	NVIC_InitTypeDef nvicStructure;
	// The IRQ channel is going to be TIM3 since this is the timer that we are using.
  nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	// The NVIC provides a Group Priority and a Sub Priority within this Group.
	// Link to this information: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/BABHGEAJ.html
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
	// Enables the IRQ channel.
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	// Actually enable the IRQ channel with NVIC.
  NVIC_Init(&nvicStructure);
}

void initializeTIM2Timer() {
	// To have information about the Nested Vectored Interrupt Controller go to Reference Manual page 195.
	// TIM2 to TIM5 main features explained in Reference Manual at page 361.
	// To have information about how the hardware interrupts work on the STM32f4 go to http://visualgdb.com/tutorials/arm/stm32/timers/
	// Another example available here http://amarkham.com/?p=29
	
	activateTIM2Interrupt();
	
	// Enables the clock for the TIM3 timer so we can use it.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	// Creates the Structure that will contain all the desired parameters.
	TIM_TimeBaseInitTypeDef timer2InitParameters;
	// The TIM_Prescaler multiplied with the Clock Period will be the time between each incrementation of the counter used to generate interrupts.
	// The total time between interrupts will be given by (TIM_Prescaler*TIM_Period*(Clock Period)) which leads to a frequency at which the 
	// interrupts will happen to be equal to: Clock Frequency/(TIM_Prescaler*TIM_Period) that is given in the Lab Experiment requirements.
	// This value has been taken arbitrarily, and TIM_Period was adjusted according to it.
	// When defining the prescaler remember that the speed is 42 Mhz (as presented on the STM32F40x Block Diagram schematic).
	timer2InitParameters.TIM_Prescaler = 40000;
	// Sets to up because we want it to count from 0 to the specified value (and generate an interrupt when the desired value is reached).
	timer2InitParameters.TIM_CounterMode = TIM_CounterMode_Up;
	// This is the upper bound of the counter. When it reaches this value it will generate an interrupt and reset itself to 0.
	timer2InitParameters.TIM_Period = 20;
	timer2InitParameters.TIM_ClockDivision = 0;
	// Initializes the TIM2 timer with the specified parameters.
	TIM_TimeBaseInit(TIM2, &timer2InitParameters);
	// Enable the interrupt generation for the TIM2 timer.
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	// Activates the TIM2 timer.
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * Setup the interrupt on the Nested Vectored Interrupt Controller for the TIM2 timer.
*/
void activateTIM2Interrupt() {
	NVIC_InitTypeDef nvicStructure;
	// The IRQ channel is going to be TIM2 since this is the timer that we are using.
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	// The NVIC provides a Group Priority and a Sub Priority within this Group.
	// Link to this information: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/BABHGEAJ.html
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
	// Enables the IRQ channel.
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	// Actually enable the IRQ channel with NVIC.
  NVIC_Init(&nvicStructure);
}

void initializeTIM5Timer() {
	// To have information about the Nested Vectored Interrupt Controller go to Reference Manual page 195.
	// TIM2 to TIM5 main features explained in Reference Manual at page 361.
	// To have information about how the hardware interrupts work on the STM32f4 go to http://visualgdb.com/tutorials/arm/stm32/timers/
	// Another example available here http://amarkham.com/?p=29
	
	activateTIM5Interrupt();
	
	// Enables the clock for the TIM3 timer so we can use it.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	// Creates the Structure that will contain all the desired parameters.
	TIM_TimeBaseInitTypeDef timer5InitParameters;
	// The TIM_Prescaler multiplied with the Clock Period will be the time between each incrementation of the counter used to generate interrupts.
	// The total time between interrupts will be given by (TIM_Prescaler*TIM_Period*(Clock Period)) which leads to a frequency at which the 
	// interrupts will happen to be equal to: Clock Frequency/(TIM_Prescaler*TIM_Period) that is given in the Lab Experiment requirements.
	// This value has been taken arbitrarily, and TIM_Period was adjusted according to it.
	// When defining the prescaler remember that the speed is 42 Mhz (as presented on the STM32F40x Block Diagram schematic).
	timer5InitParameters.TIM_Prescaler = 40000;
	// Sets to up because we want it to count from 0 to the specified value (and generate an interrupt when the desired value is reached).
	timer5InitParameters.TIM_CounterMode = TIM_CounterMode_Up;
	// This is the upper bound of the counter. When it reaches this value it will generate an interrupt and reset itself to 0.
	timer5InitParameters.TIM_Period = 50;
	timer5InitParameters.TIM_ClockDivision = 0;
	// Initializes the TIM2 timer with the specified parameters.
	TIM_TimeBaseInit(TIM5, &timer5InitParameters);
	// Enable the interrupt generation for the TIM2 timer.
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	// Activates the TIM2 timer.
	TIM_Cmd(TIM5, ENABLE);
}

/**
 * Setup the interrupt on the Nested Vectored Interrupt Controller for the TIM2 timer.
*/
void activateTIM5Interrupt() {
	NVIC_InitTypeDef nvicStructure;
	// The IRQ channel is going to be TIM2 since this is the timer that we are using.
  nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
	// The NVIC provides a Group Priority and a Sub Priority within this Group.
	// Link to this information: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/BABHGEAJ.html
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
	// Enables the IRQ channel.
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	// Actually enable the IRQ channel with NVIC.
  NVIC_Init(&nvicStructure);
}

/**
 * Setup the GPIO to work with the TIM4.
*/
void setupGPIOForTIM4() {
	// Setup the GPIO pins of the LEDs to work witht the TIM4 timer.
	GPIO_InitTypeDef GPIO_InitStructure;
	// Enables the clock for GPIOD and TIM4.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// Setup the pin source of the LEDs.
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	// The GPIO mode is set to Alternating Frequency so we can play with the Duty Cycle.
	// If it was to GPIO_Mode_OUT, then we would not be able to set a Duty Cycle using the TIM4 timer.
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	// GPIOD, because this is the one used for the LEDs.
	// Sets all the Pin numbers that will be connected to TIM4.
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
}

/**
 * Setup the TIM4 timer to work with the LEDs.
*/
void initializeTIM4Timer() {
	// Code example: https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/DispForm.aspx?ID=1816&Source=/public/STe2ecommunities/Tags.aspx?tags=stm32f4
	// Some explanation about this: http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/pwm
	
	//printf("[INFO] initializeTIM4Timer()\n");
	
	setupGPIOForTIM4();
	
	// Setup the PWM functionality with the TIM4 timer.
	uint16_t prescaler = (uint16_t) 100;
	// This is the same kind of configuration as with the TIM3 timer used for the accelerometer.
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 1000;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// So once we have inicialized the TIM4 time-based configuration, we need to configure the different channels. That will be connected to the
	// different LED pin numbers.
	// So the way this is working is that the register CCRx will be loaded at everytime the counter will overflow. Then, the PIN will on ON during
	// the Duty Cycle set in the register, then it will be turned off.
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	// Sets up PWM for Channel 1.
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	// Sets up PWM for Channel 2.
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	// Sets up PWM for Channel 3.
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	// Sets up PWM for Channel 4.
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	// Enabling the TIM4 counter, and starting the machine.
	TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}