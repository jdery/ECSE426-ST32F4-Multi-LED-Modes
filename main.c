/**
 * @file main.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date November 6th 2013
 * @brief This C file will contain all the main flow of the program and the initialization of all the threads.
*/

#include "arm_math.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery_lis302dl.h"
#include "accelerometerManager.h"
#include "timerManager.h"
#include "finiteStateMachine.h"
#include "ledManager.h"
#include "adcManager.h"
#include "movingAverageFilter.h"

// Resources:
// http://mbed.org/handbook/CMSIS-RTOS

/**
 @brief Thread that will perform the readings on 
 @param argument Unused
 */
void temperatureThread(void const *argument);
void accelerometerThread(void const *argument);
void mainThread(void const *argument);
// "Man in the middle threads"
void buttonSwitchThread(void const *argument);
void tapSwitchThread(void const *argument);

void preamble();

// Defines the thread structure for all above threads.
osThreadDef(temperatureThread, osPriorityNormal, 1, 0);
osThreadDef(accelerometerThread, osPriorityNormal, 1, 0);
osThreadDef(mainThread, osPriorityNormal, 1, 0);
// "Man in the middle threads"
osThreadDef(buttonSwitchThread, osPriorityNormal, 1, 0);
osThreadDef(tapSwitchThread, osPriorityNormal, 1, 0);

osThreadId tid_temperatureThread, tid_accelerometerThread, tid_mainThread, tid_buttonSwitchThread, tid_tapSwitchThread;

/**
 @brief Program entry point
 */
int main(void) {
	preamble();
}

/**
 * Handles the interrupts generated by the TIM2 timer.
*/
void TIM2_IRQHandler() {
	// Clears the interrupt flag since we are caching it right now.
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	osSignalSet(tid_mainThread, 1);
}

/**
 * Handles the interrupts generated by the TIM3 timer.
 *
 * This is where we read the acceleration from the 3-axis accelerometer at a rate of 25 Hz.
*/
void TIM3_IRQHandler() {
	// Clears the interrupt flag since we are caching it right now.
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	// Since it is an hardware timer, it will continue running when we are in debug, which can result in weird frequencies when
	// trying to verify it. We cannot use the debugger to verify the frequency like we did with the SysTick timer.
	osSignalSet(tid_accelerometerThread, 3);
}

/**
 * Handles the interrupts generated by the TIM5 timer.
 *
 * This is where we read the temperature from the temperature sensor at a frequency of 20 Hz.
*/
void TIM5_IRQHandler() {
	// Clears the interrupt flag since we are caching it right now.
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	osSignalSet(tid_temperatureThread, 2);
}

/**
 * Handles the button interrupt and switches the internal state of the program.
*/
void EXTI0_IRQHandler(void) {
	// Clears the interrupt bit that was set by the accelerometer.
	EXTI_ClearITPendingBit(EXTI_Line0);
	// Switches the internal state of the FSM.
	osSignalSet(tid_buttonSwitchThread, 4);
}

/**
 * Handles the interrupts generated by the Single Tap detection on the accelerometer.
 *
 * This is where we switch between the two available states.
*/
void EXTI1_IRQHandler(void) {
	// Clears the interrupt bit that was set by the accelerometer.
	EXTI_ClearITPendingBit(EXTI_Line1);
	// Switches the current state of the FSM.
	osSignalSet(tid_tapSwitchThread, 5);
}

/**
 * Main's preamble where are the initializations are done.
*/
void preamble() {
	// Initializes the Mutex used in the Moving Average Filter.
	initializeFilter();
	// Initializes the Finite State Machine with the default state being ACCELEROMETER_NORMAL_STATE.
	initializeFSM();
	
	// Does all the preamble stuff.
	initializeAccelerometer();
	configureADC();
	// Does all the magic needed for the processor to catch the interrupts generated by this external peripheral.
	activateSingleTapDetectionInterrupt();
	// Activates the interrupt on EXTI0 for the button.
	activateButtonInterrupt();
	// Initializes the TIM4 timer so that it controls the Duty Cycle of the LEDs.
	initializeTIM4Timer();
	// Initializes the TIM3 timer so that it controls the frequency at which the accelerations are fetched.
	initializeTIM3Timer();
	// Initializes the TIM2 timer so that it controls the frequency at which the main will be executed.
	initializeTIM2Timer();
	// Initializes the TIM5 timer so that it controls the frequency at which the temperature will be fetched.
	initializeTIM5Timer();
	
	// Initializes all the required threads for the continuous operation.
	tid_temperatureThread = osThreadCreate(osThread(temperatureThread), NULL);
	tid_accelerometerThread = osThreadCreate(osThread(accelerometerThread), NULL);
	tid_mainThread = osThreadCreate(osThread(mainThread), NULL);
	
	tid_buttonSwitchThread = osThreadCreate(osThread(buttonSwitchThread), NULL);
	tid_tapSwitchThread = osThreadCreate(osThread(tapSwitchThread), NULL);
}

/**
 * Defines the main thread that will process the continuous run.
*/
void mainThread(void const *argument) {
	// Fetches the data from the temperature sensor.
	while (1){
		osSignalWait(1, osWaitForever);
		processCurrentState();
	}
}

/**
 * Defines the thread that will fetch the temperature from the temperature sensor.
*/
void temperatureThread(void const *argument) {
	// Fetches the data from the temperature sensor.
	while (1){
		osSignalWait(2, osWaitForever);
		acquireADCValue();
	}
}

/**
 * Defines the thread that will fetch the accelerations from the accelerometer.
*/
void accelerometerThread(void const *argument) {
	// Fetches the data from the accelerometer at a frequency of 25 Hz.
	while (1){
		osSignalWait(3, osWaitForever);
		fetchAllAxisAccelerations();
	}
}

/**
 * Defines the thread that will switch the mode when the push-button is pressed.
*/
void buttonSwitchThread(void const *argument) {
	while (1){
		osSignalWait(4, osWaitForever);
		switchInternalState();
	}
}

/**
 * Defines the thread that will switch the mode when the Single-Tap is detected.
*/
void tapSwitchThread(void const *argument) {
	while (1){
		osSignalWait(5, osWaitForever);
		switchCurrentState();
	}
}