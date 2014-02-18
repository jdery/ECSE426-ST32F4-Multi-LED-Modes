/**
 * @file finiteStateMachine.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date October 31th 2013
 * @brief Defines the Finite State Machine used for the continuous run, along with all its interactions with the different components.
*/

#include <stdio.h>
#include "accelerometerManager.h"
#include "finiteStateMachine.h"
#include "ledManager.h"
#include "movingAverageFilter.h"
#include "cmsis_os.h"

// Mutex used to protect the currentState.
osMutexId currentState_mutex;
osMutexDef(currentState_mutex);

void processAccelerometerNormalState();
void processAccelerometerPWMState();
void processTemperatureNORMALState();
void processTemperaturePWMState();
void cwLeds();
void ccwLeds();
fsmState getCurrentState();

fsmState currentState;
activeLED currentLED;

// Defines the constants used for the boundaries in the NORMAL operation mode.
const int UPPER_BOUND = 90;
const int LOWER_BOUND = -UPPER_BOUND;
const int MIDDLE_UPPER_BOUND = 45;
const int MIDDLE_LOWER_BOUND = -MIDDLE_UPPER_BOUND;
const int UPPER_DEAD = 5;
const int LOWER_DEAD = -UPPER_DEAD;
const int MAX_PWM_DUTY_CYCLE = 1000;

// Defines the boundaries of the counters used to flash the LEDs.
const uint32_t UPPER_COUNTER_BOUND = 10;
const uint32_t MIDDLE_COUNTER_BOUND = 20;

// Counters used for the NORMAL operation state.
int pitchCounter = 0;
int rollCounter = 0;

// Variable used for the PWM mode on the LED.
int ledBrightness = MAX_PWM_DUTY_CYCLE;
uint32_t prevFilterAvg = 0;
int deviation = 0;

/**
 * Returns the currentState.
*/
fsmState getCurrentState() {
	return (currentState);	
}

/**
 * Initializes the Finite State Machine used in the continuous run.
*/
void initializeFSM() {
	//printf("[INFO] initializeFSM()\n");
	currentState = ACCELEROMETER_NORMAL_STATE;
	currentLED = TOP;
	currentState_mutex = osMutexCreate(osMutex(currentState_mutex));
}

/**
 * Switches the current state of the Finite State Machine.
 *
 * If the state is in one of the ACCELEROMETER states, it will be swithed to TEMPERATURE_NORMAL_STATE.
 * If the state is in one of the TEMPERATURE states, it will be swithed to ACCELEROMETER_NORMAL_STATE.
*/
void switchCurrentState() {
	osMutexWait(currentState_mutex, osWaitForever);
	if (currentState ==  ACCELEROMETER_NORMAL_STATE || currentState ==  ACCELEROMETER_PWM_STATE) {
		currentState = TEMPERATURE_NORMAL_STATE;
	} else if (currentState ==  TEMPERATURE_NORMAL_STATE || currentState ==  TEMPERATURE_PWM_STATE) {
		currentState = ACCELEROMETER_NORMAL_STATE;
	}
	osMutexRelease(currentState_mutex);
}

/**
 * Switches the internal state of the FSM.
 *
 * If the currentState is ACCELEROMETER_NORMAL_STATE, then the next state will be ACCELEROMETER_PWM_STATE.
 * If the currentState is ACCELEROMETER_PWM_STATE, then the next state will be ACCELEROMETER_NORMAL_STATE.
 * If the currentState is TEMPERATURE_NORMAL_STATE, then the next state will be TEMPERATURE_PWM_STATE.
 * If the currentState is TEMPERATURE_PWM_STATE, then the next state will be TEMPERATURE_NORMAL_STATE.
*/
void switchInternalState() {
	osMutexWait(currentState_mutex, osWaitForever);
	turnOffLED12();
	turnOffLED13();
	turnOffLED14();
	turnOffLED15();
	
	switch (currentState) {
		case ACCELEROMETER_NORMAL_STATE:
			currentState = ACCELEROMETER_PWM_STATE;
		break;
		case ACCELEROMETER_PWM_STATE:
			currentState = ACCELEROMETER_NORMAL_STATE;
		break;
		case TEMPERATURE_NORMAL_STATE:		
			currentState = TEMPERATURE_PWM_STATE;
		break;
		case TEMPERATURE_PWM_STATE:
			currentState = TEMPERATURE_NORMAL_STATE;
		break;
	}
	osMutexRelease(currentState_mutex);
}

/**
 * Process the actions that correspond to the state at which the FSM is currently in.
*/
void processCurrentState() {
	osMutexWait(currentState_mutex, osWaitForever);
	fsmState state = getCurrentState();
	osMutexRelease(currentState_mutex);
	
	// Executes the appropriate state functionality.
	switch (state) {
		case ACCELEROMETER_NORMAL_STATE:
			processAccelerometerNormalState();
		break;
		case ACCELEROMETER_PWM_STATE:
			processAccelerometerPWMState();
		break;
		case TEMPERATURE_NORMAL_STATE:
			processTemperatureNORMALState();
		break;
		case TEMPERATURE_PWM_STATE:
			processTemperaturePWMState();
		break;
	}
}

/**
 * Processes the ACCELEROMETER_NORMAL_STATE state operations.
*/
void processAccelerometerNormalState() {
	int allAxisAngles[2];
	getAngles(allAxisAngles);
	int pitch = allAxisAngles[0];
	int roll = allAxisAngles[1];
	
	//printf("[INFO] NORMAL STATE. Pitch=%d and Roll=%d\n", allAxisAngles[0], allAxisAngles[1]);
	
	// Manages the LED used for the Pitch angle.
	if (pitch >= LOWER_BOUND && pitch < MIDDLE_LOWER_BOUND) {
		turnOffLED15();
		if (pitchCounter < -UPPER_COUNTER_BOUND) {
			invertLED13();
			pitchCounter = 0;
		}
		pitchCounter--;
	} else if (pitch >= MIDDLE_LOWER_BOUND && pitch < LOWER_DEAD) {
		turnOffLED15();
		if (pitchCounter < -MIDDLE_COUNTER_BOUND) {
			invertLED13();
			pitchCounter = 0;
		}
		pitchCounter--;
	} else if (pitch >= LOWER_DEAD && pitch < UPPER_DEAD) {
		// Turns of the LEDs since we are in the "DEAD" zone.
		turnOffLED15();
		turnOffLED13();
	} else if (pitch >= UPPER_DEAD && pitch < MIDDLE_UPPER_BOUND) {
		turnOffLED13();
		if (pitchCounter > MIDDLE_COUNTER_BOUND) {
			invertLED15();
			pitchCounter = 0;
		}
		pitchCounter++;
	} else if (pitch >= MIDDLE_UPPER_BOUND && pitch <= UPPER_BOUND) {
		turnOffLED13();
		if (pitchCounter > UPPER_COUNTER_BOUND) {
			invertLED15();
			pitchCounter = 0;
		}
		pitchCounter++;
	}
	
	// Manages the LED used for the Roll angle.
	if (roll >= LOWER_BOUND && roll < MIDDLE_LOWER_BOUND) {
		turnOffLED14();
		if (rollCounter < -UPPER_COUNTER_BOUND) {
			invertLED12();
			rollCounter = 0;
		}
		rollCounter--;
	} else if (roll >= MIDDLE_LOWER_BOUND && roll < LOWER_DEAD) {
		turnOffLED14();
		if (rollCounter < -MIDDLE_COUNTER_BOUND) {
			invertLED12();
			rollCounter = 0;
		}
		rollCounter--;
	} else if (roll >= LOWER_DEAD && roll < UPPER_DEAD) {
		// Turns of the LEDs since we are in the "DEAD" zone.
		turnOffLED14();
		turnOffLED12();
	} else if (roll >= UPPER_DEAD && roll < MIDDLE_UPPER_BOUND) {
		turnOffLED12();
		if (rollCounter > MIDDLE_COUNTER_BOUND) {
			invertLED14();
			rollCounter = 0;
		}
		rollCounter++;
	} else if (roll >= MIDDLE_UPPER_BOUND && roll <= UPPER_BOUND) {
		turnOffLED12();
		if (rollCounter > UPPER_COUNTER_BOUND) {
			invertLED14();
			rollCounter = 0;
		}
		rollCounter++;
	}
	
	processLEDStates();
}

/**
 * Processes the ACCELEROMETER_PWM_STATE state operations.
*/
void processAccelerometerPWMState() {
	// Start with a full duty cycle (100%) and decrement it until the duty cycle is equal to 0.
	while (ledBrightness > 0) {
		// The LEDs have been configured with TIM4 so this is why we do this.
		// Check for this function to have more info: TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		// CCR1 is the TIM capture/compare register 1, same for CCR2, CCR3 and CCR4.
		TIM4->CCR1 = ledBrightness;
		TIM4->CCR2 = ledBrightness;
		TIM4->CCR3 = ledBrightness;
		TIM4->CCR4 = ledBrightness;
		ledBrightness -= 5;
		
		// Adds some delay so that the humans can see what's going on.
		osDelay(1);
	}
	
	// Start with a duty cycle equal to 0 and increment it until it reaches 100%.
	while (ledBrightness < MAX_PWM_DUTY_CYCLE) {
		// The LEDs have been configured with TIM4 so this is why we do this.
		// Check for this function to have more info: TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		// CCR1 is the TIM capture/compare register 1, same for CCR2, CCR3 and CCR4.
		TIM4->CCR1 = ledBrightness;
		TIM4->CCR2 = ledBrightness;
		TIM4->CCR3 = ledBrightness;
		TIM4->CCR4 = ledBrightness;
		ledBrightness += 5;
		
		// Adds some delay so that the humans can see what's going on.
		osDelay(1);
	}
}

/**
 * Processes the TEMPERATURE_NORMAL_STATE state operations.
*/
void processTemperatureNORMALState() {
	// We compute the filter average once and use the computed value elsewhere in the code.
	int actualFilteredAverage = (int) getTemperatureAverage();
	
	// If we observe that the computed value is different than the previous average, we update the deviation properly.
	if (prevFilterAvg < actualFilteredAverage){
		deviation += actualFilteredAverage - prevFilterAvg;
		prevFilterAvg = actualFilteredAverage;
	} else if (prevFilterAvg > actualFilteredAverage){
		deviation -= prevFilterAvg - actualFilteredAverage;
		prevFilterAvg = actualFilteredAverage;
	}
	
	// Rotates the LED accordingly.
	if (deviation > 1){
		cwLeds();
		deviation = 0;//we have gone +2 deg C, reset deviation
	} else if (deviation < -1){
		ccwLeds();
		deviation = 0;//we have gone -2 deg C, reset deviation
	}
	
	switch (currentLED) {
		case TOP:
			turnOffLED12();
			turnOnLED13();
			turnOffLED14();
			turnOffLED15();
		break;
		case LEFT:
			turnOnLED12();
			turnOffLED13();
			turnOffLED14();
			turnOffLED15();
		break;
		case BOTTOM:
			turnOffLED12();
			turnOffLED13();
			turnOffLED14();
			turnOnLED15();
		break;
		case RIGHT:
			turnOffLED12();
			turnOffLED13();
			turnOnLED14();
			turnOffLED15();
		break;
	}
	processLEDStates();
}

/**
 * rotates the active LED in a clockwise manner (increase in temp)
*/
void cwLeds(){
	if (currentLED == TOP){
		currentLED = RIGHT;
	} else if (currentLED == RIGHT){
		currentLED = BOTTOM;
	} else if (currentLED == BOTTOM){
		currentLED = LEFT;
	} else{
		currentLED = TOP;
	}
}
/**
 * rotates the active LED in a counter-clockwise manner (decrease in temp)
*/
void ccwLeds(){
	if (currentLED == TOP){
		currentLED = LEFT;
	} else if (currentLED == LEFT){
		currentLED = BOTTOM;
	} else if (currentLED == BOTTOM){
		currentLED = RIGHT;
	} else{
		currentLED = TOP;
	}
}

/**
 * Processes the TEMPERATURE_PWM_STATE state operations.
*/
void processTemperaturePWMState() {
	invertLED12();
	invertLED13();
	invertLED14();
	invertLED15();
	processLEDStates();
	osDelay(100);
}