/**
 * @file accelerometerManager.c
 * @author Jean-Sebastien Dery (260430688) and Matthew Johnston (260349319)
 * @version 1.0
 * @date October 25th 2013
 * @brief Manages all the interactions with the on-board accelerometer (initialization, read, etc.).
*/

#include <stdio.h>
#include "accelerometerManager.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stm32f4xx_exti.h"
#include "math.h"
#include "movingAverageFilter.h"
#include "stm32f4xx.h"
#include "misc.h"

const double PI = 3.1415926535;

// Those values were determined with the Least Square Method
double AxOffset = +20.5; //-21.4
double AyOffset = -81.9; //+77.3
double AzOffset = -1.9; //+28

// Those functions are private to this C file.
void activateEXTI1_On_NVIC();
void readAcceleration(int32_t* allAxisAcceleration);
int radianToDegree(double radian);

/**
 * Initializes the on-board accelerometer.
 *
 * Sets all the parameters defined, and needed, for the experiment on the LIS302DL accelerometer.
 * Initializes it and setup the SPI connection with the chip.
*/
void initializeAccelerometer() {
	// Sets up all the parameters in order to initialize the accelerometer.
	LIS302DL_InitTypeDef accInitParameters;
	// Activates the accelerometer.
	accInitParameters.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
	// Sets the output data rate, it needs to be more than 25 Hz since we will be collecting the data at 25 Hz.
	// There are only two choices available anyway (100 Hz or 400 Hz).
	// The rate was chosen since it meets the requirement and it has a lower power consumption than running it at 400 Hz.
	accInitParameters.Output_DataRate = LIS302DL_DATARATE_100;
	// Sets the axis to be activated. All the axis are activated (X, Y and Z) since we want to be able to have the Roll and Pitch of the board.
	accInitParameters.Axes_Enable = LIS302DL_Z_ENABLE;
	// The Full Scale range was set to 2_3 so we can achieve a better precision on the readings that we are doing.
	// The scale is going to be +/-2g at this mode (we don't need to have a larger scale for this lab experiment).
	// Go to the Accelerometer Application Notes at page 18 for more details about that.
	accInitParameters.Full_Scale = LIS302DL_FULLSCALE_2_3;
	// We set the SelfTest to NORMAL, because we don't want it to test itself.
	accInitParameters.Self_Test = LIS302DL_SELFTEST_NORMAL;

	// Initializes the LIS302DL accelerometer with the specified parameters in accInitParameters.
	LIS302DL_Init(&accInitParameters);
}

/**
 * Activates the interrupt mechanism used to detect the Single Tap.
*/
void activateSingleTapDetectionInterrupt() {
	// Configures the accelerometer to generate interrupts.
	LIS302DL_InterruptConfigTypeDef interruptStructure;
	// We latch the interrupt so that the processor has a chance to get it. If it was not latched, it would mean that right after
	// the interrupt has been generated, the bit goes down. And with this kind of setup, there would be chances that the processor
	// misses the generated interrupt.
	interruptStructure.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
	// Activates the Single Tap interrupt for all axis on the accelerometer.
	interruptStructure.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
	// Does not activate the Double Tap interrupt since we don't need to implement this, as per lab requirement.
	interruptStructure.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_XYZ_DISABLE;
	// Sets LIS302DL Interrupt configuration with the interruptStructure.
	LIS302DL_InterruptConfig(&interruptStructure);
	
	// All the settings under are based on the Tapping Application Note available on MyCourses.
	
	// Sets the Threshold value on all axis.
	// The value varies from (0001)BIN (0.5g)DEC to (1111)BIN (0.75g)DEC, with increments of 0.5g
	// This value was determined after a lot of time and pain.
	uint8_t thresholdValue = 0xAE;
	LIS302DL_Write(&thresholdValue, LIS302DL_CLICK_THSZ_REG_ADDR, 1);
	
	// Sets the Time Limit value for the Tap detection. Remember that the number in decimal is multiplied
	// by 5 msec to get the total Time Limit (in miliseconds).
	// The current value is 255DEC which corresponds to the maximum Time Limit possible.
	// This value was determined after a lot of time and pain.
	uint8_t timeLimitValue = 0x0F;
	LIS302DL_Write(&timeLimitValue, LIS302DL_CLICK_THSZ_REG_ADDR, 1);
	
	// Configures the latency.
  uint8_t latency = 0x7F;
  LIS302DL_Write(&latency, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);
	
	// Go in Tapping Application Note at page 9.
	uint8_t controlRegValue = 0x38;
  LIS302DL_Write(&controlRegValue, LIS302DL_CTRL_REG3_ADDR, 1);
	uint8_t clickConfigRegister = 0x10;
  LIS302DL_Write(&clickConfigRegister, LIS302DL_CLICK_CFG_REG_ADDR, 1);
	
	// The Window does not need to be configured since we are only considering Single Tap detection, and
	// it is only used in a Double Tap detection scenario.
	
	activateEXTI1_On_NVIC();
}

/**
 * Activates the Single Tap interrupt on the Nested Vectored Interrupt Controller (NVIC).
*/
void activateEXTI1_On_NVIC() {
	// A bit of resources on that: https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fcortex_mx_stm32%2fSTM32F207%20EXTI%20interrupts&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=2213
	// http://www.keil.com/support/man/docs/gsac/gsac_nvic.htm
	
	GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

	// Enables the clock for GPIOE since this GPIO is connected to the EXTI, and we need to use that since the 
	// the accelerometer is an external peripheral.
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
	// GPIO pin is set to in so that it can get the interrupt.
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// We are using Pin 1 to get the interrupt (which is GPIOE).
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Selects the GPIO pin used as EXTI Line.
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);

	// The External Interrup is set to wait for a signal that comes on GPIOE Pin 0.
	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 

	// The IRQ channel is going to be EXTI1 since it is on this channel that the Single Tap IRQ will be sent.
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	// The NVIC provides a Group Priority and a Sub Priority within this Group.
	// Link to this information: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/BABHGEAJ.html
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * Activates the button interrupt on EXTI0.
*/
void activateButtonInterrupt() {
	// Example on the internet.
	
	GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	EXTI_InitTypeDef   EXTI_InitStructure;
  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 * Gives the 3-axis tilt angles in an array. The first position is Pitch, then Roll and finaly Yaw.
 *
 * @param The array that will contain the 3-axis tilt angles in degree.
*/
void getAngles(int* allAxisAngles) {	
	double Ax = getXValue();
	double Ay = getYValue();
	double Az = getZValue();
	
	// Compite the 3-axis tilt angles. Those formulas can be found in the AN3182 Application note at page 14.
	// Those values will are in radians.
	double pitchInRad = atan(Ax / sqrt(Ay*Ay + Az*Az));
	double rollInRad = atan(Ay / sqrt(Ax*Ax + Az*Az));
	
	// Converts the readings from radian to degree.
	int pitch = radianToDegree(pitchInRad);
	int roll = radianToDegree(rollInRad);
	
	// Stores the computed tilt angles (in degree) in the array passed in the signature.
	allAxisAngles[0] = pitch;
	allAxisAngles[1] = roll;
}

/**
 * Fetches the accelerations from the 3 axis and store the retrieved values in the filter's buffer.
*/
void fetchAllAxisAccelerations() {
	// Creates the array that will contain the acceleration readings for all axis.
	int32_t allAxisAcceleration[3];
	// Reads the acceleration from the accelerometer on all axis.
	readAcceleration(allAxisAcceleration);
	
	// Stores the accelerations in different variables.
	double Ax = (double)allAxisAcceleration[0] - AxOffset;
	double Ay = (double)allAxisAcceleration[1] - AyOffset;
	double Az = (double)allAxisAcceleration[2] - AzOffset;
	
	// Add the acceleration readings to the moving average filter.
	addToXBuffer(Ax);
	addToYBuffer(Ay);
	addToZBuffer(Az);
}

/**
 * Converts the angles from radians to degrees.
 * 
 * @param Converted value in degree.
*/
int radianToDegree(double radian) {
	return (radian*360)/(2*PI);
}

/**
 * Reads the acceleration from the 3-axis onboard accelerometer.
 *
 * @param The array that will contain the acceleration for each axis.
*/
void readAcceleration(int32_t* allAxisAcceleration) {
	LIS302DL_ReadACC(allAxisAcceleration);
}