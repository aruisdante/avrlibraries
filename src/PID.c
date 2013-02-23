/**
 * @file PID.c
 * @brief Library of functions for PID control
 *
 * @date Feb 2, 2010
 * @author Adam Panzica
 * @version 1.0 Initial version with functions: PID_Initialize(), *PID_Add_Channel(), PID_Proportional(), PID_Integral(), PID_Derivative(), PID(), PID_CorrectionVal_to_Volts(), and ISR(TIMER0_COMPA_vect)
 * @version 2.0 Added pot functions to allow for dynamic configuration of pot scaling factors.
 */

#include "PID.h"
#include "Simple_Serial.h"
#include "Timer8_Functions.h"
#include "DAC.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/****************** SYMBOLIC CONSTANTS ***************/
/**Scale factor for Degree to Pot conversion*/
#define SCALE1 1000
/**Scale factor for Pot to Degree conversion*/
#define SCALE2 10000

/********************* STATIC VARS ********************/
/** Counter to keep track of the channel number for new PID channels */
static char nextchan = 0;
/** Memory alocation for up to 8 PID channels */
static PID_CHANNEL buffer[8];
/** Memory alocation for up to 8 POT_ANGLES*/
static POT_ANGLE joint[8];

/********************* PUBLIC FUNCTIONS **********************/

/**
 * @fn void PID_Initialize()
 * @brief Initializes PID control
 *
 * This function initializes PID by setting up the 8bit timer to refresh at a set frequency. The actual PID calculations are called from the ISR for this timer
 */
void PID_Initialize()
{
	Timer8_Initialize_CTC(UPDATEFREQ);
}


/**
 * @fn PID_CHANNEL *PID_Add_Channel(int *target, int *current, int *correctionValue, int *LMDVal, int Kp, int Ki, int Kd, char spool)
 * @brief Adds a PID channel to the PID controller
 * @param [in] *target pointer to the target value for this PID channel
 * @param [in] *current pointer to the current value for this PID channel
 * @param [in] *correctionValue pointer to a location to write the correction value from this PID channel
 * @param [in] *LMDVal pointer to a location to write the requested LMD output voltage to.
 * @param [in] Kp Proportional gain constant
 * @param [in] Ki Integral gain constant
 * @param [in] Kd Derivative gain constant
 * @param [in] spool Cap value for the Integral element of the PID loop to prevent it from dominating the system
 * @return A pointer to the added PID channel
 *
 * This function adds a PID channel to the PID controller. It is automatically handled from the moment it is added. Communication with the controller is handled
 * by updating the input communication vars, *target and *current. Write to target to change the value that the PID loop seeks, and update current as often as
 * possible to ensure that the PID loop has accurate current state information. The output states of the PID loop can be checked by reading the contents of *correctionVal
 * and *LMDVal
 */
PID_CHANNEL *PID_Add_Channel(int *target, int *current, int *correctionValue, int *LMDVal, int Kp, int Ki, int Kd, char spool)
{
	//check to make sure the maximum number of channels has not been reached. If it hasn't, store values into the next available channel
	if (nextchan < 8)
	{
		buffer[(int)nextchan].target = target;
		buffer[(int)nextchan].current= current;
		buffer[(int)nextchan].correctionValue = correctionValue;
		buffer[(int)nextchan].LMDVal = LMDVal;
		buffer[(int)nextchan].Kp = Kp;
		buffer[(int)nextchan].Ki = Ki;
		buffer[(int)nextchan].Kd = Kd;
		buffer[(int)nextchan].spool = spool;
		buffer[(int)nextchan]._channel = nextchan;
		nextchan++;
		return &buffer[(int)nextchan];
	}
	//return a special fail code, as the return is expected to be a pointer to a memory location
	else return '\0';
}


/************************ PRIVATE FUNCTIONS ***********************/

/**
 * @fn int PID_Proportional(int *error, int *Kp)
 * @brief calculates the proportional element of PID
 * @param [in] *error pointer to an error value
 * @param [in] *Kp pointer to the proportional gain
 * @return the proportional element of PID
 *
 * This function calculates the proportional element of a PID control loop based off a passed error value
 * and a gain constant, following the function P=Kp*E
 */
int PID_Proportional(int *error, int *Kp)
{
	long temp;
	temp = (long)(*error)*(long)(*Kp)/100;

	return (int)temp;
}

/**
 * @fn int PID_Integral(int *errorSum, int *Ki)
 * @brief calculates the integral element of PID
 * @param [in] *errorSum pointer to a value corresponding to the sum of errors
 * @param [in] *Ki pointer to the integral gain
 * @return the integral element of PID
 *
 * This function calculates the integral element of a PID control loop based off a passed sum of errors value
 * and a gain constant, following the function I=sum(E)/time*Ki
 */
int PID_Integral(int *errorSum, int *Ki)
{
	long temp;
	temp = (long)(*errorSum)*(long)(*Ki)/100;

	return (int)temp;
}

/**
 * @fn int PID_Derivative(int *error, int *lastError, int *Kd)
 * @brief calculates the derivative element of PID
 * @param [in] *error pointer to an error value
 * @param [in] *lastError point to the previous error value
 * @param [in] *Kd pointer to the derivative gain
 * @return the derivative element of PID
 *
 * This function calculates the derivative element of a PID control loop based off a passed error and last error value
 * and a gain constant, following the function D=(E2-E1)/time*Kd
 */
int PID_Derivative(int *error, int *lastError, int *Kd)
{
	long temp;

	temp = (long)(*error - *lastError)*(long)(*Kd)/100;

	return (int)temp;
}

/**
 * @fn int PID(PID_CHANNEL *PIDchan)
 * @brief calculates PID
 * @param [in] *PIDchan pointer to a PID_CHANNEL struct
 * @return a correction value using PID
 *
 * This function calculates a PID correction factor following the function PID= P+I+D
 */
int PID(PID_CHANNEL *PIDchan)
{
	int pTerm;
	int iTerm;
	int dTerm;

	//calculate P, I and D components
	pTerm = PID_Proportional(&(PIDchan->_error), &(PIDchan->Kp));
	dTerm = PID_Derivative(&(PIDchan->_error), &(PIDchan->_lastError), &(PIDchan->Kd));
	iTerm = PID_Integral(&(PIDchan->_errorSum), &(PIDchan->Ki));

	//cap iTerm if spool protection was requested
	if((PIDchan->spool != 0))
	{
		if(iTerm > (int)PIDchan->spool) iTerm = (int)PIDchan->spool;
		else if (iTerm < -1*(int)PIDchan->spool) iTerm = -1*(int)PIDchan->spool;
	}

	return pTerm+iTerm+dTerm;
}

/**
 * @fn int PID_CorrectionVal_to_Volts(int correctionVal)
 * @brief Turns the unit-less number returned by the PID loop into a voltage value for the LMD
 * @param [in] correctionVal PID value to be converted into a voltage
 * @return A voltage value between -12000 and 12000 mV
 * This function converts the arbitrary, unit-less number returned by the PID loop into a voltage value between -12000 and 12000 mV that can be
 * used to set the LMD to control motors
 */
int PID_CorrectionVal_to_Volts(int correctionVal)
{
	long temp;

	temp = (long)correctionVal*-12000/100;
	if(temp>12000) temp = 12000;
	else if (temp<-12000) temp = -12000;

	return (int)temp;
}

/**
 * @fn ISR(TIMER0_COMPA_vect)
 * @brief ISR handler that uses timer 0 to update the PID controller
 *
 * This function does all of the calculations to update the state of the PID controller. It is called every time that the counter in Timer 0 overflows. This allows the
 * user to set up a set refresh rate for the PID controller independent of external factors.
 */
ISR(TIMER0_COMPA_vect)
{
	int i=0;
	int temp;

	//update the internal variables in each PID_CHANNEL that has been initialized
	for(i=0; i<nextchan; i++)
	{
		buffer[i]._errorSum = buffer[i]._error+buffer[i]._lastError+buffer[i]._lastLastE;
		buffer[i]._lastLastE = buffer[i]._lastError;
		buffer[i]._lastError = buffer[i]._error;
		buffer[i]._error = *buffer[i].target - *buffer[i].current;
		*buffer[i].correctionValue = PID(&buffer[i]);
	}

	//insert code to do something with the correction value here
	for(i=0; i<nextchan; i++)
	{
		//Since in this instance PID is being used to control motors through the LMD output, turn the correction value into a voltage and then set the LMD output
		temp = PID_CorrectionVal_to_Volts(*(buffer[i].correctionValue));
		*buffer[i].LMDVal = temp;
		DAC_Set_LMD_Output(temp, (char)i);
	}
}

/**@fn 		int Pot_To_Degrees(int potVal, int channel)
 *@brief	Converts a 10bit pot value to an angle in degrees
 *@param	[in] potVal 10bit pot value
 *@param	[in] channel channel to use for the conversion
 *@returns	angle in degrees
 */
int Pot_To_Degrees(int potVal, int channel)
{
	long temp = 0;


	temp = (long)potVal*(long)joint[channel]._slopePTD/(long)SCALE2+(long)joint[channel]._offsetPTD;

	return (int)temp;
}

/**@fn 		int Degrees_To_Pot(int degrees, int channel)
 *@brief	Converts an angle in degrees to a 10bit pot value
 *@param	[in] degrees angle in degrees
 *@param	[in] channel channel to use for the conversion
 *@returns	10bit pot value
 */
int Degrees_To_Pot(int degrees, int channel)
{
	long temp = 0;

	temp = (long)degrees*(long)joint[channel]._slopeDTP/(long)SCALE1+(long)joint[channel]._offsetDTP;

	return (int)temp;
}


/**@fn void Calc_Transfer_Function(int *X1, int *Y1, int *X2, int *Y2, int *slope, int *intercept, int scale)
 * @brief calculates a linear transfer function's slope and offset, with the slope multiplied by scale to allow for inter only math
 * @param [in] *X1 pointer to the lower X bound of the transfer function
 * @param [in] *Y1 pointer to the lower Y bound of the transfer function
 * @param [in] *X2 pointer to the upper X bound of the transfer function
 * @param [in] *Y2 pointer to the upper Y bound of the transfer function
 * @param [in] *slope pointer to a variable to store the slope to
 * @param [in] *intercept point to a variable to store the intercept to
 * @param [in] scale scale factor to multiply the slope by
 */
void Calc_Transfer_Function(int *X1, int *Y1, int *X2, int *Y2, int *slope, int *intercept, int scale)
{
	long tempSlope;
	long tempInt;

	tempSlope = ((long)(*Y2)-(long)(*Y1))*(long)scale/((long)(*X2) - (long)(*X1));
	*slope = (int)tempSlope;
	tempInt = (long)(*Y1)-tempSlope*((long)(*X1))/(long)scale;
	*intercept = (int)tempInt;
}

/**@fn	POT_ANGLE *Add_Pot_Angle(int valMax, int angMax, int valMin, int angMin, char *name, char channel);
 * @brief This function creates a POT_ANGLE struct
 * @param [in] valMax ADC value at maximum angle
 * @param [in] angMax Angle at maximum ADC value
 * @param [in] valMin ADC value at minimum angle
 * @param [in] angMin Angle at minimum ADC value
 * @param [in] *name pointer to an array of chars containing the name of the angle (max 15 chars)
 * @param [in] channel ADC channel that the pot is on
 */
POT_ANGLE *Add_Pot_Angle(int valMax, int angMax, int valMin, int angMin, char *name, char channel)
{
	int i = 0;
	if((channel >= 0)&&(channel < 8))
	{
		joint[(int)channel].channel = channel;
		joint[(int)channel].angMax = angMax;
		joint[(int)channel].angMin = angMin;
		joint[(int)channel].valMax = valMax;
		joint[(int)channel].valMin = valMin;
		for(i=0; i<15; i++)
		{
			joint[(int)channel].name[i]=name[i];
		}

		Calc_Transfer_Function(&joint[(int)channel].angMax, &joint[(int)channel].valMax,
							   &joint[(int)channel].angMin, &joint[(int)channel].valMin,
							   &joint[(int)channel]._slopeDTP, &joint[(int)channel]._offsetDTP, SCALE1);
		Calc_Transfer_Function(&joint[(int)channel].valMax, &joint[(int)channel].angMax,
							   &joint[(int)channel].valMin, &joint[(int)channel].angMin,
							   &joint[(int)channel]._slopePTD, &joint[(int)channel]._offsetPTD, SCALE2);

		return &joint[(int)channel];
	}
	else return '\0';
}
