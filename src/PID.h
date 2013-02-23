/**
 * @file PID.h
 * @brief Header for library of functions for PID control
 * @date Feb 2, 2010
 * @author Adam Panzica
 * @version 1.0 First implementation of the PID_CHANNEL struct
 */

#ifndef PID_H_
#define PID_H_

/**
 *@struct PID_CHANNEL PID.h "PID.h"
 *@brief this strcut defines a PID_CHANNEL object
 *
 * The PID_CHANNEL serves as an interface to the PID controller. All interactions with the PID controller happen through a PID_CHANNEL struct.
 * Specifically, the Kp, Ki, Kd and spool constants are set once, and then the contents of the variables pointed to by the 'Communication Variables',
 * *target and *current, can be used to update the PID controller, while *correctionValue and *LMDValue can be used to monitor status.
 * The 'Internal Variables' are utilized by 'private' functions in the PID controller, and thus should not be accessed by the user directly.
 */
typedef struct PID_CHANNEL
{
	/**Proportional Tuning Constant*/
	int Kp;
	/**Integral Tuning Constant*/
	int Ki;
	/**Derivative Tuning Constant*/
	int Kd;
	/**Integral Spool Protection constant*/
	char spool;

	//Internal Variables
	/**Current error to target*/
	int _error;
	/**Last error to target*/
	int _lastError;
	/**Error previous to last error, used by errorSum*/
	int _lastLastE;
	/**Sum of errors to target*/
	int _errorSum;
	/**Channel number*/
	char _channel;

	//Communication Variables
	/**Pointer to the current target of the PID channel, to be red from*/
	int *target;
	/**Pointer to the current input to the PID channel, to be red from*/
	int *current;
	/**Pointer to a variable that contains the PID correction value, to be written to*/
	int *correctionValue;
	/**Voltage being sent to the LMD*/
	int *LMDVal;
} PID_CHANNEL;

/**
 * @struct POT_ANGLE PID.h "PID.h"
 * This struct creates a pot-angle object which eases the conversion from a pot value representing
 * a joint angle to a value in degrees and vice versa.
 */
typedef struct POT_ANGLE
{
	//Set variables
	/**ADC value at maximum angle*/
	int valMax;
	/**Angle at maximum ADC value*/
	int angMax;
	/**ADC value at minimum angle*/
	int valMin;
	/**Angle at minimum ADC value*/
	int angMin;
	/**Name of the angle*/
	char name[15];
	/**ADC channel that the sensor is on*/
	char channel;

	//Internal variables
	/**Slope converting from pot value to degrees*/
	int _slopePTD;
	/**Offset converting from pot value to degrees*/
	int _offsetPTD;
	/**Slope converting from degrees to pot value*/
	int _slopeDTP;
	/**Offset converting from degrees to pot value*/
	int _offsetDTP;
}POT_ANGLE;

/**@def UPDATEFREQ
 * update frequency for the PID controller
 */
#define UPDATEFREQ 100


void PID_Initialize();
PID_CHANNEL *PID_Add_Channel(int *target, int *current, int *correctionValue, int *LMDVal, int Kp, int Ki, int Kd, char spool);
POT_ANGLE *Add_Pot_Angle(int valMax, int angMax, int valMin, int angMin, char *name, char channel);
int Pot_To_Degrees(int potVal, int channel);
int Degrees_To_Pot(int degrees, int channel);

#endif /* PID_H_ */
