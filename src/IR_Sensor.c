/**
 * @file IR_Sensor.c
 * @brief Functions for utilizing the Sharp IR range finding sensor
 * @date Feb 16, 2010
 * @author Adam Panzica
 * @version 1.0 Initial version with ReadIR() and interpolate()
 */

#include "IR_Sensor.h"
#include "Simple_Serial.h"
#include "ADC.h"
#include <avr/io.h>

/**@fn int IR_interpolate(int xA, int yA, int xB, int yB, int value)
 * @brief Performs a linear interpolation
 * @param [in] xA the lower bound along the input axis
 * @param [in] yA the lower bound along the output axis
 * @param [in] xB the upper bound along the input axis
 * @param [in] yB the upper bound along the output axis
 * @param [in] value the input value to be interpolated
 * @returns the output value of the interpolated input
 *
 * This function performs a linear interpolation between two points and a given input
 */
int IR_interpolate(int xA, int yA, int xB, int yB, int value)
{
	long temp;

	temp = (long)yA+(((long)value-(long)xA)*((long)yB - (long)yA))/((long)xB-(long)xA);
	return (int)temp;
}

/**@fn int ReadIR( int channel )
 * @brief Gets a range formt the IR sensor
 * @param [in] channel the desired channel to read from
 * @returns the range to target, in mm
 *
 * This function reads a value from the sharp IR sensor on the specified channel and returns a range in millimeters
 */
int ReadIR( int channel )
{
	int rawVol;
	int interRangeHigh;
	int interVolHigh;
	int interRangeLow;
	int interVolLow;
	int reduced;
	long temp;

	int lookup [20] = {622, 521, 451, 387, 333 ,298, 270, 241, 219, 203, 184, 168, 156, 143, 130, 120, 114, 110, 105, 98};

	rawVol = ADC_Get_Value((char)channel);

	temp = ((((long)475 * (long)rawVol)/100) - 475) / 100;
	reduced = (int)temp;

	//check to make sure a valid reduced value has been formed
	if(reduced < 0) reduced = 0;
	interRangeLow = lookup[reduced];

	//check to make sure a valid reduced+1 is possible
	if(reduced < 19)interRangeHigh = lookup[reduced+1];
	else interRangeHigh = lookup[reduced];


	temp = ((((long)reduced*(long)100)+(long)475) * (long)100)/(long)475;
	interVolLow = (int)temp;
	temp = (((((long)(reduced+1))*(long)100)+(long)475) * (long)100)/(long)475;
	interVolHigh = (int)temp;


	return IR_interpolate(interVolLow, interRangeLow, interVolHigh, interRangeHigh, rawVol);

}

