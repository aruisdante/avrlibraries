/** @brief Provides functions for utilizing the ADC converter
*
* @file ADC.c
*
* This source file contains a library of functions to utilize the ADC in both polling
* and interrupt mode
*
* @author Adam Panzica
* @author Joel Sotherland
* @date 22-Jan-2009
*@version 1.0 Initial version with ADC_Init_P() and ADC_Get_Value()
*@version 1.1 Updated ADC code, fixed prescaler issue, added ADC_Sensor struct and its constructor ADC_Sensor_Construct() for conversions.
*@version 1.2 Added ADC_Sensor_Calc_Value() and ADC_Calc_Volts()
*
*/

#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"
#include "portIO.h"

/***************** SYMBOLIC CONSTANTS ***************************/


/**@def BBCLK
 * Pin number for Bit Banged SPI clock
 */
#define BBCLK	PD7

/**@def BBD
 * Pin number for Bit Banged SPI data I/O line
 */
#define BBD	PD6

/**@def BBSS
 * Pin number for Bit Banged SPI Slave Select
 */
#define BBSS	PD5

/**@def BBPERIOD
 * Clock period for Bit Banged SPI
 */
#define BBPERIOD 98


/********************** MCP3024 COMMAND UNION *********************/
/**@union MCP3024_COMMAND
 * @brief union for setting bits in a command to the MCP3024 ADC on the accelerometer board
 */
typedef union MCP3024_COMMAND{
	struct {
		unsigned 		:3;
		unsigned _D0	:1;
		unsigned _D1	:1;
		unsigned _D2	:1;
		unsigned _SGDF	:1;
		unsigned _STRB	:1;
	};
	struct {
		unsigned _w		:8;
	};
} MCP3024_COMMAND;

/********************** PUBLIC FUNCTIONS **************************/

/**@fn 		void ADC_Init_P()
 *@brief	Initialize ADC for polling use
 *@return 	none
 *
 * This function initializes the ADC to be read using a polling system
 */
void ADC_Init_P()
{
	ADCSRA = 1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0;
}

/**@fn 		int ADC_Get_Value(unsigned char channel)
 *@brief	Take a single sample from the ADC
 *@param	[in] channel channel number (0-7) to read the ADC value from. Defaults to 0 if out of bounds
 *@return 	Value on the specified ADC channel
 *
 * This function takes a single sample from a specified ADC channel using a polling mechanism to determine when the sample is ready
 */
int ADC_Get_Value(unsigned char channel)
{
	int temp;
	char low;

	//check channel bounds
	if((channel <= 7) && (channel >= 0)) ADMUX = channel;
	else ADMUX = 0;

	//start a sample
	ADCSRA |= 1<<ADSC;

	//wait till sample is complete
	while(ADCSRA&(1<<ADSC));

	//read sample value
	low = ADCL;
	temp = ADCH;
	temp = temp<<8;
	temp |= low;

	return temp;
}

/**@fn struct ADC_Sensor ADC_Sensor_Construct ( char *name, unsigned char channel, int valueAtMax, int valueAtMin)
 * @brief this function constructs an ADC_Sensor struct
 * @param [in] *name pointer to an array of chars containing the name of the sensor. Max length is 15 including null terminator
 * @param [in] channel channel number that the sensor is on (0-7)
 * @param [in] valueAtMax the value (in engineering units, E.G. PSI) of the sensor at Vin=Vref
 * @param [in] valueAtMin the value (in engineering units, E.G. PSI) of the sensor at Vin=Ground
 *
 * This function constructs an ADC_Sensor using the passed values. It returns an ADC_Sensor struct which is used in other ADC
 * functions to calculate engineering values from raw ADC units
 */

struct ADC_Sensor ADC_Sensor_Construct ( char *name, unsigned char channel, int valueAtMax, int valueAtMin)
{
	int i;
	struct ADC_Sensor temp;

	temp.scaleFactor = ((valueAtMax-valueAtMin)*100)/1024;
	temp.channel = channel;
	temp.valueAtMax = valueAtMax;
	temp.valueAtMin = valueAtMin;

	for(i=0; i<15; i++)
	{
		temp.name[i]=name[i];
	}

	return temp;
}

/**@fn int ADC_Sensor_Calc_Value (struct ADC_Sensor *sensor, int ADCValue)
 * @brief this function calculates the engineering unit value of a sensor reading in ADC values
 * @param [in] *sensor pointer to an ADC_Sensor
 * @param [in] ADCValue returned by the ADC (0-1024)
 * @returns the value of the sensor in engineering units (multiplied by a scale factor of 100 to allow for decimal values)
 *
 * This function constructs an ADC_Sensor using the passed values. It returns an ADC_Sensor struct which is used in other ADC
 * functions to calculate engineering values from raw ADC units
 */
int ADC_Sensor_Calc_Value (struct ADC_Sensor *sensor, int ADCValue)
{
	return (sensor->scaleFactor)*ADCValue+(sensor->valueAtMin)*100;
}

/**@fn int ADC_Calc_Volts(int ADCValue, int VRef)
 * @brief this function converts raw ADC values into millivolts for a given Vref.
 * @param [in] ADCValue the 10-bit value returned by the ADC
 * @param [in] VRef the reference voltage (in mV) used by the ADC
 * @returns the input voltage in millivolts
 *
 * This function calculates the input voltage based on ADC value and Vref.
 */
int ADC_Calc_Volts(int ADCValue, int VRef)
{
	long temp1;
	long temp2;
	long temp3;
	temp1 = (long)ADCValue;
	temp2 = (long)VRef;
	temp3 = temp1*temp2/1024;
	return (int)temp3;
}

/**@fn char ADC_MCP3204_Send_Command(MCP3024_COMMAND *command)
 * @brief sends a command word to the MCP3024
 * @param [in] *command Pointer to an MCP3024 command struct
 * @returns 1 if successful
 *
 * This function sends a command word to the MCP3024 ADC via bit banged SPI
 */
char ADC_MCP3204_Send_Command(MCP3024_COMMAND *command)
{
	//set up direction registers
	DDRD |= (1<<BBCLK)|(1<<BBD);
	PORTD &= (~((1<<BBCLK)|(1<<BBD)));

	//send start bit of the command
	Pin_Set('D', BBD, command->_STRB);
	_delay_us(1);
	Pin_Set('D', BBCLK, 1);
	_delay_us(BBPERIOD/2);
	Pin_Set('D', BBCLK, 0);
	_delay_us(BBPERIOD/2);

	//send Single/Diff bit of the command
	Pin_Set('D', BBD, command->_SGDF);
	_delay_us(1);
	Pin_Set('D', BBCLK, 1);
	_delay_us(BBPERIOD/2);
	Pin_Set('D', BBCLK, 0);
	_delay_us(BBPERIOD/2);

	//send D2 bit of the command
	Pin_Set('D', BBD, command->_D2);
	_delay_us(1);
	Pin_Set('D', BBCLK, 1);
	_delay_us(BBPERIOD/2);
	Pin_Set('D', BBCLK, 0);
	_delay_us(BBPERIOD/2);

	//send D1 bit of the command
	Pin_Set('D', BBD, command->_D1);
	_delay_us(1);
	Pin_Set('D', BBCLK, 1);
	_delay_us(BBPERIOD/2);
	Pin_Set('D', BBCLK, 0);
	_delay_us(BBPERIOD/2);

	//send D0 bit of the command
	Pin_Set('D', BBD, command->_D0);
	_delay_us(1);
	Pin_Set('D', BBCLK, 1);
	_delay_us(BBPERIOD/2);
	Pin_Set('D', BBCLK, 0);
	_delay_us(BBPERIOD/2);

	return 1;
}

/**@fn char ADC_MCP3204_Recive_Data(int *receive)
 * @brief Reads a value from the MCP3024
 * @param [in] *receive Pointer to an integer to write the value to
 * @returns 1 if successful
 *
 * This function reads a data segment from the MCP3024 ADC via bit banged SPI
 */
char ADC_MCP3204_Recive_Data(int *receive)
{
	char i;
	char temp;
	//set up direction registers to receive data
	DDRD &= ~(1<<BBD);

	//clock in the null bit
	Pin_Set('D', BBCLK, 1);
	_delay_us(BBPERIOD/2);
	Pin_Set('D', BBCLK, 0);
	_delay_us(BBPERIOD/2);

	//clock in data
	for (i=0; i<=12; i++)
	{
		temp = Pin_Read('D', BBD);
		*receive |= temp;
		*receive = *receive << 1;
		_delay_us(BBPERIOD/2);
		Pin_Set('D', BBCLK, 0);
		_delay_us(BBPERIOD/2);
		if(i<12) Pin_Set('D', BBCLK, 1);
	}
	//Pin_Set('D', BBCLK, 0);
	return 1;
}

/**@fn int ADC_MCP3204_Get_Value(unsigned char channel, unsigned char justify)
 * @brief Reads a value from the MCP3024 from the specified channel
 * @param [in] channel ADC channel to read from
 * @param [in] justify 0 if the returned 12bit value should be right justified, 1 if it should be left justified
 * @returns A 12bit value from the ADC
 *
 * This function uses bit banged SPI to communicate with the MCP3024 ADC. It gets a measurement from the requested channel and, if desired, left justifies it.
 */
int ADC_MCP3204_Get_Value(unsigned char channel, unsigned char justify)
{
	MCP3024_COMMAND commandByte;
	int dataPacket=0;

	//set up SS line direction register
	DDRD |= (1<<BBSS);

	commandByte._STRB = 1;
	commandByte._SGDF = 1;
	switch (channel)
	{
	case 'a':
	case 'A':
	case 0:
		commandByte._D2 = 0;
		commandByte._D1 = 0;
		commandByte._D0 = 0;
		break;
	case 'b':
	case 'B':
	case 1:
		commandByte._D2 = 0;
		commandByte._D1 = 0;
		commandByte._D0 = 1;
		break;
	case 'c':
	case 'C':
	case 2:
		commandByte._D2 = 0;
		commandByte._D1 = 1;
		commandByte._D0 = 0;
		break;
	case 'd':
	case 'D':
	case 3:
		commandByte._D2 = 0;
		commandByte._D1 = 1;
		commandByte._D0 = 1;
		break;
	default:
		commandByte._D2 = 0;
		commandByte._D1 = 0;
		commandByte._D0 = 0;
		break;
	}

	Pin_Set('D', BBSS, 0);
	ADC_MCP3204_Send_Command(&commandByte);
	ADC_MCP3204_Recive_Data(&dataPacket);
	Pin_Set('D', BBSS, 1);

	//if justify was set, left justify the data
	if(justify == 1)
	{
		dataPacket = dataPacket << 4;
	}

	return dataPacket;
}

/**@fn int ADC_MCP3204_Count_to_Gs(int ADCVal, int VRef)
 * @brief Converts a 12bit ADC value to 100ths of a G
 * @param [in] ADCVal 12bit ADC value to convert
 * @param [in] VRef The VRef value from the ADC
 * @returns a value in 100ths of a G
 *
 * This function converts the 12bit ADC value from the MCP3024 on the Parallax accelerometer board and converts it into 100ths of a G.
 */
int ADC_MCP3204_Count_to_Gs(int ADCVal, int VRef)
{
	long temp;

	temp = ((long)ADCVal - (long)VRef)*(long)22/(long)100;

	return (int)temp;
}
