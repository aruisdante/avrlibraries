/**
 * @file DAC.c
 * @brief Library of DAC functions
 *
 * This source file contains a library of functions for utilizing the DAC on the WPI daughter board
 *
 * @date Jan 27, 2010
 * @author: Adam Panzica
 * @version 1.0 Initial version with DAC_Initialize(), DAC_Volts_to_DACcoun(), DAC_Set_Value(), and DAC_Set_LMD_Output()
 * @version 1.1 Modified all functions to improve functionality. Also made all functions except for DAC_Initialize() and DAC_Set_LMD_Output 'private' by excluding a prototype in the header
 * @todo Once DAC LMD output functionality is completely working, re-write functions to use a DAC class to improve hardware abstraction
 */

#include "DAC.h"
#include "SPI.h"
#include "Simple_Serial.h"
#include <avr/io.h>

/**@def LD1
 * Bit number for the LD1 bit in the DAC control word
 */
#define LD1 7
/**@def LD0
 * Bit number for the LD0 bit in the DAC control word
 */
#define LD0 6
/**@def SEL1
 * Bit number for the SEL1 bit in the DAC control word
 */
#define SEL1 5
/**@def SEL0
 * Bit number for the SEL0 bit in the DAC control word
 */
#define SEL0 4



/**@fn 		char DAC_Initilize()
 *@brief	Initializes the SPI bus for use with the DAC
 */
char DAC_Initilize()
{
	//Initialize SPI mode to master, MSB first, Clock leading edge high, sample on trailing edge
	Serial_Print_String("Initializing SPI\r\n");
	SPI_Initialize(MSTRSEL,MSBF,CPLLH,CPHST);
	return 1;
}

/**@fn 		int DAC_Volts_To_DACcount(int mVolts)
 *@brief	Convert a voltage in mV to a 10bit DAC value
 *@param	[in] mVolts Voltage in mV
 *
 * This function takes a voltage in mV and converts it into a 10bit value that corresponds to that voltage on the DAC
 */
int DAC_Volts_To_DACcount(int mVolts)
{
	long DACpermVolt;
	long temp;

	DACpermVolt = ((long)DACRES*10000)/((long)VREF);
	temp = (long)mVolts*DACpermVolt/10000;

	return (int)temp;
}

/**@fn 		char DAC_Set_Value (int DACcounts, unsigned char channel)
 *@brief	Sets a DAC channel to a specified value
 *@param	[in] DACcounts 10bit value to set the DAC output to
 *@param	[in] channel Channel to adjust the output of
 *
 * This function sets a specified DAC channel to a new output value
 */
char DAC_Set_Value (int DACcounts, unsigned char channel)
{
	char packet[2]={0,0};

	//check to make sure SPI is enabled
	if(!(SPCR & (1<<SPE))) return 0;

	//Sanitize DACcounts to prevent buffer overflow
	if(DACcounts >= DACRES) DACcounts = DACRES-1;
	if(DACcounts < 0) DACcounts = 0;
	//pack the DACcount data bits into two 8bit chars for transmission
	packet[0] = DACcounts>>8;
	packet[1] = (char) DACcounts;

	//pack the DAC operation mode bits onto packetHigh
	packet[0] |=  (1<<LD1)|(0<<LD0);

	switch(channel)
	{
		case 'a':
		case 'A':
		case 0:
			//select channel A
			packet[0] |= (0<<SEL1)|(0<<SEL0);
			break;
		case 'b':
		case 'B':
		case 1:
			//select channel B
			packet[0] |= (0<<SEL1)|(1<<SEL0);
			break;
		case 'c':
		case 'C':
		case 2:
			//select channel C
			packet[0] |= (1<<SEL1)|(0<<SEL0);
			break;
		case 'd':
		case 'D':
		case 3:
			//select channel D
			packet[0] |= (1<<SEL1)|(1<<SEL0);
			break;

	}

	//Send data packets
	/*Serial_Print_String("Sending Packet: ");
	Serial_Print_Int((int)packet[0],2);
	Serial_Print_String("   ");
	Serial_Print_Int((int)packet[1],2);
	Serial_Print_String("\r\n");*/
	SPI_Send_Packet(packet, 2, SSPIN_DAC);

	return 1;
}

/**@fn 		char DAC_Set_LMD_Output (int mVolts, char channel)
 *@brief	Sets a specified LMD output to a specified voltage
 *@param	[in] mVolts voltage to set the LMD output to, in mV
 *@param	[in] channel Channel to adjust the output of
 */
char DAC_Set_LMD_Output (int mVolts, char channel)
{
	long tempDACVolts;
	int tempDACCounts;

	//DAC_Initilize();

	if((mVolts <= LMDREFH)&&(mVolts >= LMDREFL))
	{
		tempDACVolts = (((long)(mVolts)*20))/((long)GAIN+1)+(long)VSHIFT;
		/*Serial_Print_String("DAC Volts: ");
		Serial_Print_Int((int)tempDACVolts, 10);
		Serial_Print_String("\r\n");*/
		tempDACCounts = DAC_Volts_To_DACcount((int)tempDACVolts);
		/*Serial_Print_String("DAC Counts: ");
		Serial_Print_Int(tempDACCounts, 10);
		Serial_Print_String("\r\n");*/
		return DAC_Set_Value (tempDACCounts, channel);
	}
	else return 0;
}
