/** @brief Lab 2 main source file
*
* @file Lab2.c
*
* This source file contains the main code loops for Lab 2.
* Additional functionality is provided by various header files
*
* @author Adam Panzica
* @author Joel Sutherland
* @date 27-Jan-2009
*
*/
#include "Simple_Serial.h"
#include "portIO.h"
#include "ADC.h"
#include "Timer16_Functions.h"
#include "DAC.h"
#include "PID.h"
#include "Arm_Kinematics_Functions.h"
#include <avr\io.h>
#include <util\delay.h>
#include <avr\interrupt.h>

/**Overwrites default unhandled interrupt vector*/
ISR(BADISR_vect)
{
	Serial_Print_String("Unhandled Interrupt\r\n");
}


#define POTMAX	875
#define POTMIN	175


#define POT90	529
#define POT0	865

#define DEGPERPOT0 2486
#define PTDOFFSET0 -43
#define POTPERDEG0 4022
#define DTPOFFSET0 175

#define DEGPERPOT1 -2678
#define PTDOFFSET1 232
#define POTPERDEG1 -3733
#define DTPOFFSET1 867 //863

#define TRIPRECISION 7	// Number of data points in the triangle

int Pot_To_Degrees(int potVal, int channel);
int Degrees_To_Pot(int degrees, int channel);
int within(int range, int comp1, int comp2);

/** Program entry point*/
int main()
{
	int rSense;
	char input;

	int linkLength0 = 200;
	int linkLength1 = 170;
	int jointAngle0;
	int jointAngle1;
	int xLocal;
	int yLocal;


	int channel0Target = 0;
	int channel0Current = 0;
	int channel0CorrectionVal = 0;
	int channel0LMDVal = 0;

	int channel1Target = 0;
	int channel1Current = 0;
	int channel1CorrectionVal = 0;
	int channel1LMDVal = 0;

	int trif = 0;
	int tristep = 0;

	int triangleAngles[TRIPRECISION][2] = {{0, 90},{10, 58},{21, 27},{31, 6},{31, 20},{28, 21},{12, 60}};

	DDRD = 0xFF;
	DDRC = 0x00;

	_delay_ms(1000);

	Init_Serial_P(BAUD576, FRM8, STOP1, NOPAR);
	ADC_Init_P();
	DAC_Initilize();
	PID_Initialize();

	PID_Add_Channel(&channel0Target, &channel0Current, &channel0CorrectionVal, &channel0LMDVal, 550, 30, 150, 150);
	PID_Add_Channel(&channel1Target, &channel1Current, &channel1CorrectionVal, &channel1LMDVal, 450, 30, 200, 150);

	//Serial_Print_String("Motor Test\r\n");

	channel0Target = Degrees_To_Pot(0, 0);
	channel1Target = Degrees_To_Pot(90, 1);
	while(1)
	{
		rSense = ADC_Get_Value(1);
		channel0Current = ADC_Get_Value(0);
		channel1Current = ADC_Get_Value(2);
		jointAngle0 = Pot_To_Degrees(channel0Current, 0);
		jointAngle1 = Pot_To_Degrees(channel1Current, 1);
		AKF_2link_Forward(&jointAngle0, &jointAngle1, &linkLength0, &linkLength1, &xLocal, &yLocal);
		input = PINC;
		switch(input)
		{
		case 0b11111110:
			trif = 0;
			channel0Target = Degrees_To_Pot(0, 0);
			channel1Target = Degrees_To_Pot(90, 1);
			break;
		case 0b11111101:
			trif = 0;
			channel0Target = Degrees_To_Pot(30, 0);
			channel1Target = Degrees_To_Pot(60, 1);
			break;
		case 0b11111011:
			trif = 0;
			channel0Target = Degrees_To_Pot(45, 0);
			channel1Target = Degrees_To_Pot(45, 1);
			break;
		case 0b11110111:
			trif = 0;
			channel0Target = Degrees_To_Pot(60, 0);
			channel1Target = Degrees_To_Pot(30, 1);
			break;
		case 0b11101111:
			trif = 0;
			channel0Target = Degrees_To_Pot(90, 0);
			channel1Target = Degrees_To_Pot(0, 1);
			break;
		case 0b11011111:
			trif = 1;
			tristep = 0;
			break;
		case 0b10111111:
			trif = 2;
			tristep = 0;
			break;
		default:
			break;
		}

		if(trif == 1)
		{
			//Serial_Print_String("\r\nTriangle! Tstep: ");
			//Serial_Print_Int(tristep, 10);
			//Serial_Print_String("\r\n");
			switch (tristep)
			{
			case 0:
				channel0Target = Degrees_To_Pot(45, 0);
				channel1Target = Degrees_To_Pot(45, 1);
				if(within(10, channel0Target, channel0Current) && within(10, channel1Target, channel1Current)) tristep++;
				break;
			case 1:
				channel0Target = Degrees_To_Pot(60, 0);
				channel1Target = Degrees_To_Pot(0, 1);
				if(within(10, channel0Target, channel0Current) && within(10, channel1Target, channel1Current)) tristep++;
				break;
			case 2:
				channel0Target = Degrees_To_Pot(30, 0);
				channel1Target = Degrees_To_Pot(30, 1);
				if(within(10, channel0Target, channel0Current) && within(10, channel1Target, channel1Current)) tristep++;
				break;
			default:
				tristep = 0;
				break;
			}
		}
		if(trif == 2)
		{
			//Serial_Print_String("\r\nTriangle 2! Tstep: ");
			//Serial_Print_Int(tristep, 10);
			//Serial_Print_String("\r\n");
			channel0Target = Degrees_To_Pot(triangleAngles[tristep][0], 0);
			channel1Target = Degrees_To_Pot(triangleAngles[tristep][1], 1);
			if((((channel0Current - channel0Target) < 10) || ((channel0Target - channel0Current) > 10))&&(((channel1Current - channel1Target) < 10) || ((channel1Target - channel1Current) > 10))) tristep++;
			if(tristep == TRIPRECISION) tristep = 0;
		}
		//Data Logging

		//Serial_Print_String("\r\nTarget 0: ");
		Serial_Print_Int(Pot_To_Degrees(channel0Target, 0),10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nCurrent 0: ");
		Serial_Print_Int(jointAngle0,10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nCorrecttion 0: ");
		Serial_Print_Int(channel0CorrectionVal,10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nRSense 0: ");
		Serial_Print_Int(rSense,10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nLMD Value 0:");
		Serial_Print_Int(channel0LMDVal, 10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nTarget 1: ");
		Serial_Print_Int(Pot_To_Degrees(channel1Target, 1),10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nCurrent 1: ");
		Serial_Print_Int(jointAngle1,10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nCorrecttion 1: ");
		Serial_Print_Int(channel1CorrectionVal,10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nRSense 1: ");
		Serial_Print_Int(rSense,10);
		Serial_Print_String(",");
		//Serial_Print_String("\r\nLMD Value 1:");
		Serial_Print_Int(channel1LMDVal, 10);
		Serial_Print_String(",");
		Serial_Print_Int(xLocal, 10);
		Serial_Print_String(",");
		Serial_Print_Int(yLocal, 10);
		Serial_Print_String("\r\n");
		//Serial_Print_String("\r\n\r\n");
		_delay_ms(50);
	}
	return 1;
}

/**@fn 		int Pot_To_Degrees(int potVal, int channel)
 *@brief	Converts a 10bit pot value to an angle in degrees
 *@param	[in] potVal 10bit pot value
 *@returns	angle in degrees
 */
int Pot_To_Degrees(int potVal, int channel)
{
	long temp = 0;

	switch(channel)
	{
	case 0:
		temp = (long)potVal*(long)DEGPERPOT0/10000+(long)PTDOFFSET0;
		break;
	case 1:
		temp = (long)potVal*(long)DEGPERPOT1/10000+(long)PTDOFFSET1;
		break;
	default:
		break;
	}

	return (int)temp;
}

/**@fn 		int Degrees_To_Pot(int degrees, int channel)
 *@brief	Converts an angle in degrees to a 10bit pot value
 *@param	[in] degrees angle in degrees
 *@returns	10bit pot value
 */
int Degrees_To_Pot(int degrees, int channel)
{
	long temp = 0;

	switch(channel)
	{
	case 0:
		temp = (long)degrees*(long)POTPERDEG0/1000+(long)DTPOFFSET0;
		break;
	case 1:
		temp = (long)degrees*(long)POTPERDEG1/1000+(long)DTPOFFSET1;
		break;
	default:
		break;
	}

	return (int)temp;
}
int within(int range, int comp1, int comp2){
	if(((comp1) >= comp2) && ((comp1-comp2) < range)) return 1;
	if(((comp1) < comp2) && ((comp2-comp1) < range)) return 1;
	return 0;
}
