/*
 * Lab3.c
 *
 *  Created on: Feb 10, 2010
 *      Author: Aruis
 */

#include "Simple_Serial.h"
#include "portIO.h"
#include "ADC.h"
#include "Timer8_Functions.h"
#include "Timer16_Functions.h"
#include "DAC.h"
#include "PID.h"
#include "Arm_Kinematics_Functions.h"
#include "Encoder.h"
#include <avr\io.h>
#include <util\delay.h>
#include <avr\interrupt.h>

/**Overwrites default unhandled interrupt vector*/
ISR(BADISR_vect)
{
	Serial_Print_String("Unhandled Interrupt\r\n");
}

#define SAMPLERATE 1000
#define FCLK	18432
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
#define DTPOFFSET1 862

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

	long encCounts;


	DDRD = 0xFF;
	DDRC = 0x00;

	_delay_ms(1000);

	Init_Serial_P(BAUD576, FRM8, STOP1, NOPAR);
	ADC_Init_P();
	Encoder_Initialize();
	//DAC_Initilize();
	//PID_Initialize();
	//Timer16_Initialize(SAMPLERATE, FCLK);

	//PID_Add_Channel(&channel0Target, &channel0Current, &channel0CorrectionVal, &channel0LMDVal, 450, 10, 150, 150);
	//PID_Add_Channel(&channel1Target, &channel1Current, &channel1CorrectionVal, &channel1LMDVal, 550, 10, 150, 150);

	//channel0Target = Degrees_To_Pot(0, 0);
	//channel1Target = Degrees_To_Pot(90, 1);

	while(1)
	{
		//Encoder_Read(11, &encCounts);
	}
	/*while(1)
	{
		//rSense = ADC_Get_Value(1);
		//channel0Current = ADC_Get_Value(0);
		//channel1Current = ADC_Get_Value(2);
		//jointAngle0 = Pot_To_Degrees(channel0Current, 0);
		//jointAngle1 = Pot_To_Degrees(channel1Current, 1);
		//AKF_2link_Forward(&jointAngle0, &jointAngle1, &linkLength0, &linkLength1, &xLocal, &yLocal);
		Serial_Print_String("\r\nEncoder Test \r\n");
		Serial_Print_Int(EncoderCounts(0), 10);
		input = PINC;
		switch (input)
		{
		case 0b11111110:
			DAC_Set_LMD_Output(0, 1);
			Serial_Print_String("\r\n 0 Volts");
			break;
		case 0b11111101:
			DAC_Set_LMD_Output(-3000, 1);
			Serial_Print_String("\r\n -3 Volts");
			break;
		case 0b11111011:
			DAC_Set_LMD_Output(3000, 1);
			Serial_Print_String("\r\n 3 Volts");
			break;
		case 0b11110111:
			DAC_Set_LMD_Output(6000, 1);
			Serial_Print_String("\r\n 6 Volts");
			break;
		case 0b11101111:
			DAC_Set_LMD_Output(12000, 1);
			Serial_Print_String("\r\n 12 Volts");
			break;
		default:
			break;
		}
		/*
		switch(input){
		case 0b11111110:
			channel0Target = Degrees_To_Pot(0, 0);
			channel1Target = Degrees_To_Pot(90, 1);
			break;
		case 0b11111101:
			channel0Target = Degrees_To_Pot(90, 0);
			channel1Target = Degrees_To_Pot(90, 1);
			break;
		default:
			break;
		}
		*/

		//Serial_Print_Int(ADC_MCP3204_Get_Value(0, 0), 10);
		//Serial_Print_String("\r\n");
		//Data Logging


	//}
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
