/** @brief Lab 1 main source file
*
* @file Lab1_Demos.c
*
* This source file contains the main code loops for Lab 1.
* Additional functionality is provided by various header files
*
* @author Adam Panzica
* @author Joel Sotherland
* @date 26-Jan-2009
*/
#include "Simple_Serial.h"
#include "portIO.h"
#include "ADC.h"
#include "Timer16_Functions.h"
#include <avr\io.h>
#include <avr\delay.h>
#include <avr\interrupt.h>


void Step_2();
void Step_3();
void Step_6();
unsigned int Switch_To_Frequency(char pins);

/**@fn ISR(BADISR_vect)
 * @brief Overwrites the default unhandled interrupt vector
 *
 * Overwrites the bad ISR vector to write out an error message
 * to the serial port instead of killing the processor
 */
ISR(BADISR_vect)
{
	Serial_Print_String("Unhandled Interrupt\r\n");
}

/**@var sensor
 * @brief global struct containing the ADC sensor data
 */
struct ADC_Sensor sensor;


/**@fn 		int main()
 *@brief	Program entry point
 */
int main()
{
	//****Initialization****//
	DDRD = 0xFF;
	DDRB = 0xFF; // set the LEDs port as output
	PORTB = 0xFF;
	_delay_ms(1000);

	Init_Serial_P(BAUD192, FRM8, STOP1, NOPAR);
	ADC_Init_P();
	sensor = ADC_Sensor_Construct("Sensor1", 1, 270, 0);

	//****Functionality****//

	Step_2();
	//Step_3();
	//Step_6();

	return 1;
}

/**@fn 		unsigned int Switch_To_Frequency(char pins)
 *@brief	Converts a switch press into a frequency setting
 *@param	[in] pins The value of the pins attached to the switches on the STK500
 *@returns	A frequency value from 1-100Hz depending on the swtich pressed
 */
unsigned int Switch_To_Frequency(char pins){
	pins = ~pins;
	switch(pins){
		case 0b0:
			return 1;
			break;
		case 0b1:
			return 25;
			break;
		case 0b10:
			return 50;
			break;
		case 0b100:
			return 75;
			break;
		case 0b1000:
			return 100;
			break;
		default:
			return 1;
			break;
	}
	return 1;
}

/**@fn 		void Step_2()
 *@brief	Code for solving step 2 of the lab
 */
void Step_2(){
	int ADCTemp;
	int count=0;

	while(1)
	{
		ADCTemp = ADC_Get_Value(sensor.channel);
		Serial_Print_String(sensor.name);
		Serial_Print_String(", ");
		Serial_Print_Int(count, 10);
		Serial_Print_String(", ");
		Serial_Print_Int(ADCTemp, 10);
		Serial_Print_String(", ");
		Serial_Print_Int(ADC_Calc_Volts(ADCTemp, 5000), 10);
		Serial_Print_String(", ");
		Serial_Print_Int(ADC_Sensor_Calc_Value(&sensor, ADCTemp), 10);
		Serial_Print_String("\r\n");
		count++;
		_delay_ms(100);
	}
}

/**@fn 		void Step_3()
 *@brief	Code for solving step 3 of the lab
 */
void Step_3(){
	int ADCTemp;
	int freq = 1;
	int count = 0;
	unsigned char dutyCycle;

	Square_Wave16_Initialize(freq, 18432);
	while(1){
		count++;
		if(PINC != 0xFF){
			freq = Switch_To_Frequency(PINC);
			Square_Wave16_Set_Freq(freq, 18432);
		}

		ADCTemp = ADC_Get_Value(sensor.channel);
		dutyCycle = ADC_To_Duty(ADCTemp);
		Square_Wave16_Set_Duty_Cycle(dutyCycle);
		Serial_Print_String("PWM, ");
		Serial_Print_Int(count,10);
		Serial_Print_String(", ");
		Serial_Print_Int((int)dutyCycle,10);
		Serial_Print_String(", ");
		Serial_Print_Int(freq,10);
		Serial_Print_String(", ");
		Serial_Print_Int((int)((PIND&0b01000000)>>PD6),2);
		Serial_Print_String(", ");
		Serial_Print_Int(ADCTemp,10);
		Serial_Print_String("\r\n");
	}
}

/**@fn 		void Step_6()
 *@brief	Code for solving step 6 of the lab
 */
void Step_6(){
	int i=0xFF;
	while(i==0xFF)
	{
		i=PINC;
	}
	Timer16_Initialize(200, 18432);
	while(1);
}
