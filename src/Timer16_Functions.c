/**
 * @file Timer16_Functions.c
 * @brief Functions for generating square waves
 *
 *This source file contains functions for generating square waves using the 16bit timer on the AVR and interrupts
 *
 *@date Jan 25, 2010
 *@author Adam Panzica
 *@author Joel Sutherland
 *@version 1.0 Initial version with Timer_Initilize() and Timer_Set_Duty_Cycle(), and ISR(TIMER1_COMPA) and ISR(TIMER1_CAPT)
 *@version 1.1 Reworked Timer_Initilize() and Timer_Set_Duty_Cycle() to Square_Wave16_Initialize() and Square_Wave16_Set_Duty_Cycle(). Added Square_Wave16_Set_Duty_Cycle() and Square_Wave16_Set_Freq()
 *@version 2.0 Changed file name to Timer16_Functions, added Timer16_Initialize() and Timer16_Set_Freq(), and adjusted ISR's to allow for timer specific code as well as PWM code
 */

/**@def PRESCALE
 * @brief prescale value for timer calculations*/
#define PRESCALE 1024

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Simple_Serial.h"
#include "Timer16_Functions.h"
#include "Encoder.h"
#include "ADC.h"
#include "IR_Sensor.h"
/**@var mFlag
 * @brief flag for ISR's to select between Timer and PWM operation
 */
static char mFlag = 0;

static char count = 0;

//lab1 specific counting flag
static int timestamp = 0;


/**@fn ISR(TIMER1_COMPA_vect)
 * @brief ISR handler for Timer1 compare A match
 *
 * This ISR is run when the value in OCR1A matches TCNT1. It blinks an LED on PORTB and toggles a pin on PORTD
 */
ISR(TIMER1_COMPA_vect){
	int vRef;
	int xVal;
	int yVal;
	int zVal;
	switch(mFlag)
	{
		case 0:

			//Insert Timer mode handling code here
			/*xVal = ADC_MCP3204_Get_Value(0, 0);
			yVal = ADC_MCP3204_Get_Value(1, 0);
			zVal = ADC_MCP3204_Get_Value(2, 0);
			vRef = ADC_MCP3204_Get_Value(3, 0);
			Serial_Print_Int(ADC_MCP3204_Count_to_Gs(xVal, vRef), 10);
			Serial_Print_String(", ");
			Serial_Print_Int(ADC_MCP3204_Count_to_Gs(yVal, vRef), 10);
			Serial_Print_String(", ");
			Serial_Print_Int(ADC_MCP3204_Count_to_Gs(zVal, vRef), 10);
			Serial_Print_String(", ");
			Serial_Print_Int(ADC_Get_Value(0), 10);
			Serial_Print_String(", ");
			Serial_Print_Int(ADC_Get_Value(2), 10);
			Serial_Print_String(", ");*/
			/*
			Serial_Print_Int(ADC_Get_Value(4), 10);
			Serial_Print_String(",   ");
			Serial_Print_Int(ReadIR(4), 10);
			Serial_Print_String("\r\n");
			*/
			break;
		case 1:
			//Insert PWM mode handling code here
			PORTB = PORTB &(~(1<<PB3));
			PORTD = PORTD & ~(1<<PD6);
			break;
		default:
			break;
	}
}

/**@fn ISR(TIMER1_OVF_vect)
 * @brief ISR handler for Timer1 overflow
 *
 * This ISR is run when the value TCNT1 matches ICR1. It blinks an LED on PORTB and toggles a pin on PORTD
 */
ISR(TIMER1_OVF_vect){
	PORTB = PORTB|(1<<PB3);
	PORTD = PORTD | (1<<PD6);
}

/**@fn 		void Square_Wave16_Initialize(unsigned int frequency, unsigned int fclk)
 *@brief	Initialize the square wave generator with a frequency
 *@param	[in] frequency the desired frequency in Hz
 *@param	[in] fclk the base clock of the processor in MHz/100
 *
 *This function initializes a square wave generator using the AVR's 16bit timer. Default duty cycle is 0%
 */
void Square_Wave16_Initialize(unsigned int frequency, unsigned int fclk)
{
	//Set the mFlag to PWM mode
	mFlag = 1;

	//Enable Interrupts, set interrupt flags in TIMSK1
	sei();
	TIMSK1 = (1<<OCIE1A)|(1<<TOIE1);

	//calculate value of TOP from desired frequency and base clock
	Square_Wave16_Set_Freq(frequency, fclk);
	OCR1A = 0;

	//Set up TCCR1A/B/C for FastPWM operation with ICRn as the TOP value
	TCCR1A|= (1<<WGM11)|(0<<WGM10);
	TCCR1B = (1<<WGM13)|(1<<WGM12);
	TCCR1C = 0;

	//Set output pin states
	TCCR1A|= (0<<COM1A1)|(1<<COM1A0) | (0<<COM1B1) | (1<<COM1B0);

	//Set Prescaler
	TCCR1B|= (1<<CS12)|(0<<CS11)|(1<<CS10);
}

/**@fn 		void Timer16_Initialize(unsigned int frequency, unsigned int fclk)
 *@brief	Initialize a 16bit timer
 *@param	[in] frequency the desired frequency in Hz
 *@param	[in] fclk the base clock of the processor in MHz/100
 *
 *This function initializes a 16bit timer utilizing OCR1A for the TOP value
 */
void Timer16_Initialize(unsigned int frequency, unsigned int fclk)
{
	//Set the mFlag to Timer mode
	mFlag = 0;

	//Enable Interrupts, set interrupt flags in TIMSK1
	sei();
	TIMSK1 = (1<<OCIE1A)|(1<<TOIE1);

	//calculate value of TOP from desired frequency and base clock
	Square_Wave16_Set_Freq(frequency, fclk);
	OCR1A = 0;

	//Set up TCCR1A/B/C for CTC operation depending on mode flag with ICRn as the TOP value
	TCCR1A|= (0<<WGM11)|(0<<WGM10);
	TCCR1B = (1<<WGM13)|(1<<WGM12);
	TCCR1C = 0;

	//Set output pin states
	TCCR1A|= (0<<COM1A1)|(0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0);

	//Set Prescaler
	TCCR1B|= (1<<CS12)|(0<<CS11)|(1<<CS10);
}

/**@fn 		void Square_Wave16_Set_Duty_Cycle(char dutyCycle)
 *@brief	Set the duty cycle of the square wave
 *@param	[in] dutyCycle the desired duty cycle to be set in %, 0-100
 *
 *This function sets the duty cycle for the square wave being generated by the 16bit timer.
 */
void Square_Wave16_Set_Duty_Cycle(char dutyCycle)
{
	unsigned long temp;

	temp = ((long)(ICR1)*(long)dutyCycle)/100;
	OCR1A = (int)temp;
}

/**@fn 		void Square_Wave16_Set_Freq(unsigned int frequency, unsigned int fclk)
 *@brief	Set the frequency of the square wave
 *@param	[in] frequency Desired frequency to be set
 *@param	[in] fclk Base clock rate
 *
 *This function sets the frequency for the square wave being generated by the 16bit timer.
 */
void Square_Wave16_Set_Freq(unsigned int frequency, unsigned int fclk)
{
	long temp;
	int top;

	temp = ((long)fclk/((long)frequency))*1000/PRESCALE-1;
	top = (int)temp;
	ICR1 = top;
	//reset the timer to ensure the timer doesn't miss TOP and wrap around
	TCNT1 = 0x00;
}

/**@fn 		void Timer16_Set_Freq(unsigned int frequency, unsigned int fclk)
 *@brief	Set the frequency of the timer
 *@param	[in] frequency Desired frequency to be set
 *@param	[in] fclk Base clock rate
 *
 *This function sets the frequency for the timer generated by the 16bit timer.
 */
void Timer16_Set_Freq(unsigned int frequency, unsigned int fclk)
{
	long temp;
	int top;

	temp = ((long)fclk/((long)frequency))*1000/PRESCALE-1;
	top = (int)temp;
	OCR1A = top;
	//reset the timer to ensure the timer doesn't miss TOP and wrap around
	TCNT1 = 0x00;
}

/**@fn 		unsigned char ADC_To_Duty(int ADCVal)
 *@brief	Converts an ADC value to a duty cycle
 *@param	[in] ADCVal 10bit ADC value
 *@return 	Duty cycle (0-100%)
 *
 * This function converts a 10bit ADC value to a duty cycle from
 * 0 to 100%
 */
unsigned char ADC_To_Duty(int ADCVal){
	long temp;
	temp = ((long) (ADCVal)*100)/1024;
	return (unsigned char) temp;
}
