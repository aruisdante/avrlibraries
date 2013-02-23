/**
 * @file Timer8_Functions.c
 * @brief library of functions for using 8bit Timer 0
 * @date Feb 4, 2010
 * @author Adam Panzica
 *
 * @version 1.0 Initial version with Timer8_Initialize_CTC()
 */
#include "Timer8_Functions.h"
#include "Simple_Serial.h"
#include "portIO.h"
#include <avr/io.h>
#include <avr/interrupt.h>


/** Constant representing the base clock of the AVR*/
#define FCLK		18432000
/** Constant representing the minimum output frequency possible with a prescale of 1*/
#define FMINPS1		36000
/** Constant representing the minimum output frequency possible with a prescale of 8*/
#define FMINPS8		4500
/** Constant representing the minimum output frequency possible with a prescale of 64*/
#define FMINPS64	562
/** Constant representing the minimum output frequency possible with a prescale of 256*/
#define FMINPS256	140
/** Constant representing the minimum output frequency possible with a prescale of 1024*/
#define FMINPS1024	35

/**@fn void Timer8_Initialize_CTC(unsigned int frequency)
 * @brief Sets up the timer to trigger an interrupt at a set frequency
 * @param [in] frequency desired frequency of the trigger, in Hz
 *
 * This function sets up the AVR's 8bit Timer 0 to run using CTC mode in order to trigger an interrupt at a set frequency.
 * NOTE: The minimum frequency possible given the 18.432MHz base clock is 35MHz. The maximum frequency is 18.432MHz/2
 */
void Timer8_Initialize_CTC(unsigned int frequency)
{
	unsigned long top;
	unsigned int prescale = 1;
	unsigned char tempTCCR0A = 0;
	unsigned char tempTCCR0B = 0;

	//Enable interrupts and set interrupt enable
	sei();
	TIMSK0 |= (1<<OCIE0A);

	//calculate the prescale based on the requested frequency
	if(frequency >= FMINPS1) prescale = 1;
	else if ((frequency < FMINPS1) && (frequency >= FMINPS8)) prescale = 8;
	else if ((frequency < FMINPS8) && (frequency >= FMINPS64)) prescale = 64;
	else if ((frequency < FMINPS64) && (frequency >= FMINPS256)) prescale = 256;
	else if (frequency < FMINPS256) prescale = 1024;

	//calculate the TOP value for the requested frequency
	top = (((long)FCLK)/(2*(long)prescale*(long)frequency))-1;

	//Set up TCCR0A for CTC mode
	tempTCCR0A |= (1<<WGM01)|(0<<WGM00);

	//set up TCCR0B for use with the calculated prescale
	switch(prescale)
	{
		case 1:
			tempTCCR0B |= (0<<CS02)|(0<<CS01)|(1<<CS00);
			break;
		case 8:
			tempTCCR0B |= (0<<CS02)|(1<<CS01)|(0<<CS00);
			break;
		case 64:
			tempTCCR0B |= (0<<CS02)|(1<<CS01)|(1<<CS00);
			break;
		case 256:
			tempTCCR0B |= (1<<CS02)|(0<<CS01)|(0<<CS00);
			break;
		case 1024:
			tempTCCR0B |= (1<<CS02)|(0<<CS01)|(1<<CS00);
			break;
		default:
			break;
	}

	OCR0A = (int)top;
	TCCR0A = tempTCCR0A;
	TCCR0B = tempTCCR0B;
}

/**@fn void Timer8_Initialize_US_Pulse()
 * @brief Sets up the timer to generate a pulse at set intervals
 *
 * This function sets up the AVR's 8bit Timer 0 to generate a 1ms pulse every 31ms. It generates this continuously, trapping the processor in an infinate loop.
 */
void Timer8_Initialize_US_Pulse()
{
	int i = 0;
	int startTime;
	int currTime;

	DDRB |= (1<<PB3);
	TCCR0A = 0;
	TCCR0B = (1<<CS02)|(0<<CS01)|(1<<CS00);
	while(1)
	{
		Pin_Set('B', PB3, 1);
		while(i <= 31)
		{
			startTime = TCNT0;
			currTime = TCNT0;

			//if there will not be a valid frame window without roll over, reset the timer
			if (startTime > 245)
			{
				TCNT0 = 0;
				startTime = 0;
				currTime = 0;
			}
			//idle waiting for 1ms to pass
			while((currTime-startTime) <= 18 )
			{
				currTime = TCNT0;
			}
			//increment the ms counter
			i++;
		}
		i = 0;
		Pin_Set('B', PB3, 0);
		TCNT0 = 0;
		startTime = 0;
		currTime = 0;
		//idle waiting for 1ms to pass
		while((currTime-startTime) <= 18 )
		{
		currTime = TCNT0;
		}
	}

}
