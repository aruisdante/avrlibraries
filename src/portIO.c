/**
 * @brief This file adds functions that allow for special port IO function
 * @file portIO.c
 *
 * @date Jan 20, 2010
 * @author Joel Sotherland
 * @author Adam Panzica
 * @version 1.0 Initial version with Pin_Read() and Pin_Set()
 */

#include <avr/io.h>


/**@fn 		char Pin_Read( unsigned char port, unsigned char pin )
 * @brief	Read an individual pin for a speccified port
 * @param	[in] port desired port to be read from
 * @param	[in] pin desired pin to be red from
 * @return 	value of the selected pin
 *
 * This function reads the value from a single pin from a specified port
 */
unsigned char Pin_Read(unsigned char port, unsigned char pin)
{
	unsigned char data;
	switch(port) {
		case 'A':
			data = PINA;
			break;
		case 'B':
			data = PINB;
			break;
		case 'C':
			data = PINC;
			break;
		case 'D':
			data = PIND;
			break;
		default:
			return 0;
	}
	data = (data>>pin)&1;
	return data;
}


/**@fn 		char Pin_Set( unsigned char port, unsigned char pin, unsigned char value )
 * @brief	Write to an individual pin on a specified port
 * @param	[in] port desired port to be read from
 * @param	[in] pin desired pin to be red from
 * @param	[in] value desied value to set the pin to
 * @return 	0 if an invalid port/pin was specified, else 1
 *
 * This function writes to a single pin on a port without changing the values of the other pins.
 */
unsigned char Pin_Set(unsigned char port, unsigned char pin, unsigned char value)
{
	unsigned char bitmask;

	if((pin <= 7) && (pin >= 0))
	{
		if(value == 1) bitmask = 1<<pin;
		else if (value == 0) bitmask = 0xFF^(1<<pin);
		else return 0;

		switch(port)
		{
			case 'A':
				if(value == 1)PORTA = PORTA|bitmask;
				else PORTA = PORTA&bitmask;
				break;
			case 'B':
				if(value == 1)PORTB = PORTB|bitmask;
				else PORTB = PORTB&bitmask;
				break;
			case 'C':
				if(value == 1)PORTC = PORTC|bitmask;
				else PORTC = PORTC&bitmask;
				break;
			case 'D':
				if(value == 1)PORTD = PORTD|bitmask;
				else PORTD = PORTD&bitmask;
				break;
			default:
				return 0;
		}
		return 1;
	}
	else return 0;
}
