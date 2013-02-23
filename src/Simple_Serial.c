/** @brief Provide a simple serial protocol utilizing USART0
*
* @file Simple_Serial.c
*
* This source file contains a library of functions to provide simple
* serial communications (Char/String out, Char/String in), utilizing
* either polling or interrupt based transmission handling. It uses
* a single USART for communications, USART0
*
* @author Adam Panzica
* @date 19-Jan-2009
* @version 1.0 Initial version with fixed parameters for initialization functions, Print_Char(), Print_String(), Read_Char()
* @version 1.1 Modified initialization functions to allow for adjustable setup parameters.
* @version 1.2 Added Print_Int()
*/
#include <avr/io.h>
#include "Simple_Serial.h"
#include <avr/delay.h>
#include <stdlib.h>

/** clock frequency of the AVR in MHz/100*/
#define FCLK 18432

/**@fn 		char Init_Serial_P( unsigned int baud, unsigned char frameSize, unsigned char stopBits, unsigned char parity )
 *@brief	Initialize USART0 for asynchronous serial communications, polling
 *@param	[in] baud Baudrate for serial communications
 *@param	[in] frameSize number of data bits (5-9), defaults to 8 if parameter is out of bounds
 *@param	[in] stopBits number of stop bits (1-2), defaults to 1 if parameter is out of bounds
 *@param	[in] parity 1 if even parity is desired, 2 if odd parity is desired, 0 for no parity, defaults to no parity if parameter is out of bounds
 *@return 	1 if success, else 0 if there was still data in RXX/TX buffers, preventing a change to USART parameters
 *
 * This function initializes USART0 for asynchronous serial communications by setting the appropriate enable and control registers
 * In this mode, the RX and TX buffers do not generate interrupts, and thus must be polled to receive information
 */
char Init_Serial_P( unsigned int baud, unsigned char frameSize, unsigned char stopBits, unsigned char parity )
{
	unsigned char tempUCSRB = 0b0;
	unsigned char tempUCSRC = 0b0;
	int tempBaud;

	//Check to ensure that there is no data in the RX/TX buffers before changing frame size
	if((UCSR0A & (1<<RXC0)) | (UCSR0A & (1<<TXC0))) return 0;

	//Set Parity bits in UCSRC
	switch (parity)
	{
		case 0:
			tempUCSRC|= (0<<UPM01) | (0<<UPM00);
			break;
		case 1:
			tempUCSRC|= (1<<UPM01) | (0<<UPM00);
			break;
		case 2:
			tempUCSRC|= (1<<UPM01) | (1<<UPM00);
			break;
		default:
			tempUCSRC|= (0<<UPM01) | (0<<UPM00);
			break;
	}

	//Set stop bits in UCSRC
	switch(stopBits)
	{
		case 1:
			tempUCSRC |= 0<<USBS0;
			break;
		case 2:
			tempUCSRC |= 1<<USBS0;
			break;
		default:
			tempUCSRC |= 0<<USBS0;
			break;

	}

	//Set frame size in UCSR and UCSRB
	switch(frameSize)
	{
		case 9:
			tempUCSRC|= (1<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 1<<UCSZ02;
			break;
		case 8:
			tempUCSRC|= (1<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		case 7:
			tempUCSRC|= (0<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		case 6:
			tempUCSRC|= (1<<UCSZ00) | (0<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		case 5:
			tempUCSRC|= (0<<UCSZ00) | (0<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		default:
			tempUCSRC|= (1<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;

	}

	//Set enable flags
	tempUCSRB|= (1<<RXEN0) | (1<<TXEN0);

	//Set baud rate
	tempBaud = FCLK/((baud/100)*16)*10-1;
	UBRR0H = (unsigned char)(tempBaud>>8);
	UBRR0L = (unsigned char)tempBaud;

	//Set control registers
	UCSR0B = tempUCSRB;
	UCSR0C = tempUCSRC;

	return 1;
}

/**@fn 		char Init_Serial_I( unsigned int baud, unsigned char frameSize, unsigned char stopBits, unsigned char parity )
 *@brief	Initialize USART0 for asynchronous serial communications, interrupts
 *@param	[in] baud Baudrate for serial communications
 *@param	[in] frameSize number of data bits (5-9), defaults to 8 if parameter is out of bounds
 *@param	[in] stopBits number of stop bits (1-2), defaults to 1 if parameter is out of bounds
 *@param	[in] parity 1 if even parity is desired, 2 if odd parity is desired, 0 for no parity, defaults to no parity if parameter is out of bounds
 *@return 	1 if success, else 0 if there was still data in RXX/TX buffers, preventing a change to USART parameters
 *
 * This function initializes USART0 for asynchronous serial communications by setting the appropriate enable and control registers
 * In this mode, the RX and TX buffers generate interrupts
 */
char Init_Serial_I( unsigned int baud, unsigned char frameSize, unsigned char stopBits, unsigned char parity )
{
	unsigned char tempUCSRB = 0b0;
	unsigned char tempUCSRC = 0b0;
	int tempBaud;

	//Check to ensure that there is no data in the RX/TX buffers before changing frame size
	if((UCSR0A & (1<<RXC0)) | (UCSR0A & (1<<TXC0))) return 0;

	//enable RX/TX interrupt flags
	tempUCSRB |= (1<<RXCIE0) | (1<<TXCIE0) | (1<<UDRIE0);

	//Set Parity bits in UCSRC
	switch (parity)
	{
		case 0:
			tempUCSRC|= (0<<UPM01) | (0<<UPM00);
			break;
		case 1:
			tempUCSRC|= (1<<UPM01) | (0<<UPM00);
			break;
		case 2:
			tempUCSRC|= (1<<UPM01) | (1<<UPM00);
			break;
		default:
			tempUCSRC|= (0<<UPM01) | (0<<UPM00);
			break;
	}

	//Set stop bits in UCSRC
	switch(stopBits)
	{
		case 1:
			tempUCSRC |= 0<<USBS0;
			break;
		case 2:
			tempUCSRC |= 1<<USBS0;
			break;
		default:
			tempUCSRC |= 0<<USBS0;
			break;

	}

	//Set frame size in UCSR and UCSRB
	switch(frameSize)
	{
		case 9:
			tempUCSRC|= (1<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 1<<UCSZ02;
			break;
		case 8:
			tempUCSRC|= (1<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		case 7:
			tempUCSRC|= (0<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		case 6:
			tempUCSRC|= (1<<UCSZ00) | (0<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		case 5:
			tempUCSRC|= (0<<UCSZ00) | (0<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;
		default:
			tempUCSRC|= (1<<UCSZ00) | (1<<UCSZ01);
			tempUCSRB|= 0<<UCSZ02;
			break;

	}

	//Set enable flags
	tempUCSRB|= (1<<RXEN0) | (1<<TXEN0);

	//Set baud rate
	tempBaud = FCLK/((baud/100)*16)*10-1;

	UBRR0H = (unsigned char)(tempBaud>>8);

	UBRR0L = (unsigned char)tempBaud;


	//Set control registers
	UCSR0B = tempUCSRB;
	UCSR0C = tempUCSRC;

	return 1;
}

/**@fn 		char Serial_Print_Char( unsigned char data)
 * @brief	Print a single character to the serial port
 * @param	[in] data char to be sent to the serial port
 * @return 	1 if success, else 0
 *
 * This function prints a single character to
 *the serial port utilizing USART0
 */
char Serial_Print_Char( unsigned char data)
{
	// check to make sure serial port is enabled
	if(!(UCSR0B & (1<<TXEN0))) return 0;
	// Wait for empty transmit buffer
	while ( !( UCSR0A & (1<<UDRE0)) );
	// Put data into buffer, sends the data
	UDR0 = data;
	return 1;
}

/**@fn 		char Serial_Read_Char( unsigned char* data)
 * @brief	Read a single character from the serial port
 * @param	[in] data* pointer to the location to score the red byte from the serial port
 * @return 	1 if success, else 0
 *
 * This function reads a single character from
 *the serial port utilizing USART0
 */
char Serial_Read_Char(unsigned char* data)
{
	// check to make sure serial port is enabled
	if(!(UCSR0B & (1<<RXEN0))) return 0;

	// Wait for data to be received
	while ( !(UCSR0A & (1<<RXC0)) );
	//Get and return received data from buffer
	*data = UDR0;
	return 1;
}

/**@fn 		char Serial_Print_String( char* string)
 * @brief	Print a string of chars to the serial port
 * @param	[in] string* pointer to the location of a null terminated string
 * @return 	1 if success, else 0
 *
 * This function reads a single character from
 *the serial port utilizing USART0
 */
char Serial_Print_String(char* string)
{
	int i=0;

	// check to make sure serial port is enabled
	if(!(UCSR0B & (1<<TXEN0))) return 0;

	//print the string to the serial port
	while(string[i] != '\0')
	{
		Serial_Print_Char(string[i]);
		i++;
	}
	return 1;
}

/**@fn 		char Serial_Print_Int( int data, int base)
 * @brief	Print an int to the serial port
 * @param	[in] data integer to be printed to the serial port
 * @param	[in] base base to use for int to string conversion (2,10,16, etc)
 * @return 	1 if success, else 0
 *
 * This function prints a single int to
 *the serial port utilizing USART0
 */
char Serial_Print_Int( int data, int base)
{
	char buffer[17];

	//check to make sure the serial port is enabled
	if(!(UCSR0B & (1<<TXEN0))) return 0;

	//perform data conversion from int to string
	itoa(data, buffer, base);

	//print the resultant string to the serial port
	Serial_Print_String(buffer);
	return 1;
}
