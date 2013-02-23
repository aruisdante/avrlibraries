/** @brief Library of functions for SPI
*
* @file SPI.c
*
* This source file contains a library of functions to utilize the AVR's hardware SPI interface
*
* @author Adam Panzica
* @date 27-Jan-2009
* @version 1.0 Initial version with SPI_Initialize(), SPI_Send_Byte(), SPI_Send_Packet()
* @version 1.1 Modified SPI_Intialize() to remove chance of the default SPI_SS pin on the AVR being set to an input and low, which would clear the master flag in SPSR
*/

#include "SPI.h"
#include "Simple_Serial.h"
#include <avr/io.h>


/**@fn 		char SPI_Initialize(char master, char dataOder, char clockPolarity, char clockPhase)
 *@brief	Initialize SPI for polling use
 *@param	[in] master selects between master and slave. Use symbolic constants in SPI.h
 *@param	[in] dataOder selects between MSB first and LSB first. Use symbolic constants in SPI.h
 *@param	[in] clockPolarity selects the clock polarity mode. Use symbolic constants in SPI.h
 *@param	[in] clockPhase selects the clock phase mode. Use symbolic constants in SPI.h
 *
 * This function initializes the hardware SPI controller
 */
char SPI_Initialize(char master, char dataOder, char clockPolarity, char clockPhase)
{
	//check if there is an SPI op in progress. If there is, idle till it is finished to prevent data corruption
	//while(!(SPSR & (1<<SPIF)));

	//ensure that SPI is not disabled by power save
	PRR = PRR & (~(1<<PRSPI));

	//Ensure the DDR is properly set for PORTB. Port B pin 2 is set as the SS pin, but the default pin (PB4) must also be set as output to avoid Master/Slave select conflicts
	DDRB = (1<<PB7)|(1<<PB5)|(1<<PB4)|(1<<SSPIN_DAC)|(1<<SSPIN_ENC);
	//Bring SSPIN high to disable communications till ready
	PORTB |= (1<<SSPIN_DAC) | (1<<SSPIN_ENC);

	//Set up SPCR based on the passed parameters
	SPCR = (1<<SPE)|(master<<MSTR)|(dataOder<<DORD)|(clockPolarity<<CPOL)|(clockPhase<<CPHA)|(1<<SPR1)|(1<<SPR0);
	return 1;
}


/**@fn 		char SPI_Send_Byte(char cData)
 *@brief	Send a byte on the SPI bus
 *@param	[in] cData byte of data to be sent on the SPI bus
 *
 * This function initializes the hardware SPI controller
 */
char SPI_Send_Byte(char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	return 1;
}

/**@fn SPI_Read_Byte()
 * @brief this function reads a byte of the AVR's hardware SPI port
 * @return 8bit value from the SPI port
 * @note Due to the way the hardware SPI port works on the AVR, this function clocks out zeros on the MOSI port as it is clocking in
 * on MISO. Ensure that this does not cause problems
 */
char SPI_Read_Byte()
{
	/* Start transmission */
	SPDR = 0;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

/**@fn 		char SPI_Send_Packet(char *packet, int length, char SSPin)
 *@brief	Send a packet over the SPI bus
 *@param	[in] *packet pointer to an array of bytes containing the packet to be sent over the SPI bus
 *@param	[in] length length of the packet, in bytes
 *@param	[in] SSPin Pin on port B to use as slave select
 *
 * This function sends a packet of data over the SPI bus
 */
char SPI_Send_Packet(char *packet, int length, char SSPin)
{
	int i;
	//Drop SSPIN low to enable communications
	PORTB &= (~(1<<SSPin));

	//Send packet one byte at a time
	for(i=0; i<length; i++)
	{
		SPI_Send_Byte(packet[i]);
	}

	//Bring SSPIN back high to disable communications
	PORTB |= 1<<SSPin;

	return 1;
}

/**@fn 		char SPI_Read_Packet(char *packet, int length, char SSPin)
 *@brief	Send a packet over the SPI bus
 *@param	[in] *packet pointer to an array of bytes to store the packet to be red over the SPI bus
 *@param	[in] length length of the packet, in bytes
 *@param	[in] SSPin Pin on port B to use as slave select
 *
 * This function reads a packet of data over the SPI bus
 */
char SPI_Read_Packet(char *packet, int length, char SSPin)
{
	int i;
	//Drop SSPIN low to enable communications
	PORTB &= (~(1<<SSPin));

	//Send packet one byte at a time
	for(i=0; i<length; i++)
	{
		packet[i] = SPI_Read_Byte();
	}

	//Bring SSPIN back high to disable communications
	PORTB |= 1<<SSPin;

	return 1;
}


