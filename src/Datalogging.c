/** @brief Provide functions for simplifying datalogging
*
* @file Datalogging.c
*
* This source file contains a library of functions to simplify commonly
* used datalogging operations such as data buffering
*
* @author Adam Panzica
* @date 22-Jan-2009
* @version 1.0: Initial version with Log_Write() and Log_Read() to maintain data buffers
*/

#include <avr/io.h>
#include <stdlib.h>

/**@fn 		char Log_Write(int data, int *buffer, int *buffTop, int *size)
 *@brief	Write data to a buffer
 *@param	[in] data data to be written to the buffer
 *@param	[in] *buffer pointer to a buffer to hold the data
 *@param	[in] *buffTop pointer to a variable that will hold the current position of data in the buffer
 *@param	[in] *size pointer to a variable that has the maximum size of the buffer
 *@return 	1 if success, else 0 if there is no more room in the buffer
 *
 * This function writes an int of data onto a buffer stack. It will return 1 if it successfully writes to the buffer, or it will
 * return 0 if the maximum size of the buffer has been reached. It does NOT overwrite the last value in the buffer; it discards
 * the data if there is no more room in the buffer. Also, care must be taken that nothing outside the function writes to the values
 * stored in *buffTop or *size, as this will cause data errors and unexpected results. *buffTop is updated by this function automatically
 */
char Log_Write(int data, int *buffer, int *buffTop, int *size)
{
	if(*buffTop < *size) buffer[*buffTop] = data;
	else return 0;
	*buffTop = *buffTop+1;
	return 1;
}


/**@fn 		int Log_Read(int *buffer, int *buffTop)
 *@brief	Read data from a buffer
 *@param	[in] *buffer pointer to a buffer to hold the data
 *@param	[in] *buffTop pointer to a variable that will hold the current position of data in the buffer
 *@return 	integer value on top of the buffer
 *
 * This function reads data from a buffer. It returns the value that is on top of a buffer, and automatically de-increments
 * the buffer position. If the bottom of the buffer is reached, it will output the value on the bottom of the buffer if future calls
 * are made without writing new data to the buffer.
 */
int  Log_Read(int *buffer, int *buffTop)
{
	if(*buffTop>0)
	{
		*buffTop = *buffTop-1;
		return buffer[*buffTop+1];
	}
	return buffer[*buffTop];
}
