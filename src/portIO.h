/*
 * portIO.h
 *
 *  Created on: Jan 21, 2010
 *      Author: Adam Panzica and Joel Sotherland
 */

#ifndef PORTIO_H_
#define PORTIO_H_

unsigned char Pin_Read(unsigned char port, unsigned char pin);
unsigned char Pin_Set(unsigned char port, unsigned char pin, unsigned char value);

#endif /* PORTIO_H_ */
