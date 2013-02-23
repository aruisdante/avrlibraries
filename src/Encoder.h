/**
 *@file Encoder.h
 *
 *@date Feb 19, 2010
 *@author Adam Panzica
 */

#ifndef ENCODER_H_
#define ENCODER_H_

/***************** ENCODER COMMANDS ******************/
/** Write to MDR0*/
#define WRITEMDR0	9
/** Write to MDR1*/
#define WRITEMDR1	10
/** Read from MDR0*/
#define READMDR0	4
/** Read from MDR1*/
#define READMDR1	5
/** Read from CNTR*/
#define READCNT 	6
/** Clear CNTR*/
#define CLEARCNT	2


void Encoder_Initialize();
char Encoder_Read(char command, long *result);

#endif /* ENCODER_H_ */
