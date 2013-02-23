/*
 * Datalogging.h
 *
 *  Created on: Jan 22, 2010
 *      Author: apanzica
 */

#ifndef DATALOGGING_H_
#define DATALOGGING_H_

char Log_Write(int data, int *buffer, int *buffTop, int *size);
int  Log_Read(int *buffer, int *buffTop);

#endif /* DATALOGGING_H_ */
