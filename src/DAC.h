/*
 * DAC.h
 *
 *  Created on: Jan 27, 2010
 *      Author: apanzica
 */

#ifndef DAC_H_
#define DAC_H_

#define VREF 5000
#define DACRES 4096
#define GAIN 95
#define VSHIFT 2500


char DAC_Initilize();
char DAC_Set_LMD_Output (int mVolts, char channel);


/**@def LMDREFH
 * Max voltage output for the LMD connected to the DAC
 */
#define LMDREFH 12000

/**@def LMDREFH
 * Min voltage output for the LMD connected to the DAC
 */
#define LMDREFL -12000

#endif /* DAC_H_ */
