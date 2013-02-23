/**
 * @file SPI.h
 * @brief Header file for SPI library
 * @date Jan 27, 2010
 * @author: Adam Panzica
 */

#ifndef SPI_H_
#define SPI_H_

char SPI_Initialize(char master, char dataOder, char clockPolarity, char clockPhase);
char SPI_Send_Packet(char *packet, int length, char SSPin);
char SPI_Read_Packet(char *packet, int length, char SSPin);
char SPI_Send_Byte(char cData);
char SPI_Read_Byte();

/******************************** SYMBOLIC CONSTANTS ********************/

/**@def SSPIN_DAC
 * Pin value for SS of the DAC on the SPI bus
 */
#define SSPIN_DAC 2

/**@def SSPIN_ENC
 * Pin value for SS of the Encoder on the SPI bus
 */
#define SSPIN_ENC 1


/**@def MSTRSEL
 * Constant to select master mode when initializing SPI mode
 */
#define MSTRSEL 1
/**@def SLV
 * Constant to select slave mode when initializing SPI mode
 */
#define SLV 0

/**@def MSBF
 * Constant to select Most Significant Bit First when initializing SPI mode
 */
#define MSBF 0
/**@def LSBF
 * Constant to select Least Significant Bit First when initializing SPI mode
 */
#define LSBF 1;

/**@def CPLLH
 * Constant to set clock polarity to leading edge high when initializing SPI mode
 */
#define CPLLH 0
/**@def CPLTH
 * Constant to set clock polarity to trailing edge high when initializing SPI mode
 */
#define CPLTH 1

/**@def CPHSL
 * Constant to set clock phase to sample on the leading edge when initializing SPI mode
 */
#define CPHSL 0
/**@def CPHST
 * Constant to set clock phase to sample on the trailing edge when initializing SPI mode
 */
#define CPHST 1

#endif /* SPI_H_ */
