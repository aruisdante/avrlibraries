/**
 * @file Simple_Serial.h
 *
 *@date Jan 26, 2010
 *@author Adam Panzica
 */
#ifndef SIMPLE_SERIAL_H_
#define SIMPLE_SERIAL_H_

char Init_Serial_P( unsigned int baud, unsigned char frameSize, unsigned char stopBits, unsigned char parity );
char Init_Serial_I( unsigned int baud, unsigned char frameSize, unsigned char stopBits, unsigned char parity );

char Serial_Print_Char( unsigned char data);
char Serial_Read_Char(unsigned char* data);

char Serial_Print_String(char* string);
char Serial_Print_Int( int data, int base);

/******************* SYMBOLIC CONSTANTS ****************************/
/**@def BAUD24
 * @brief Represents a baudrate of 2400bps*/
#define BAUD24 2400
/**@def BAUD48
 * @brief Represents a baudrate of 4800bps*/
#define BAUD48 4800
/**@def BAUD96
 * @brief Represents a baudrate of 9600bps*/
#define BAUD96 9600
/**@def BAUD144
 * @brief Represents a baudrate of 14.4kbps*/
#define BAUD144 14400
/**@def BAUD192
 * @brief Represents a baudrate of 19.2kbps*/
#define BAUD192 19200
/**@def BAUD288
 * @brief Represents a baudrate of 28.8kbps*/
#define BAUD288 28800
/**@def BAUD384
 * @brief Represents a baudrate of 38.4kbps*/
#define BAUD384 38400
/**@def BAUD576
 * @brief Represents a baudrate of 57.6kbps*/
#define BAUD576 57600
/**@def BAUD768
 * @brief Represents a baudrate of 76.8kbps*/
#define BAUD768 76800
/**@def BAUD1152
 * @brief Represents a baudrate of 115.2kbps*/
#define BAUD1152 115200
/**@def BAUD2304
 * @brief Represents a baudrate of 230.4kbps*/
#define BAUD2304 230400
/**@def BAUD2500
 * @brief Represents a baudrate of 250.0kbps*/
#define BAUD2500 250000

/**@def FRM9
 * @brief Represents a frame size of 9 bits*/
#define FRM9 9
/**@def FRM8
 * @brief Represents a frame size of 8 bits*/
#define FRM8 8
/**@def FRM7
 * @brief Represents a frame size of 7 bits*/
#define FRM7 7
/**@def FRM6
 * @brief Represents a frame size of 6 bits*/
#define FRM6 6
/**@def FRM5
 * @brief Represents a frame size of 5 bits*/
#define FRM5 5

/**@def STOP1
 * @brief Represents 1 stop bit*/
#define STOP1 1
/**@def STOP2
 * @brief Represents 2 stop bits*/
#define STOP2 2

/**@def NOPAR
 * @brief Represents no parity*/
#define NOPAR 0
/**@def ODDPAR
 * @brief Represents odd parity*/
#define ODDPAR 2
/**@def EVPAR
 * @brief Represents even parity*/
#define EVPAR 1

#endif /* SIMPLE_SERIAL_H_ */
