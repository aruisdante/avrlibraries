/**
 *@file Encoder.c
 *
 *@date Feb 19, 2010
 *@author Adam Panzica
 */

#include "Encoder.h"
#include "Simple_Serial.h"
#include "SPI.h"
#include "PortIO.h"
#include <avr/interrupt.h>
#include <avr/io.h>

/************************* ENCODER COMMAND UNION ******************/
typedef union ENCODER_COMMAND{
	struct {
		unsigned _DC	:3;
		unsigned _REGSEL:3;
		unsigned _COMSEL:2;
	};
	struct {
		unsigned _COMMAND:8;
	};
} ENCODER_COMMAND;

/************************* ENCODER MDR UNIONS ******************/
typedef union ENCODER_MDR0{
	struct {
		unsigned _QCM	:2;
		unsigned _FRCM	:2;
		unsigned _INDEX	:2;
		unsigned _AI	:1;
		unsigned _CLKDIV:1;
	};
	struct {
		unsigned _MDR0:8;
	};
} ENCODER_MDR0;

typedef union ENCODER_MDR1{
	struct {
		unsigned _CMODE	:2;
		unsigned _ENABLE:1;
		unsigned _RESER	:1;
		unsigned _FLAGS	:4;
	};
	struct {
		unsigned _MDR1:8;
	};
} ENCODER_MDR1;


/************************* ENCODER OPCODES ******************/
/**Selects no register*/
#define RSNONE	0b000
/**Selects MDR0 register*/
#define RSMDR0	0b001
/**Selects MDR1 register*/
#define RSMDR1	0b010
/**Selects DTR register*/
#define RSDTR	0b011
/**Selects CNTR register*/
#define RSCNTR	0b100
/**Selects OTR register*/
#define RSOTR	0b101
/**Selects STR register*/
#define RSSTR	0b110
/**Perform a clear command*/
#define CCLR	0b00
/**Perform a read command*/
#define CRD		0b01
/**Perform a write command*/
#define CWR		0b10
/**Perform a load command*/
#define CLOAD	0b11

/********************** PRIVATE FUNCTIONS *************************/

/**@fn 		char Encoder_Initialize_SPI()
 *@brief	Initializes the AVR's hardware SPI to communicate with the LS7366R encoder chip
 */
char Encoder_Initialize_SPI()
{
	SPI_Initialize(MSTRSEL,MSBF,CPLLH,CPHSL);
	return 1;
}

/**@fn 		void Encoder_Generate_Command(unsigned char encCommand, ENCODER_COMMAND *genCom)
 *@brief	Generates a valid LS7366R encoder chip command
 *@param	[in] encCommand Command number to be generated
 *@param	[in] *genCom Pointer to an ENCODER_COMMAND union to store the generate command to
 */
void Encoder_Generate_Command(unsigned char encCommand, ENCODER_COMMAND *genCom)
{
	genCom->_DC = 0;
	switch(encCommand)
	{
	case 0:				//Generates a command to clear MDR0
		genCom->_REGSEL = RSMDR0;
		genCom->_COMSEL = CCLR;
		break;
	case 1:				//Generates a command to clear MDR1
		genCom->_REGSEL = RSMDR1;
		genCom->_COMSEL = CCLR;
		break;
	case 2:				//Generates a command to clear CNTR
		genCom->_REGSEL = RSCNTR;
		genCom->_COMSEL = CCLR;
		break;
	case 3:				//Generates a command to clear STR
		genCom->_REGSEL = RSSTR;
		genCom->_COMSEL = CCLR;
		break;
	case 4:				//Generates a command to read MDR0
		genCom->_REGSEL = RSMDR0;
		genCom->_COMSEL = CRD;
		break;
	case 5:				//Generates a command to read MDR1
		genCom->_REGSEL = RSMDR1;
		genCom->_COMSEL = CRD;
		break;
	case 6:				//Generates a command to read CNTR
		genCom->_REGSEL = RSCNTR;
		genCom->_COMSEL = CRD;
		break;
	case 7:				//Generates a command to read OTR
		genCom->_REGSEL = RSOTR;
		genCom->_COMSEL = CRD;
		break;
	case 8:				//Generates a command to read STR
		genCom->_REGSEL = RSSTR;
		genCom->_COMSEL = CRD;
		break;
	case 9:				//Generates a command to write MDR0
		genCom->_REGSEL = RSMDR0;
		genCom->_COMSEL = CWR;
		break;
	case 10:			//Generates a command to write MDR1
		genCom->_REGSEL = RSMDR1;
		genCom->_COMSEL = CWR;
		break;
	case 11:			//Generates a command to write DTR
		genCom->_REGSEL = RSDTR;
		genCom->_COMSEL = CWR;
		break;
	case 12:			//Load DTR to CNTR in 'parallel'
		genCom->_REGSEL = RSCNTR;
		genCom->_COMSEL = CLOAD;
		break;
	case 13:			//Load CNTR to DTR in 'parallel'
		genCom->_REGSEL = RSOTR;
		genCom->_COMSEL = CLOAD;
		break;
	default:			//Generates an empty command (operates on no register)
		genCom->_REGSEL = RSNONE;
		genCom->_COMSEL = CRD;
		break;
	}
}

/********************** PUBLIC FUNCTIONS *************************/
/**@fn 		void Encoder_Initialize()
 *@brief	Initializes the LS7366R encoder chip for use with the EDUarm
 */
void Encoder_Initialize()
{
	ENCODER_COMMAND init[3];
	ENCODER_MDR1 mdr1;
	ENCODER_MDR0 mdr0;
	char packets[5];
	long tmdr0 = 0;
	long tmdr1 = 0;
	long tcnt  = 1232;

	Encoder_Generate_Command(WRITEMDR0, &init[0]);
	Encoder_Generate_Command(WRITEMDR1, &init[1]);
	Encoder_Generate_Command(CLEARCNT, &init[2]);
	Serial_Print_String("\r\nInitialization Commands Generated");
	mdr0._CLKDIV= 0;			//Sets the clock divider in mdr0
	mdr0._AI 	= 0;			//Sets the Index Syncornisity in mdr0
	mdr0._INDEX = 0b00;			//Sets the Intedx mode in mdr0
	mdr0._FRCM	= 0b00;			//Sets the free-run mode in MDR0
	mdr0._QCM	= 0b10;			//Sets the quadrature mode in MDR0
	mdr1._CMODE		= 0;		//Sets the count mode in MDR1
	mdr1._ENABLE	= 1;		//Enables the counter
	mdr1._FLAGS		= 0;		//Sets the flags in MDR1
	mdr1._RESER		= 0;		//filler for the reserved bit

	packets[0]=init[0]._COMMAND;
	packets[1]=mdr0._MDR0;
	packets[2]=init[1]._COMMAND;
	packets[3]=mdr1._MDR1;
	packets[4]=init[2]._COMMAND;
	//disable interrupts to ensure that there will be no conflict with DAC SPI that is called on interrupts
	cli();
	//send initialization packets
	Encoder_Initialize_SPI();
	SPI_Send_Packet(&packets[0],2,SSPIN_ENC);
	SPI_Send_Packet(&packets[2],2,SSPIN_ENC);
	SPI_Send_Packet(&packets[3],1,SSPIN_ENC);

	//debug
	Serial_Print_String("\r\nSet MDR0 To: ");
	Serial_Print_Int((int)mdr0._MDR0,2);
	Serial_Print_String("\r\nSet MDR1 To: ");
	Serial_Print_Int((int)mdr1._MDR1,2);
	Encoder_Read(READMDR0, &tmdr0);
	Encoder_Read(READMDR1, &tmdr1);
	Encoder_Read(READCNT, &tcnt);
	Serial_Print_String("\r\nMDR0:  ");
	Serial_Print_Int((int)tmdr0, 2);
	Serial_Print_String("\r\nMDR1:  ");
	Serial_Print_Int((int)tmdr1, 2);
	Serial_Print_String("\r\nCNTR:  ");
	Serial_Print_Int((int)tcnt, 2);
	//re-enable interrupts
	sei();


}

/**@fn 		char Encoder_Read(char command, long *result)
 *@brief	Reads from a register on the LS7366R encoder chip
 *@param	[in] command A valid encoder command. Use the symbolic constants in Encoder.h
 *@param	[in] *result pointer to a long value to store the result to
 *
 *This function reads from a register on the LS7366R encoder chip over SPI. It stores the result to the location pointed to by *result.
 *NOTE: Not all registers are longs (and depending on configuration, none of them might be), so it is up to the user to interpret the
 *returned value correctly. For READCNT commands, if the CNTR register is set to be less than 4 bytes, it will store the returned data
 *in the upper n bytes of the long, MSBF, where n is the number of bytes CNTR is set up to be. For all other commands, it stores the value
 *in the lower bytes.
 *NOTE: Interrupts are disabled while this operation communicates over the SPI bus. Care should be taken that critical interrupts do not
 *happen during the operation, as they will be ignored.
 */
char Encoder_Read(char command, long *result)
{
	ENCODER_COMMAND genCom;

	Encoder_Generate_Command(command, &genCom);
	Serial_Print_String("\r\nGenerated Command: ");
	Serial_Print_Int((int)genCom._COMMAND, 2);
	cli();
	Encoder_Initialize_SPI();
	//bring encoder SS pin low to enable communications
	Pin_Set('B', SSPIN_ENC, 0);
	switch(command)
	{
	case READMDR0:
	case READMDR1:
		SPI_Send_Byte(genCom._COMMAND);
		*result = SPI_Read_Byte();
		break;
	case READCNT:
		SPI_Send_Byte(genCom._COMMAND);
		*result = SPI_Read_Byte();
		*result = *result<<8;
		*result |= SPI_Read_Byte();
		*result = *result<<8;
		*result |= SPI_Read_Byte();
		*result = *result<<8;
		*result |= SPI_Read_Byte();
	}
	Pin_Set('B', SSPIN_ENC, 1);
	sei();
	return 1;
}
