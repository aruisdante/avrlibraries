/**
 *@file Final_Project.h
 *@brief Contains includes, data structures and function protos for the RBE3001 final project
 *@date Feb 25, 2010
 *@author Adam Panzica
 *@author Joel Sutherland
 */

#ifndef FINAL_PROJECT_H_
#define FINAL_PROJECT_H_

#include "ADC.h"
#include "DAC.h"
#include "IR_Sensor.h"
#include "PID.h"
#include "Simple_Serial.h"
#include "portIO.h"
#include "Timer16_Functions.h"
#include "Timer8_Functions.h"
#include "SPI.h"
#include <math.h>
#include <avr\IO.h>
#include <util\delay.h>
#include <avr\interrupt.h>

/**************** SYMBOLIC CONSTANTS *********************/
/** ADC channel for Joint 1*/
#define JOINT1 	0
/** ADC channel for Joint 2*/
#define JOINT2 	2
/** ADC channel for the X axis IR sensor*/
#define IR_X 	3
/** ADC channel for the Y axis IR sensor*/
#define IR_Y 	4
/** ADC channel for the rsense*/
#define RSENSE	1
/** Far cutoff for X direction object detection*/
#define X_MAX 	245
/** Close cutoff for X direction object detection*/
#define X_MIN 	120
/** Value representing an invalid X value (for no object state)*/
#define NO_OBJ 	999
/** Value representing the maximum Y distance */
#define Y_MAX 	240
/** Value representing the minimum Y distance */
#define Y_MIN 	140
/** Value reperesenting the tolerance on the Y target*/
#define Y_TOL 	10
/** Port letter for the electromagnet*/
#define MAGPORT 'B'
/** Pin number for the electromagnet*/
#define MAGPIN	1
/** Value to turn the electromagnet on*/
#define MAGON 	1
/** Value to turn the electromagnet off*/
#define MAGOFF 	0
/** Length of Link 1 in mm **/
#define L_1     70
/** Length of Link 2 in mm **/
#define L_2     70
/** Height of the blocks, in mm */
#define BLOCK_HEIGHT 37
/** Offset between the IR sensor and the base of the robot in the X direction*/
#define X_SHIFT 115
/** Offset between the conveyor belt and the base of the robot in the Y direction*/
#define Y_SHIFT -60
/** X location of the heavy bin */
#define HEAVY_X 270
/** Y location of the heavy bin */
#define HEAVY_Y 80
/** Weight cutoff */
#define HEAVY 13000

/**************** DATA STRUCTURES ***********************/
/**@struct JOINT
 * @brief this struct contains all of the data needed to track the state of a joint on the EDUarm
 */
typedef struct JOINT
{
	/** Link length (in mm) of the arm link attached to the joint*/
	int linkLength;
	/** Current joint angle, in degrees */
	int jointAngle;
	/** Current X location relative to arm base of the end of the link attached to the joint*/
	int xLocal;
	/** Current Y location relative to arm base of the end of the link attached to the joint*/
	int yLocal;

	/** ADC value of the motor current for the joint*/
	int rsense;
	/** Target value for the joint, in ADC values */
	int targetVal;
	/** Current value of the joint, in ADC values */
	int currentVal;
	/** Correction value for the joint, in arbitrary PID units */
	int correctionVal;
	/** LMD output voltage for the joint, in mV*/
	int LMDVal;
}JOINT;

/**@union STATE_FLAGS
 * @brief Union for holding flags about the current state of the system
 */
typedef union STATE_FLAGS
{
	struct
	{
		/** Flag for the magnet being enabled*/
		unsigned _MAGON : 1;
		/** Flag for weighing the object*/
		unsigned _WEIGH : 1;
		/** Flag for object detected*/
		unsigned _OBDET : 1;
		/** Flag for object in range*/
		unsigned _OBINR	: 1;
		/** Reserved bits*/
		unsigned _RES	: 4;
	};
	struct
	{
		unsigned char _FLAGS;
	};
}STATE_FLAGS;

/**@struct OBJECT_LOCATION
 * @brief data struction for holding the x and y location of objects on the conveyor
 */
typedef struct OBJECT_LOCATION
{
	/** X location, relative to object frame, in mm, of a detected object*/
	int xLocation;
	/** Y location, relative to object frame, in mm, of a detected object*/
	int yLocation;
}OBJECT_LOCATION;

/**@struct DATA_PACK
 * @brief data structure for passing data back and forth to various functions
 */
typedef struct DATA_PACK
{
	/** Pointer to a the first element in an array of joints that the MATLAB program is supposed to operate on. Must be in order staring
	 * at the joint closest to the base.
	 */
	JOINT *joint;

	/** Pointer to the state flags */
	STATE_FLAGS *flags;

	/** Pointer to the object Location */
	OBJECT_LOCATION *location;

}DATA_PACK;

/**@fn char Check_For_Object(OBJECT_LOCATION *location)
 * @brief this function checks to see if there is an object on the conveyer belt
 * @returns 1 if object detected, else 0
 */
char Check_For_Object(OBJECT_LOCATION *location);

/**@fn char Check_In_Range(OBJECT_LOCATION *location);
 * @brief this function checks to see if the object has reached the pickup area
 * @returns 1 if object in range, else 0
 */
char Check_In_Range(OBJECT_LOCATION *location);


/**void Enable_Magnet()
 * @brief enables the electro-magnet
 */
void Enable_Magnet();

/**void Disable_Magnet()
 * @brief enables the electro-magnet
 */
void Disable_Magnet();

/** char Calc_Inv_Kin(DATA_PACK *data)
 * @breif calculates the inverse kinematics for the robot arm target locations
 * @param [in] *data pointer to a DATA_PACK struct containing the robot information
 * @param [in] block 1 for hard coded block offsets, 0 for arbitrary location passed through the x/y location in the DATA_PACK structure
 * @return 1 if the calculation was a success, else 0
 * @note also updates the target values for the two joints to the correct values based on the caluclation
 */
char Calc_Inv_Kin(DATA_PACK *data, char block);

/** float Degrees_To_Radians(int degrees)
 * @brief converts degrees to radians
 * @param int degrees the angle in degrees
 * @return the equivalent angle in radians
 */
float Degrees_To_Radians(int degrees);

/** int Radians_To_Degrees(float radians)
 * @brief converts radians to degrees
 * @param radians the angle in radians
 * @return the equivalent angle in degrees
 */
int Radians_To_Degrees(float radians);

/**@fn int main()
 * The main function for the final project will implement a simple control system where it runs a PID control on target data recived from Matlab. It will update
 * Matlab with the current state of the system at a set frequency, and receive new target data at another set frequency. It will also check to see if there are
 * objects on the conveyer and if the are in range of the grabber, which is sent to matlab as well. To summarize, order of operations is as follows:
 * 1) Receive new target joint angle data from Matlab
 * 2) Check to see if there is an object on the conveyer
 * 3) if there was, check to see if it is in grabber range
 * 4) If Matlab told it to enable/disable the magnet at the end of the current move, do so if at target values
 * 5) Update Matlab with the current state of the robot
 * 6) Repeat from step 1
 * Background) PID control on the target joint angles
 */



#endif /* FINAL_PROJECT_H_ */
