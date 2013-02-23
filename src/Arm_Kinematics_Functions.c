/**
 *@file Arm_Kinematics_Functions.c
 *
 *@date Feb 7, 2010
 *@author Adam Panzica
 *
 */
#include <math.h>
#include <avr/io.h>
#include "Simple_Serial.h"

/**@fn char AKF_2link_Forward(int *jointAngle0, int *jointAngle1, int *linkLength0, int *linkLength1, int *xcoord, int *ycoord)
 * @brief Calculates the forward kinematics of a 2 DOF arm
 * @param [in] *jointAngle0 Pointer to the Angle between link 0 and horizontal
 * @param [in] *jointAngle1 Pointer to the Angle between link 1 and perpendicular to link 0
 * @param [in] *linkLength0 Pointer to the Length of link 0
 * @param [in] *linkLength1 Pointer to the Length of link 1
 * @param [out] *xcoord Pointer to a location to write the X coordinate to
 * @param [out] *ycoord Pointer to a location to write the Y coordinate to
 */
char AKF_2link_Forward(int *jointAngle0, int *jointAngle1, int *linkLength0, int *linkLength1, int *xcoord, int *ycoord)
{
	double angle0;
	double angle1;
	double tempX;
	double tempY;


	//convert joint angles into radians
	angle0 = (float)*jointAngle0*M_PI/180;
	angle1 = (float)(*jointAngle1+*jointAngle0-90)*M_PI/180;

	tempX = (float)*linkLength0*cos(angle0)+(float)*linkLength1*cos(angle1);
	tempY = (float)*linkLength0*sin(angle0)+(float)*linkLength1*sin(angle1);

	*xcoord = (int)tempX;
	*ycoord = (int)tempY;

	return 1;
}
