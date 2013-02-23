/**
 *@file Final_Project.c
 *
 *@date Feb 28, 2010
 *@author Adam Panzica
 *@author Joel Sutherland
 */

#include "Final_Project.h"

/*************************** LOCAL FUNCTION PROTOS *************************/
void State_0(DATA_PACK *data);
void State_1(DATA_PACK *data);
void State_2(DATA_PACK *data);
void State_3(DATA_PACK *data);
void State_Machine(char *state, DATA_PACK *data);
char Within(int target, int current);

/*************************** PROGRAM CODE **********************************/

/**Program Entry Point*/
int main()
{
	JOINT joints[2];
	STATE_FLAGS flags;
	OBJECT_LOCATION objLoc;
	DATA_PACK dataPack;
	char state = 0;

	char input;

	//initialize pointers in mlMessage
	dataPack.flags = &flags;
	dataPack.joint = &joints[0];
	dataPack.location = &objLoc;

	//Set initial values for the state flags
	flags._FLAGS = 0;

	//Set initial values for the joints
	joints[0].linkLength = 200;
	joints[0].targetVal = 0;
	joints[0].currentVal = 0;
	joints[0].LMDVal = 0;
	joints[0].correctionVal = 0;

	joints[1].linkLength = 195;
	joints[1].targetVal = 0;
	joints[1].currentVal = 0;
	joints[1].LMDVal = 0;
	joints[1].correctionVal = 0;

	//Set direction registers for debugging use
	DDRD = 0xFF;
	DDRC = 0x00;

	//Initialize serial communications, the ADC, the DAC and PID control
	Init_Serial_P(BAUD576, FRM8, STOP1, NOPAR);
	ADC_Init_P();
	DAC_Initilize();
	PID_Initialize();

	Serial_Print_String("\r\nMag Pickup Test\r\n");

	/*while(1)
	{
		Serial_Print_String("\r\nIR_X:  ");
		Serial_Print_Int(ReadIR(IR_X), 10);
		Serial_Print_String("\r\nIR_Y:  ");
		Serial_Print_Int(ReadIR(IR_Y), 10);
		_delay_ms(200);
	}*/
	//Calibrate joint pots
	Add_Pot_Angle(542, 90, 175, 0, "Joint 1", JOINT1);
	Add_Pot_Angle(504, 90, 849, 0, "Joint 2", JOINT2);

	//Add PID channels to control the two joints
	PID_Add_Channel(&joints[0].targetVal, &joints[0].currentVal, &joints[0].correctionVal, &joints[0].LMDVal, 450, 50, 300, 250);
	PID_Add_Channel(&joints[1].targetVal, &joints[1].currentVal, &joints[1].correctionVal, &joints[1].LMDVal, 375, 10, 350, 150);


	while(1)
	{
		joints[0].currentVal = ADC_Get_Value(JOINT1);
		joints[1].currentVal = ADC_Get_Value(JOINT2);
		joints[0].targetVal = Degrees_To_Pot(30, JOINT1);
		joints[1].targetVal = Degrees_To_Pot(60, JOINT2);

		State_Machine(&state, &dataPack);


		/*Serial_Print_String("\r\nTarget: ");
		Serial_Print_Int(Pot_To_Degrees(joints[1].targetVal, JOINT2), 10);
		Serial_Print_String("\r\nCurrent: ");
		Serial_Print_Int(Pot_To_Degrees(joints[1].currentVal, JOINT2), 10);*/

		_delay_ms(100);
	}

}

/********************** RANGE DETERMINING FUNCTIONS ****************************/
/**@fn char Check_For_Object(OBJECT_LOCATION *location)
 * @brief checks to see if there is an object on the belt, and finds its X position
 * @param [in] *location pointer to an OBJECT_LOCATION struct to store object location data to
 * @returns 1 if object is detected, else 0
 * @note also writes X location data to the OBJECT_LOCATION struct
 */
char Check_For_Object(OBJECT_LOCATION *location)
{
	int rangeBuff = 0;
	int temp;
	char i;

	temp = ReadIR(IR_X);
	//check to see if there was an object detected
	if((temp <= X_MAX)&&(temp >= X_MIN))
	{
		//take the average of 16 measurements to determine X position
		for(i = 0; i<16; i++)rangeBuff+=ReadIR(IR_X);
		rangeBuff = rangeBuff/16;
		location->xLocation = rangeBuff;
		return 1;
	}
	return 0;
}

/**@fn char Check_In_Range(OBJECT_LOCATION *location)
 * @brief checks to see if the object is in grabber range, and finds its Y position
 * @param [in] *location pointer to an OBJECT_LOCATION struct to store object location data to
 * @returns 1 if object is in range, else 0
 * @note also writes Y location data to the OBJECT_LOCATION struct
 */
char Check_In_Range(OBJECT_LOCATION *location)
{
	int rangeBuff = 0;

	rangeBuff = ReadIR(IR_Y);
	location->yLocation = rangeBuff;
	if((rangeBuff <= (Y_MAX + Y_TOL)&&(rangeBuff >= (Y_MIN - Y_TOL))))
	{
		_delay_ms(100);
		return 1;
	}
	return 0;
}
/************************ MAGNET CONTROL FUNCTIONS **************************/
/**Enables the electromagnet*/
void Enable_Magnet()
{
	Pin_Set(MAGPORT, MAGPIN, MAGON);
}
/**Disables the electromagnet*/
void Disable_Magnet()
{
	Pin_Set(MAGPORT, MAGPIN, MAGOFF);
}
/************************ STATE MACHINE FUNCTIONS ***************************/
/**@fn void State_Machine(char *state, DATA_PACK *data)
 * @brief Implements a state transition table for the simple state machine controlling the robot arm
 * @param [in] *state Pointer to the state variable
 * @param [in] *data Pointer to a DATA_PACK struct containing the robot information
 */
void State_Machine(char *state, DATA_PACK *data)
{
	switch(*state)
	{
	case 0:
		State_0(data);
		switch(data->flags->_OBDET)
		{
		case 0:
			*state = 0;
			break;
		case 1:
			*state = 1;
			break;
		default:
			*state = 0;
			break;
		}
		break;
	case 1:
		State_1(data);
		switch(data->flags->_OBINR)
		{
		case 0:
			*state = 1;
			break;
		case 1:
			*state = 2;
			break;
		default:
			*state = 0;
			break;
		}
		break;
	case 2:
		State_2(data);
		*state = 3;
		break;
	case 3:
		Serial_Print_String("\r\nPICKING UP OBJECT");
		State_3(data);
		*state = 0;
		break;
	default:
		Serial_Print_String("\r\nSTATE ERROR!!!!");
		break;
	}
}

/**@fn void State_0(DATA_PACK *data)
 * @brief State 0. Initial startup state and state for after object has been binned
 * @param [in] *data pointer to a DATA_PACK struct containing the robot information
 */
void State_0(DATA_PACK *data)
{
	JOINT *jptr;

	//initialize jptr to allow addressing of the JOINT array in the DATA_PACK structure
	jptr = data->joint;

	//Move arm to the idle position
	jptr[0].targetVal = Degrees_To_Pot(30, JOINT1);
	jptr[1].targetVal = Degrees_To_Pot(60, JOINT2);

	//Clear object detection/state information
	data->flags->_FLAGS = 0;
	data->location->xLocation = NO_OBJ;
	data->location->yLocation = Y_MAX;

	//check for a new object
	data->flags->_OBDET = Check_For_Object(data->location);
}
/**@fn void State_1(DATA_PACK *data)
 * @brief State 1. Object detected, waiting for it to come into range
 * @param [in] *data pointer to a DATA_PACK struct containing the robot information
 */
void State_1(DATA_PACK *data)
{
	JOINT *jptr;

	//initialize jptr to allow addressing of the JOINT array in the DATA_PACK structure
	jptr=data->joint;

	//Print out object detection information
	Serial_Print_String("\r\nOBJECT DETECTED!\r\n Object X Location: ");
	Serial_Print_Int(data->location->xLocation, 10);

	//Move arm to the ready position
	jptr[0].targetVal = Degrees_To_Pot(30, JOINT1);
	jptr[1].targetVal = Degrees_To_Pot(40, JOINT2);

	//Check to see if the object is in range
	data->flags->_OBINR = Check_In_Range(data->location);
}

/**@fn void State_2(DATA_PACK *data)
 * @brief State 2. Object in range, move arm to target and activate magnet
 * @param [in] *data pointer to a DATA_PACK struct containing the robot information
 */
void State_2(DATA_PACK *data)
{
	JOINT *jptr;

	//initialize jptr to allow addressing of the JOINT array in the DATA_PACK structure
	jptr=data->joint;

	//turn on the electromagnet
	Enable_Magnet();

	//Move arm to final target location
	Calc_Inv_Kin(data, 1);
	Serial_Print_String("\r\nMAG ON, MOVING TO TARGET");
	Enable_Magnet();
	while(!(Within(jptr[0].currentVal, jptr[0].targetVal))||!(Within(jptr[1].currentVal, jptr[1].targetVal)))
	{
		jptr[0].currentVal = ADC_Get_Value(JOINT1);
		jptr[1].currentVal = ADC_Get_Value(JOINT2);
	}

	//idle waiting for the arm to reach the target
	Serial_Print_String("\r\nAT TARGET, PICKING UP OBJECT");
	_delay_ms(500);
}

/**@fn void State_3(DATA_PACK *data)
 * @brief State 2. Picked up object, Sort into bin
 * @param [in] *data pointer to a DATA_PACK struct containing the robot information
 */
void State_3(DATA_PACK *data)
{
	JOINT *jptr;
	char heavy = 1;
	long weightsum = 0;

	//initialize jptr to allow addressing of the JOINT array in the DATA_PACK structure
	jptr=data->joint;


	Serial_Print_String("\r\nSORTING");
	//Move arm to the weighing position. Do a running sum on rsense.
	//The heavy block will take longer to move, and thus will have a bigger total
	jptr[0].targetVal = Degrees_To_Pot(45, JOINT1);
	jptr[1].targetVal = Degrees_To_Pot(45, JOINT2);
	while(!(Within(jptr[0].currentVal, jptr[0].targetVal))||!(Within(jptr[1].currentVal, jptr[1].targetVal)))
	{
		jptr[0].currentVal = ADC_Get_Value(JOINT1);
		jptr[1].currentVal = ADC_Get_Value(JOINT2);
		weightsum += (long)ADC_Get_Value(RSENSE);
		Serial_Print_String("\r\nWeight Sum:  ");
		Serial_Print_Int(weightsum, 10);
		_delay_ms(50);
	}
	//Sort into correct bin
	if(weightsum >= HEAVY) heavy = 1;
	else heavy = 0;
	switch(heavy)
	{
	case 0:
		jptr[0].targetVal = Degrees_To_Pot(90, JOINT1);
		jptr[1].targetVal = Degrees_To_Pot(0, JOINT2);
		break;
	case 1:
		data->location->xLocation = jptr[0].linkLength+jptr[1].linkLength;
		data->location->yLocation = 0;
		Calc_Inv_Kin(data, 0);
		break;
	default:
		break;
	}
	//Calculate joint angles to get to the bin and move arm

	while(!(Within(jptr[0].currentVal, jptr[0].targetVal))||!(Within(jptr[1].currentVal, jptr[1].targetVal)))
	{
		jptr[0].currentVal = ADC_Get_Value(JOINT1);
		jptr[1].currentVal = ADC_Get_Value(JOINT2);
	}
	//drop the object
	Disable_Magnet();
	Serial_Print_String("\r\nOBJECT SORTED, RESETTING");
}

char Calc_Inv_Kin(DATA_PACK *data, char block)
{
    JOINT *jptr;
    OBJECT_LOCATION *lptr;
    int theta1, theta2;
    float beta, gamma, delta, targetx, targety;

    //Serial_Print_String("\r\nINVERSE KINEMATICS\r\n");

    jptr=data->joint;
    lptr=data->location;

    switch(block)
    {
    case 0:
    	targetx = (float)lptr->xLocation;
    	targety = (float)lptr->yLocation;
    	break;
    case 1:
        targetx = (float)(lptr->xLocation+X_SHIFT);
        targety = (float)(BLOCK_HEIGHT+Y_SHIFT);
        break;
    default:
    	return 0;
    }

    /*Serial_Print_String("\r\ntargetx: ");
    Serial_Print_Int((int)targetx, 10);
    Serial_Print_String("\r\ntargety: ");
    Serial_Print_Int((int)targety, 10);*/

    beta = atan(targety/targetx);
    gamma = acos((pow(targetx, (float)2) + pow(targety, (float)2) + pow((float)jptr[0].linkLength,(float)2) - pow((float)jptr[1].linkLength,(float)2))/((float)2*(float)jptr[0].linkLength*sqrt(pow(targetx, (float)2) + pow(targety, (float)2))));
    delta = acos((pow((float)jptr[0].linkLength,(float)2)+pow((float)jptr[1].linkLength,(float)2)-(pow(targetx,(float)2)+pow(targety,(float)2)))/((float)2*(float)jptr[0].linkLength*(float)jptr[1].linkLength));

    /*Serial_Print_String("\r\nBeta: ");
    Serial_Print_Int((int)(beta*100), 10);
    Serial_Print_String("\r\nGamma: ");
    Serial_Print_Int((int)(gamma*100), 10);*/

    theta1 = Radians_To_Degrees(beta + gamma);
    theta2 = Radians_To_Degrees(delta) - 90;

    /*Serial_Print_String("\r\nTheta1: ");
    Serial_Print_Int(theta1, 10);
    Serial_Print_String("\r\nTheta2: ");
    Serial_Print_Int(theta2, 10);*/

    if(!(theta1 >= 0 && theta1 <= 90 && theta2 >= 0 && theta2 <= 90)) return 0;

    //Move arm to the proper position
    jptr[0].targetVal = Degrees_To_Pot(theta1, JOINT1);
    jptr[1].targetVal = Degrees_To_Pot(theta2, JOINT2);
  return 1;
}

float Degrees_To_Radians(int degrees)
{
  return (float)degrees * (float)3.14159 / (float)180;
}

int Radians_To_Degrees(float radians)
{
  return (int)((float)radians*(float)180/(float)3.14159);
}

char Within(int target, int current)
{
	if((current <= (target+10))&&(current >= (target-10))) return 1;
	return 0;
}
