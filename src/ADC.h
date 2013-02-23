/**
 * @file ADC.h
 *
 *@date Jan 22, 2010
 *@author Adam Panzica
 */

#ifndef ADC_H_
#define ADC_H_
/**
 *@struct ADC_Sensor ADC.h "ADC.h"
 *@brief this strcut defines an ADC Sensor object
 */
struct ADC_Sensor
{

	char name[15];	/**< name array of chars for storing the sensor name*/
	unsigned char channel; /**< channel number the ADC sensor is on*/
	int valueAtMax;	/**< valueAtMax value of the ADC sensor in engineering units at Vin=VRef*/
	int valueAtMin;	/**< valueAtMin value of the ADC sensor in engineering units at Vin=Ground*/
	int scaleFactor;	/**< scaleFactor scale factor for converting ADC units to engineering units for the sensor*/
};

void ADC_Init_P();
int ADC_Get_Value(unsigned char channel);
int ADC_Calc_Volts(int ADCValue, int VRef);
struct ADC_Sensor ADC_Sensor_Construct ( char *name, unsigned char channel, int valueAtMax, int valueAtMin);
int ADC_Sensor_Calc_Value (struct ADC_Sensor *sensor, int ADCValue);

int ADC_MCP3204_Get_Value(unsigned char channel, unsigned char justify);
int ADC_MCP3204_Count_to_Gs(int ADCVal, int VRef);


#endif /* ADC_H_ */
