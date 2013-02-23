/**
 * @file Timer16_Functions.h
 *
 *@date Jan 26, 2010
 *@author Adam Panzica
 *@author Joel Sutherland
 */

#ifndef SQUARE_WAVE_H_
#define SQUARE_WAVE_H_

void Square_Wave16_Initialize(unsigned int frequency, unsigned int fclk);
void Square_Wave16_Set_Duty_Cycle(char dutyCycle);
void Square_Wave16_Set_Freq(unsigned int frequency, unsigned int fclk);
void Timer16_Initialize(unsigned int frequency, unsigned int fclk);
void Timer16_Set_Freq(unsigned int frequency, unsigned int fclk);
unsigned char ADC_To_Duty(int ADCVal);

/****************** SYMBOLIC CONSTANTS *******************************/
/**@def FPWM
 * @brief selection for fast PWM mode
 */
#define FPWM 0
/**@def CTC
 * @brief selection for CTC mode
 */
#define CTC 0

#endif /* SQUARE_WAVE_H_ */
