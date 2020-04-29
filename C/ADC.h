/* 
 * File:   ADC.h
 * Author: table4
 *
 * Created on 9 septembre 2019, 10:29
 */

#ifndef ADC_H
#define	ADC_H


void InitADC1(void);
void ADC1StartConversionSequence(void);
void ADCClearConversionFinishedFlag(void);
unsigned char ADCIsConversionFinished(void);
unsigned int * ADCGetResult(void);

#endif	/* ADC_H */

