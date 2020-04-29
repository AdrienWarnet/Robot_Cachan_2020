#ifndef TIMER_H
#define	TIMER_H

extern unsigned long timestamp ;
extern unsigned long flagSendLedBleue, flagSendLedBlanche, flagSendLedOrange ;
extern unsigned long flagSendMoteur ;
extern unsigned long flagSendPosition; ;

void InitTimer23(void);
void InitTimer1(void);
void SetFreqTimer1(float);
void InitTimer4 (void) ;
void SetFreqTimer4(float);

#endif /*TIMER_H*/
