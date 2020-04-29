 #include<xc.h>
#include"timer.h"
#include"IO.h"
#include "PWM.h"
#include "ADC.h"
#include "main.h"
#include "Robot.h"
#include "UART_Protocol.h"
#include "QEI.h"

 unsigned long timestamp ;
 unsigned long flagSendLedBleue ; 
//         flagSendLedBlanche, flagSendLedOrange ;
 unsigned long flagSendMoteur ;
 unsigned long flagSendPosition;
 
 char i=0;
 char j = 0;
//init d'un timer de 32 bits

void InitTimer23 (void)  {
    T3CONbits .TON = 0;  // Stop any 16?bit  Timer3  operation
    T2CONbits .TON = 0;  // Stop any 16/32?bit  Timer3  operation
    T2CONbits . T32 = 1;  //  Enable 32?bit  Timer mode
    T2CONbits .TCS = 0;  //  Select  i n t e r n a l  i n s t r u c t i o n  cycle  clock
    T2CONbits .TGATE = 0;  //  Disable  Gated Timer mode
    T2CONbits .TCKPS = 0b01 ;  //  Select  1:1  Prescaler
    TMR3 = 0x00 ;  //  Clear 32?bit  Timer  (msw)
    TMR2 = 0x00 ;  //  Clear 32?bit  Timer  ( lsw )
    PR3 = 0x98 ;  // Load 32?bit  period  value  (msw)
    PR2 = 0x5A00 ;  // Load 32?bit  period  value  ( lsw )
    IPC2bits . T3IP = 0x01 ;  //  Set  Timer3  Interrupt  P r i o r i t y  Leve
    IFS0bits . T3IF = 0;  //  Clear  Timer3  Interrupt  Flag
    IEC0bits . T3IE = 1;  //  Enable Timer3  interrupt
    T2CONbits .TON = 1;  //  Start 32?bit  Timer
}


float consignePWM=0;
// interup du timer 32 bits sur 2-3
void __attribute__ (( interrupt ,  no_auto_psv ))  _T3Interrupt (void)  {
    IFS0bits . T3IF = 0;  //  Clear  Timer3  Interrupt  Flag
}

//initialisation d'un timer 16 bits
void InitTimer1 (void) {

        //Timer1 pour  horodater  l e s  mesures  (1ms)
        SetFreqTimer1 (250);
        T1CONbits.TON = 0;  //  Disable  Timer
        T1CONbits.TSIDL = 0  ;  // continue  in  i d l e  mode
        T1CONbits.TGATE = 0  ;  // Accumulation  disabled
    //    T1CONbits.TCKPS = 0b11 ;  // Prescaler
        //11 = 1:256  prescale  value
        //10 = 1:64  prescale  value
        //01 = 1:8  prescale  value
        //00 = 1:1  prescale  value
        T1CONbits.TCS = 0;  // clock  source = i n t e r n a l  clock
    //    PR1 = 1563;

        IFS0bits.T1IF = 0;  //  Clear  Timer  Interrupt  Flag
        IEC0bits.T1IE = 1;  //  Enable Timer  interrupt
        T1CONbits.TON = 1;  //  Enable Timer
    
}

//interupt timer 1

void __attribute__ (( interrupt ,  no_auto_psv ))  _T1Interrupt (void) 
{    
    IFS0bits.T1IF = 0;
    //ADC1StartConversionSequence();
    QEIUpdateData(); 
    PWMUpdateSpeed();
    PWMSetSpeedConsignePolaire();
   
    flagSendMoteur++;     
    if(j++ >= 4)
    { // 200/4 = 50Hz
        SendPositionData(); //position + vitesse polaire reelle
        sendMessageConsignePolaire(); //Retour des consignes en polaire
        sendMessageVitesseReelle(robotState.vitesseGaucheConsigne,robotState.vitesseDroiteConsigne); //envoie vitesse reelle des moteur gauche et droit
        j=0;
    }    
}


void SetFreqTimer1(float freq)
{
    T1CONbits.TCKPS = 0b00; //00 = 1:1 prescaler value
    if(FCY /freq > 65535)
    {
    T1CONbits.TCKPS = 0b01; //01 = 1:8 prescaler value
    if(FCY /freq / 8 > 65535)
    {
    T1CONbits.TCKPS = 0b10; //10 = 1:64 prescaler value
    if(FCY /freq / 64 > 65535)
    {
    T1CONbits.TCKPS = 0b11; //11 = 1:256 prescaler value
    PR1 = (int)(FCY / freq / 256);
    }
    else
    PR1 = (int)(FCY / freq / 64);
    }
    else
    PR1 = (int)(FCY / freq / 8);
    }
    else
    PR1 = (int)(FCY / freq);
}


void InitTimer4 (void) {
    //Timer1 pour  horodater  l e s  mesures  (1ms)
    SetFreqTimer4 (1000);
    T4CONbits.TON = 0;  //  Disable  Timer
    T4CONbits.TSIDL = 0  ;  // continue  in  i d l e  mode
    T4CONbits.TGATE = 0  ;  // Accumulation  disabled
//    T1CONbits.TCKPS = 0b11 ;  // Prescaler
    //11 = 1:256  prescale  value
    //10 = 1:64  prescale  value
    //01 = 1:8  prescale  value
    //00 = 1:1  prescale  value
    T4CONbits.TCS = 0;  // clock  source = i n t e r n a l  clock
//    PR1 = 1563;
    
    IFS1bits.T4IF = 0;  //  Clear  Timer  Interrupt  Flag
    IEC1bits.T4IE = 1;  //  Enable Timer  interrupt
    T4CONbits.TON = 1;  //  Enable Timer
}

void SetFreqTimer4(float freq) {
    T4CONbits.TCKPS = 0b00; //00 = 1:1 prescaler value
    if (FCY / freq > 65535) {
        T4CONbits.TCKPS = 0b01; //01 = 1:8 prescaler value
        if (FCY / freq / 8 > 65535) {
            T4CONbits.TCKPS = 0b10; //10 = 1:64 prescaler value
            if (FCY / freq / 64 > 65535) {
                T4CONbits.TCKPS = 0b11; //11 = 1:256 prescaler value
                PR4 = (int) (FCY / freq / 256);
            } else
                PR4 = (int) (FCY / freq / 64);
        } else
            PR4 = (int) (FCY / freq / 8);
    } else
        PR4 = (int) (FCY / freq);
}

void __attribute__ (( interrupt ,  no_auto_psv ))  _T4Interrupt (void) 
{
    timestamp++;
    IFS1bits.T4IF = 0; 
}