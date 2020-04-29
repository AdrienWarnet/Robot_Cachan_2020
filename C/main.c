#include<xc.h>
#include"ChipConfig.h"
#include"IO.h"
#include <stdio.h>
#include <stdlib.h>
#include "timer.h"
#include "PWM.h"
#include "Robot.h"
#include "Toolbox.h"
#include "ADC.h"
#include "UART.h"
#include "main.h"
#include "CB_TX1.h"
#include <libpic30.h>
#include "CB_RX1.h"
#include "UART_Protocol.h"
#include "QEI.h"

unsigned char autoControlState = 1;
unsigned char jackValue = 0;

int main(void) {

    //init de l'oscillateur
    InitOscillator();

    //config des Entré sorties
    InitIO();

    //config des timers
    InitTimer23();
    InitTimer1();
    InitTimer4();

    //init moteur 
    InitPWM();
    InitAsservissement();
    //init ADC
    InitADC1();
    
    //init Uart
    InitUART();
    
    //init QEI

    InitQEI1();
    InitQEI2();

    //boucle principal
   // unsigned int * result;
    
   
 
   
   
    while (1) {
        
        int i;
        for (i = 0; i < CB_RX1_GetDataSize(); i++)
        {
            unsigned char c = CB_RX1_Get();
            UartDecodeMessage(c);
        }
        
        ////////////////////////////Envoi de l'état des leds sur le port serie//////////////
//        if (flagSendLedBleue == 870){
//            SendLedState(LED_BLEUE_Number);
//            flagSendLedBleue = 0;
//        }
//         
//        if (flagSendMoteur == 50){
//            sendMessagePWM(robotState.vitesseGaucheConsigne,robotState.vitesseDroiteConsigne);
//            flagSendMoteur = 0;
//        }
//        
//        if (flagSendPosition == 1){
//            SendPositionData();
//            flagSendPosition = 0;
//        }
    }   
}//fin main  


unsigned char stateRobot, droite_Ou_gauche = 0;

void OperatingSystemLoop(void) { 
  if(JACK) {
//    robotState.vitesseAngulaireConsigne = PI/4;
//    robotState.vitesseLineaireConsigne = 0;
      if(jackValue == 0)
      {
          jackValue = 1;
          unsigned char jackMessage[4] = {jackValue, 5, 10, 20};
          UartEncodeAndSendMessage(COMMAND_ID_JACK, 4, jackMessage);
      }
      //LED_ORANGE = 1;
      //LED_BLANCHE = 1;
      //LED_BLEUE = 1 ;
      
   
      
    if (autoControlState) {
    
        
        }
        else
        {
            PWMSetSpeedConsigne(robotState.vitesseGaucheConsigne,MOTEUR_GAUCHE);
            PWMSetSpeedConsigne(robotState.vitesseDroiteConsigne,MOTEUR_DROIT); 
        }
    } else {
        if(jackValue == 1)
        {
            jackValue = 0;
            
            //LED_ORANGE = 1;
            //LED_BLANCHE = 0;
            //LED_BLEUE = 0 ;
            unsigned char jackMessage[4] = {jackValue, 5, 10, 20};
            UartEncodeAndSendMessage(COMMAND_ID_JACK, 4, jackMessage);
        }
        timestamp = 0;
        stateRobot = STATE_ATTENTE;
        robotState.vitesseDroiteConsigne = 0;
        robotState.vitesseGaucheConsigne = 0;
       robotState.vitesseAngulaireConsigne = 0;
       robotState.vitesseLineaireConsigne = 0;
       robotState.xPosFromOdometry = 0;
       robotState.yPosFromOdometry = 0;
       robotState.angleRadianFromOdometry = 0;
       
    }
}

void SetRobotState (unsigned char c){
    stateRobot = c;
    OperatingSystemLoop();
}

void SetRobotAutoControlState (unsigned char c) {
    if (c == 1)
        autoControlState = 1;
    else if (c == 0 )
        autoControlState = 0;
}

void SendLedState (int numero_led){

    if(numero_led == LED_BLANCHE_Number){
        
        unsigned char ledMessage[2] = {LED_BLANCHE_Number ,LED_BLANCHE};
        UartEncodeAndSendMessage(COMMAND_ID_LED, 2, ledMessage);
        
    } else if(numero_led == LED_ORANGE_Number){ 
        
        unsigned char ledMessage2[2] = {LED_ORANGE_Number ,LED_ORANGE};
        UartEncodeAndSendMessage(COMMAND_ID_LED, 2, ledMessage2);
        
    } else if(numero_led == LED_BLEUE_Number){
        
        unsigned char ledMessage3[2] = {LED_BLEUE_Number ,LED_BLEUE};
        UartEncodeAndSendMessage(COMMAND_ID_LED, 2, ledMessage3);
    }
}


//void SendStateRobot(int stateEnCours){
//    unsigned char msgStateRobot[5];
//    msgStateRobot[0] = stateEnCours;
//    msgStateRobot[1] = timestamp ;
//    UartEncodeAndSendMessage(COMMAND_ID_STATE, 5, msgStateRobot);
//}











