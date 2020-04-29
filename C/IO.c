/*
 * File:   IO.c
 */

#include <xc.h>
#include "IO.h"
//#include "main.h"

void InitIO()
{
    // IMPORTANT : désactiver les entrées analogiques, sinon on perd les entrées numériques
    ANSELA = 0; // 0 desactive
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELF = 0;
    ANSELG = 0;

    //********** Configuration des sorties ********************************
    // LED
    _TRISC10 = 0;  // LED Orange
    _TRISG6 = 0; //LED Blanche
    _TRISG7 = 0; // LED Bleue
    
    // Moteurs Gauche
    _TRISB15 = 0;   //IN2
    _TRISB14 = 0;   //IN1
    
    //Moteur Droit   
    _TRISC7 = 0;   //IN2
    _TRISC6 = 0;   //IN1
    
    

    //********** Configuration des entrées ********************************   
    
    //Jack
    _TRISE12 = 1;
    
    
    //********** UART *****************************************************
    
    _U1RXR = 24 ;
    _RP36R = 0b00001 ;
    
    //**************QEI****************************************************
    
    _QEA2R = 97; //assigne le QEI A à la pin remappable RP97
    _QEB2R = 96; //assigne le QEI B à la pin remapppable RP96
    
    
    _QEA1R = 70; //assigne le QEI A à la pin remappable RP70
    _QEB1R = 69; //assigne le QEI B à la pin remapppable RP69
    
    
    
    
    
    
    
}

