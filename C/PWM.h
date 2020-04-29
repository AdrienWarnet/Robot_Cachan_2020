/* 
 * File:   PWM.h
 * Author: table4
 *
 * Created on 4 septembre 2019, 15:50
 */

#ifndef PWM_H
#define	PWM_H

#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1

#define COEFF_VITESSE_LINEAIRE_PERCENT 35.0//31.0
#define COEFF_VITESSE_ANGULAIRE_PERCENT 4.6  //4.0


//void PWMSetSpeed(float, char);
void InitPWM(void);
void PWMUpdateSpeed(void);
void PWMSetSpeedConsigne(float , char);
void sendMessageVitesseReelle (float,float);
void PWMSetSpeedConsignePolaire();
void sendMessageConsignePolaire();

void InitAsservissement();
#endif	/* PWM_H */

