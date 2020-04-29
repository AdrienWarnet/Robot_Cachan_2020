#include <xc.h>
#include "IO.h"
#include "PWM.h"
#include "Robot.h"
#include "Toolbox.h"
#include "UART_Protocol.h"
#include "QEI.h"
#include "Utilities.h"

#define PWMRER 40.0
unsigned char acceleration = 4;
unsigned char messagePWM[2];

void InitPWM(void) {
    PTCON2bits.PCLKDIV = 0b000; // Divide by 1
    PTPER = 100 * PWMRER; //éPriode  en  pourcentage

    //réglage PWM moteur 1 sur hacheur 1
    IOCON1bits.POLH = 1; //High = 1 and  active  on low =0
    IOCON1bits.POLL = 1; //High = 1
    IOCON1bits.PMOD = 0b01; // Set PWM Mode to  Redundant
    FCLCON1 = 0x0003; //éDsactive  la  gestion  des  f a u l t s

    // Reglage PWM moteur 2  sur  hacheur 6
    IOCON6bits.POLH = 1; //High = 1*
    IOCON6bits.POLL = 1; //High = 1
    IOCON6bits.PMOD = 0b01; // Set PWM Mode to  Redundant
    FCLCON6 = 0x0003; //éDsactive  la  gestion  des  f a u l t s

    /*enable PWM module*/
    PTCONbits.PTEN = 1;

}

//void PWMSetSpeed(float vitesseEnPourcents, char motor) {
//    if (motor == MOTEUR_GAUCHE) {
//        robotState.vitesseGaucheCommandeCourante = vitesseEnPourcents;
//        if (vitesseEnPourcents >= 0) {
//            MOTEUR_GAUCHE_ENL = 0; // Pilotage  de  la  pin  en mode IO
//            MOTEUR_GAUCHE_INL = 1; //Mise à 1 de  la  pin
//            MOTEUR_GAUCHE_ENH = 1; // Pilotage  de  la  pin  en mode PWM
//        } else
//            if (vitesseEnPourcents < 0) { //pour changer le sens du moteur on change les H en L ou inversement
//            MOTEUR_GAUCHE_ENH = 0; // Pilotage  de  la  pin  en mode IO
//            MOTEUR_GAUCHE_INH = 1; //Mise à 1 de  la  pin
//            MOTEUR_GAUCHE_ENL = 1; // Pilotage  de  la  pin  en mode PWM
//        }
//        MOTEUR_GAUCHE_DUTY_CYCLE = Abs(robotState.vitesseGaucheCommandeCourante * PWMRER);
//    } else
//        if (motor == MOTEUR_DROIT) {
//        robotState.vitesseDroiteCommandeCourante = vitesseEnPourcents;
//        if (vitesseEnPourcents >= 0) {
//            MOTEUR_DROIT_ENL = 0; // Pilotage  de  la  pin  en mode IO
//            MOTEUR_DROIT_INL = 1; //Mise à 1 de  la  pin
//            MOTEUR_DROIT_ENH = 1; // Pilotage  de  la  pin  en mode PWM
//        } else
//            if (vitesseEnPourcents < 0) { //pour changer le sens du moteur on change les H en L ou inversement
//            MOTEUR_DROIT_ENH = 0; // Pilotage  de  la  pin  en mode IO
//            MOTEUR_DROIT_INH = 1; //Mise à 1 de  la  pin
//            MOTEUR_DROIT_ENL = 1; // Pilotage  de  la  pin  en mode PWM
//        }
//        MOTEUR_DROIT_DUTY_CYCLE = Abs(robotState.vitesseDroiteCommandeCourante * PWMRER);
//    }
//}

void PWMUpdateSpeed() {
    // Cette fonction est appelée sur timer et permet de suivre des rampes d?accélération
    if (robotState.vitesseDroiteCommandeCourante < robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Min(robotState.vitesseDroiteCommandeCourante + acceleration,robotState.vitesseDroiteConsigne);
    if (robotState.vitesseDroiteCommandeCourante > robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Max(robotState.vitesseDroiteCommandeCourante - acceleration,robotState.vitesseDroiteConsigne);

    if (robotState.vitesseDroiteCommandeCourante > 0) {
        MOTEUR_DROIT_ENH = 0; //pilotage de la pin en mode IO
        MOTEUR_DROIT_INH = 1; //Mise à 1 de la pin
        MOTEUR_DROIT_ENL = 1; //Pilotage de la pin en mode PWM
    } else {
        MOTEUR_DROIT_ENL = 0; //pilotage de la pin en mode IO
        MOTEUR_DROIT_INL = 1; //Mise à 1 de la pin
        MOTEUR_DROIT_ENH = 1; //Pilotage de la pin en mode PWM
    }
    MOTEUR_DROIT_DUTY_CYCLE = Abs(robotState.vitesseDroiteCommandeCourante) * PWMRER;

    if (robotState.vitesseGaucheCommandeCourante < robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Min(robotState.vitesseGaucheCommandeCourante + acceleration,robotState.vitesseGaucheConsigne);
    if (robotState.vitesseGaucheCommandeCourante > robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Max(robotState.vitesseGaucheCommandeCourante - acceleration,robotState.vitesseGaucheConsigne);

    if (robotState.vitesseGaucheCommandeCourante > 0) {
        MOTEUR_GAUCHE_ENL = 0; //pilotage de la pin en mode IO
        MOTEUR_GAUCHE_INL = 1; //Mise à 1 de la pin
        MOTEUR_GAUCHE_ENH = 1; //Pilotage de la pin en mode PWM
    } else {
        MOTEUR_GAUCHE_ENH = 0; //pilotage de la pin en mode IO
        MOTEUR_GAUCHE_INH = 1; //Mise à 1 de la pin
        MOTEUR_GAUCHE_ENL = 1; //Pilotage de la pin en mode PWM
    }
    MOTEUR_GAUCHE_DUTY_CYCLE = Abs(robotState.vitesseGaucheCommandeCourante) * PWMRER;
}

void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur) {
    if (moteur == MOTEUR_DROIT) {
        robotState.vitesseDroiteConsigne = vitesseEnPourcents;
    }
    else
        if (moteur == MOTEUR_GAUCHE) {
        robotState.vitesseGaucheConsigne = vitesseEnPourcents;

    }
}


void sendMessageVitesseReelle(float vitesseGauche, float vitesseDroite){
        
    messagePWM[0] = (unsigned char) vitesseGauche ;
    messagePWM [1] = (unsigned char) vitesseDroite;
    UartEncodeAndSendMessage(0x0040,2,messagePWM);
}

//void PWMSetSpeedConsignePolaire(){
//    double kpAng = 0;
//    double kpLin = 0; 
//    //Correction Angulaire
//    double erreurVitesseAngulaire = robotState.vitesseAngulaireConsigne - robotState.vitesseAngulaireFromOdometry;
//    double sortieCorrecteurAngulaire=kpAng * erreurVitesseAngulaire;
//    double correctionVitesseAngulaire =(robotState.vitesseAngulaireConsigne + sortieCorrecteurAngulaire) * (DISTROUES/2);
//    double correctionVitesseAngulairePourcent = correctionVitesseAngulaire * COEFF_VITESSE_ANGULAIRE_PERCENT;
//    
//    //Correction Linéaire
//    double erreurVitesseLineaire = robotState.vitesseLineaireConsigne - robotState.vitesseLineaireFromOdometry;
//    double sortieCorrecteurLineaire= kpLin * erreurVitesseLineaire;
//    double correctionVitesseLinaire = (robotState.vitesseLineaireConsigne + sortieCorrecteurLineaire) * (DISTROUES/2);
//    double correctionVitesseLinairePourcent = correctionVitesseLinaire * COEFF_VITESSE_LINEAIRE_PERCENT;
//    
//    // Génération des consigne droite et gauche
//    robotState.vitesseDroiteConsigne = correctionVitesseLinairePourcent + correctionVitesseAngulairePourcent ;
//    robotState.vitesseDroiteConsigne = LimitToInterval(robotState.vitesseDroiteConsigne, -100,100);
//    robotState.vitesseGaucheConsigne = correctionVitesseLinairePourcent - correctionVitesseAngulairePourcent ;
//    robotState.vitesseGaucheConsigne = LimitToInterval(robotState.vitesseGaucheConsigne, -100,100);
//    
//}


double integraleErreurVitesseAngulaire=0;
double KpAngulaire = 1.8; //6.0
double KiAngulaire = 0.35*FREQ_ECH_QEI; //0.29*FREQ_ECH_QEI car on divise l'erreur par le freq d'echantillonage
double effetMaxIntegraleVitesseAngulaire = 2 * PI;
double integraleErreurVitesseAngulaireMax;

double integraleErreurVitesseLineaire=0;
double KpLineaire = 0; //2.7;
double KiLineaire = 0.0 * FREQ_ECH_QEI; //0.15;
double effetMaxIntegraleVitesseLineaire = 5.0;
double integraleErreurVitesseLineaireMax;

void InitAsservissement()
{  
    //A commenter pour bien comprendre l'effet max du correcteur PI
    if(KiAngulaire!=0)
        integraleErreurVitesseAngulaireMax = effetMaxIntegraleVitesseAngulaire*KpAngulaire/KiAngulaire;
    else
        integraleErreurVitesseAngulaireMax = 0;
    
    if(KiLineaire!=0)
        integraleErreurVitesseLineaireMax = effetMaxIntegraleVitesseLineaire*KpLineaire/KiLineaire;
    else
        integraleErreurVitesseLineaireMax = 0;
}

void PWMSetSpeedConsignePolaire()
{    
    robotState.vitesseAngulaireConsigne = 0.0;   //Nécessaire pour régler la correction angulaire, TODO : enlever cette ligne à la fin
    robotState.vitesseLineaireConsigne = 0.0;    //Nécessaire pour régler la correction linéaire, TODO : enlever cette ligne à la fin
    
    //Correction angulaire    
    double erreurVitesseAngulaire = robotState.vitesseAngulaireConsigne - robotState.vitesseAngulaireFromOdometry;
    //On calcule l'intégrale
    integraleErreurVitesseAngulaire += erreurVitesseAngulaire/FREQ_ECH_QEI;
    //On borne l'intégrale
    integraleErreurVitesseAngulaire = LimitToInterval(integraleErreurVitesseAngulaire, -integraleErreurVitesseAngulaireMax, integraleErreurVitesseAngulaireMax);
    
    //On calcul les corrections
    double correctionVitesseAngulaireKp = KpAngulaire*erreurVitesseAngulaire;
    double correctionVitesseAngulaireKi = KiAngulaire * integraleErreurVitesseAngulaire;    
    double correctionVitesseAngulaire = (correctionVitesseAngulaireKp + correctionVitesseAngulaireKi) ;
    
//    //TODO : Code à commenter (3 lignes), uniquement pour le calage initial
//    robotState.vitesseAngulaireConsigne = 5;
//    correctionVitesseAngulaire = robotState.vitesseAngulaireConsigne;
            
    double correctionVitesseAngulairePourcent = correctionVitesseAngulaire * COEFF_VITESSE_ANGULAIRE_PERCENT;
    
//    
    //Correction Lineaire    
    double erreurVitesseLineaire = robotState.vitesseLineaireConsigne - robotState.vitesseLineaireFromOdometry;
    //On calcule l'intégrale
    integraleErreurVitesseLineaire += erreurVitesseLineaire/FREQ_ECH_QEI;
    //On borne l'intégrale
    integraleErreurVitesseLineaire = LimitToInterval(integraleErreurVitesseLineaire, -integraleErreurVitesseLineaireMax, integraleErreurVitesseLineaireMax);
    
    //On calcul les corrections
    double correctionVitesseLineaireKp = KpLineaire*erreurVitesseLineaire;
    double correctionVitesseLineaireKi = KiLineaire * integraleErreurVitesseLineaire;    
    double correctionVitesseLineaire = (correctionVitesseLineaireKp + correctionVitesseLineaireKi) ;
    
//    //TODO : Code à commenter (3 lignes), uniquement pour le calage initial
//    robotState.vitesseLineaireConsigne = 1.0;
//    correctionVitesseLineaire = robotState.vitesseLineaireConsigne;
            
    double correctionVitesseLineairePourcent = correctionVitesseLineaire * COEFF_VITESSE_LINEAIRE_PERCENT;

   
    
    //Génération des consignes droite et gauche
    robotState.vitesseDroiteConsigne = correctionVitesseLineairePourcent + correctionVitesseAngulairePourcent;
    robotState.vitesseDroiteConsigne = LimitToInterval(robotState.vitesseDroiteConsigne, -100, 100);
    robotState.vitesseGaucheConsigne = correctionVitesseLineairePourcent - correctionVitesseAngulairePourcent;
    robotState.vitesseGaucheConsigne = LimitToInterval(robotState.vitesseGaucheConsigne, -100, 100);
}



void sendMessageConsignePolaire(){
    unsigned char consignePolairePayload[8];
    getBytesFromFloat(consignePolairePayload, 0, (float)(robotState.vitesseAngulaireConsigne));
    getBytesFromFloat(consignePolairePayload, 4, (float)(robotState.vitesseLineaireConsigne));
    UartEncodeAndSendMessage(COMMAND_POLAIRE, 8, consignePolairePayload);
    
}