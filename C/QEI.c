#include<xc.h>
#include"ChipConfig.h"
#include"IO.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Robot.h"
#include "Toolbox.h"
#include "QEI.h"
#include "timer.h"
#include "UART.h"
#include "UART_Protocol.h"
#include "Utilities.h"


double QeiDroitPosition_T_1 = 0, QeiDroitPosition = 0 ;
double QeiGauchePosition_T_1 = 0, QeiGauchePosition = 0;

double delta_d, delta_g, dx, delta_theta;

void InitQEI1()
{
    QEI1IOCbits.SWPAB = 1; //QEAx and QEBx are swapped
    QEI1GECL = 0xFFFF ;
    QEI1GECH = 0xFFFF ;
    QEI1CONbits.QEIEN = 1; //Enable QEI Module
}


void InitQEI2()
{
    QEI2IOCbits.SWPAB = 1; //QEAx and QEBx are swapped
    QEI2GECL = 0xFFFF ;
    QEI2GECH = 0xFFFF ;
    QEI2CONbits.QEIEN = 1; //Enable QEI Module
}


void QEIUpdateData()
{
    //On sauvegarde les anciennes valeurs 
    
    QeiDroitPosition_T_1 = QeiDroitPosition ;
    QeiGauchePosition_T_1 = QeiGauchePosition ;
    
    //On réactualise les valeurs des positions
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ((long)POS1HLD << 16);
    
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long)POS2HLD << 16);
    
    //Convertion en mm (réglé pour la taille des roues codeuses)
    QeiDroitPosition = 0.01620*QEI1RawValue/1000;
    QeiGauchePosition = -0.01620*QEI2RawValue/1000;
    
    //Calcul des deltas de position
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1;
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1;
    //delta_theta = atan((delta_d - delta_g) / DISTROUES);
    delta_theta = (delta_d - delta_g) / DISTROUES; //voir de combien on a tourné depuis la dernière mesure
    dx = (delta_d + delta_g) / 2; //voir de combien on a avancé ou reculé depuis la derniere mesure
    
    //Calcul des vitesses
    //attention à remultiplier par la fréquence d'échantillonnage
    robotState.vitesseDroitFromOdometry = delta_d*FREQ_ECH_QEI;
    robotState.vitesseGaucheFromOdometry = delta_g*FREQ_ECH_QEI;
    robotState.vitesseLineaireFromOdometry = 
            (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry) / 2;
    robotState.vitesseAngulaireFromOdometry = delta_theta*FREQ_ECH_QEI;
    
    //Mise à jour du positionnement terrain à t - 1
    robotState.xPosFromOdometry_1 = robotState.xPosFromOdometry;
    robotState.yPosFromOdometry_1 = robotState.yPosFromOdometry;
    robotState.angleRadianFromOdometry_1 = robotState.angleRadianFromOdometry;
    
    //Calcul des positions dans le referentiel du terrain
    robotState.xPosFromOdometry +=  dx*cos(robotState.angleRadianFromOdometry_1);
    robotState.yPosFromOdometry += dx*sin(robotState.angleRadianFromOdometry_1); 
    robotState.angleRadianFromOdometry += delta_theta; 
    if(robotState.angleRadianFromOdometry > PI)
        robotState.angleRadianFromOdometry -= 2*PI;
    if(robotState.angleRadianFromOdometry < -PI)
        robotState.angleRadianFromOdometry += 2*PI;

}

void SendPositionData()
{
    unsigned char positionPayload[24];
    getBytesFromInt32(positionPayload, 0, timestamp);
    //getBytesFromFloat(positionPayload, 4, (float)timestamp);test
    getBytesFromFloat(positionPayload, 4, (float)(robotState.xPosFromOdometry));
    getBytesFromFloat(positionPayload, 8, (float)(robotState.yPosFromOdometry));
    getBytesFromFloat(positionPayload, 12, (float)(robotState.angleRadianFromOdometry));
    getBytesFromFloat(positionPayload, 16, (float)(robotState.vitesseLineaireFromOdometry));
    getBytesFromFloat(positionPayload, 20, (float)(robotState.vitesseAngulaireFromOdometry));
    UartEncodeAndSendMessage(COMMAND_DATA, 24, positionPayload);
    LED_BLANCHE = !LED_BLANCHE;
   
    //on envoie les positions en mm et la vitesse linéaire en mm/s
    //les angles en rad et la vitesse agulaire en rad/s 
    
     // unsigned int i;
    //    long xPosFromOdo = robotState.xPosFromOdometry;
//    long yPosFromOdo = robotState.yPosFromOdometry;
//    long vitesseLin = robotState.vitesseLineaireFromOdometry*1000;
//    long vitesseAng = RadianToDegree(robotState.vitesseAngulaireFromOdometry) * 1000;
//    long angle = RadianToDegree(robotState.angleRadianFromOdometry)*1000 ;
//    
//    for(i = 0 ; i<4 ; i++){
//        positionPayload[3 - i] = (unsigned char)(timestamp>>(8*i));
//        positionPayload[7 - i] = (unsigned char)((xPosFromOdo)>>(8*i));//Transimission de xPosFromOdometry
//        positionPayload[11 - i] = (unsigned char)((yPosFromOdo)>> (8*i));//Transmission de yPosFromOdometry
//        positionPayload[15 - i] = (unsigned char)((angle) >> (8 * i)) ; //Transmission de l'angle du robot
//        positionPayload[19 - i] = (unsigned char)((vitesseLin)>>(8*i));//Transmission de la vitesse Linéaire
//        positionPayload[23 - i] = (unsigned char)((vitesseAng)>>(8*i));//Transmission de la vitesse angulaire
//    }
    
   
}
