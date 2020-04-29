 
#include <xc.h>
#include "UART_Protocol.h"
#include "CB_TX1.h"
#include "main.h"
#include "Robot.h"
#include "IO.h"
#include "Utilities.h"

#define BUFFER_SIZE 128


unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload)
{
    //Fonction prennant en entrée la trame et sa longueur pour calculer le checksum
    unsigned char checksum;
    checksum = 0xFE;
    checksum ^= (unsigned char)msgFunction;
    checksum ^= (unsigned char)(msgFunction >> 8);
    checksum ^= (unsigned char)msgPayloadLength;
    checksum ^= (unsigned char)(msgPayloadLength >> 8);
    int i;
    for (i = 0; i < msgPayloadLength; i++)
        checksum ^= msgPayload[i];
    return checksum;
}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload)
{
    //Fonction d'encodage et d'envoi des messages
    unsigned char message[msgPayloadLength + 6];
    
    int index = 0;
    message[index++] = 0xFE;
    message[index++] = (unsigned char)(msgFunction >> 8);
    message[index++] = (unsigned char)msgFunction;
    message[index++] = (unsigned char)(msgPayloadLength >> 8);
    message[index++] = (unsigned char)msgPayloadLength;
    int i;
    for (i = 0; i < msgPayloadLength; i++)
        message[index++] = msgPayload[i];
    message[index++] = UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
    
    SendMessage(message, index);
}

int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[BUFFER_SIZE];
int msgDecodedPayloadIndex = 0;
unsigned char rcvState = WAITING;

void UartDecodeMessage(unsigned char c)
{
    //Fonction prennant en entrée un octet et servant à reconstituer les trames
    switch(rcvState)
    {
        case WAITING:
            if (c == 0xFE)
                rcvState = FUNCTION_MSB;
            break;
            
        case FUNCTION_MSB:
            msgDecodedFunction = c << 8;
            rcvState = FUNCTION_LSB;
            break;
            
        case FUNCTION_LSB:
            msgDecodedFunction += c;
            rcvState = PAYLOAD_LENGTH_MSB;
            break;
            
        case PAYLOAD_LENGTH_MSB:
            msgDecodedPayloadLength = c << 8;
            rcvState = PAYLOAD_LENGTH_LSB;
            break;
            
        case PAYLOAD_LENGTH_LSB:
            msgDecodedPayloadLength += c;
            rcvState = PAYLOAD;
            break;
            
        case PAYLOAD:
            if (msgDecodedPayloadLength >= BUFFER_SIZE)
            {
                msgDecodedFunction = 0;
                msgDecodedPayloadLength = 0;
                rcvState = WAITING;
            }
            else if(msgDecodedPayloadLength > 0)
            {
                msgDecodedPayload[msgDecodedPayloadIndex++] = c;
                if(msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                {
                    msgDecodedPayloadIndex = 0;
                    rcvState = CHECKSUM;
                }
            }
            else
            {
                rcvState = CHECKSUM;
            }
            break;
            
        case CHECKSUM:
        {
            unsigned char calculatedChecksum = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            unsigned char receivedChecksum = c;
            if (calculatedChecksum == receivedChecksum)
            {
                //ok
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            }
            
            rcvState = WAITING;
            break;
        }
    }
}
//
void UartProcessDecodedMessage(unsigned char function, unsigned char payloadLength, unsigned char* payload)
{
    //Fonction appelé après le décodage pour exécuter l'action correspondant au message recu
    switch(function)
    {
        case COMMAND_ID_SET_ROBOT_STATE:
            SetRobotState(payload[0]);
            break;
            
        case COMMAND_ID_SET_ROBOT_AUTO_CONTROL:
            SetRobotAutoControlState(payload[0]);
            break;
            
        case COMMAND_ID_SET_PWM_SPEED:
            //Pour le vitesses négatives, le bit de poids fort est à 1, sinon il est a 0
            //La case 0 est pour la vitesse gauche
            if (0x80 == (0x80 & payload[0]))
                robotState.vitesseGaucheConsigne = -(payload[0] & ~0x80);
            else 
                robotState.vitesseGaucheConsigne = payload[0] & ~0x80;
            
            //La case 1 est pour la vitesse droite
            if (0x80 == (0x80 & payload[1]))
                robotState.vitesseDroiteConsigne = -(payload[1] & ~0x80);
            else
                robotState.vitesseDroiteConsigne = payload[1] & ~0x80;
            break;
            
        case COMMAND_ID_LED:
            switch(payload[0])
            {
                case 0:
                    //Led orange
                    LED_ORANGE = payload[1];
                    break;
                    
                case 1:
                    //Led bleue
                    LED_BLANCHE = payload[1];
                    break;
                    
                case 2:
                    //Led blanche
                    LED_BLEUE = payload[1];
                    break;
            }
            break;
            
        case COMMAND_ID_SET_VITESSE_POLAIRE:
            //on envoie les consigne de vitesse linéraie et polaire de l'asservissement
               
            robotState.vitesseAngulaireConsigne = getDouble(payload, 0);
            robotState.vitesseLineaireConsigne = getDouble(payload, 4);  

            break;
    }
}
