/* 
 * File:   UART_Protocol.h
 * Author: table4
 *
 * Created on 6 décembre 2019, 09:02
 */
#ifndef UART_PROTOCOL_H
#define	UART_PROTOCOL_H

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartDecodeMessage(unsigned char c);
void UartProcessDecodedMessage(unsigned char function, unsigned char payloadLength, unsigned char* payload);

#define COMMAND_ID_TEXT 0x0080
#define COMMAND_ID_JACK 0x0010
#define COMMAND_ID_LED 0x0020
#define COMMAND_ID_IR 0x0030
#define COMMAND_ID_PWM 0x0040
#define COMMAND_ID_STATE 0x0050
#define COMMAND_ID_SET_ROBOT_STATE 0x0051
#define COMMAND_ID_SET_ROBOT_AUTO_CONTROL 0x0052
#define COMMAND_ID_SET_PWM_SPEED 0x0053
#define COMMAND_ID_SET_LED 0x0054
#define COMMAND_DATA 0x0061
#define COMMAND_POLAIRE 0x0062
#define COMMAND_ID_SET_VITESSE_POLAIRE 0x0063

#define WAITING 0
#define FUNCTION_MSB 1
#define FUNCTION_LSB 2
#define PAYLOAD_LENGTH_MSB 3
#define PAYLOAD_LENGTH_LSB 4
#define PAYLOAD 5
#define CHECKSUM 6


#endif	/* UART_PROTOCOL_H */



//#ifndef UART_PROTOCOL_H
//#define	UART_PROTOCOL_H
//
//#define Waiting  1
//#define FunctionMSB  2
//#define FunctionLSB  3
//#define PayloadLengthMSB  4
//#define PayloadLengthLSB 5
//#define Payload  6
//#define CheckSum  7
//
//
//#define   TransmissionText 0x0080
//#define   ReglageLed 0x0020
//#define    DistTelemIR  0x0030
//#define    ConsigneVitesse  0x0040
//
//unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
//void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
//void UartDecodeMessage(unsigned char c);
//void UartProcessDecodedMessage(unsigned char function, unsigned char payloadLength, unsigned char* payload);
//
//#endif	/* UART_PROTOCOL_H */

