/* 
 * File:   CB_TX1.h
 * Author: table4
 *
 * Created on 5 novembre 2019, 14:47
 */

#ifndef CB_TX1_H
#define	CB_TX1_H

void SendMessage(unsigned char* message, int length);
void CB_TX1_Add(unsigned char value);
unsigned char CB_TX1_Get(void);
void SendOne();
unsigned char CB_TX1_IsTransmitting(void);
int CB_TX1_RemainingSize (void);

#endif	/* CB_TX1_H */

