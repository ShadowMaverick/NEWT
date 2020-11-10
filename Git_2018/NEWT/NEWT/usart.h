/*
* usart.h
*
* Created: 2018-08-29 13:37:38
*  Author: Max
*/


#include <inttypes.h>

#ifndef USART_H_
#define USART_H_

#ifndef F_CPU
// Default F_CPU value
#define F_CPU 16000000UL
#endif

// Initialize USART with given baudrate
void USART_Init(uint32_t baud);

// Wait until ready then send byte
void USART_Transmit_Byte(uint8_t data);

// Send data of given length 
void USART_Transmit(uint8_t data[], uint32_t length);

// Send a string
void USART_Transmit_String(char *text);

// Send a line of text
void USART_Transmit_Line(char *text);
// Wait for and receive a byte
uint8_t USART_Receive_Byte(void);


#endif /* USART_H_ */