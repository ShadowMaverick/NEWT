/*
* usart.c
*
* Created: 2018-09-17 14:22:32
*  Author: Max
*/

#include <avr/io.h>
#include <string.h>
#include "usart.h"

void USART_Init(uint32_t baud)
{
	unsigned int ubrr = ((F_CPU)/16/(baud)-1);
	/* Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
void USART_Transmit_Byte(uint8_t data)
{
	// Wait until the transmit buffer is empty
	while(!(UCSR0A & (1<<UDRE0)))
	;

	// Put data into buffer (sends the data)
	UDR0 = data;
}
		USART_Transmit_Byte(*curbyte);
uint8_t USART_Receive_Byte(void)
{
	// Wait while data is being received
	while(!((UCSR0A) & (1<<RXC0)))
	;

	// Return 8-bit data
	return UDR0;
}