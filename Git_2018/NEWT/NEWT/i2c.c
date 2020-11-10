/*
* i2c.c
*
* Created: 2018-09-18 12:06:32
*  Author: Max
*/

#include <util/twi.h>
#include <stdio.h>
#include "i2c.h"
#include "twi.h"
#include "usart.h"

static uint8_t rxBuffer[I2C_BUFFER_LENGTH];
static uint8_t rxBufferIndex;
static uint8_t rxBufferLength;

static uint8_t txAddress;
static uint8_t txBuffer[I2C_BUFFER_LENGTH];
static uint8_t txBufferIndex;
static uint8_t txBufferLength;

static uint8_t transmitting;
static void (*user_onRequest)(void);
static void (*user_onReceive)(int);
static void onRequestService(void);
static void onReceiveService(uint8_t*, int);



// Function definitions

void i2c_begin(void)
{
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;

	twi_init();
}

void i2c_begin_attach(uint8_t address)
{
	twi_setAddress(address);
	twi_attachSlaveTxEvent(onRequestService);
	twi_attachSlaveRxEvent(onReceiveService);

	i2c_begin();
}

void i2c_end(void)
{
	twi_disable();
}

void i2c_setClock(uint32_t clock)
{
	twi_setFrequency(clock);
}

void i2c_beginTransmission(uint8_t address)
{
	// indicate that we are transmitting
	transmitting = 1;
	// set address of targeted slave
	txAddress = address;
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
}

uint8_t i2c_endTransmission(uint8_t sendStop)
{
	// transmit buffer (blocking)
	uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
	// indicate that we are done transmitting
	transmitting = 0;
	return ret;
}

uint8_t i2c_requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop)
{
	if (isize > 0) {
		// send internal address; this mode allows sending a repeated start to access
		// some devices' internal registers. This function is executed by the hardware
		// TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)

		i2c_beginTransmission(address);

		// the maximum size of internal address is 3 bytes
		if (isize > 3){
			isize = 3;
		}

		// write internal register address - most significant byte first
		while (isize-- > 0)
		i2c_write_byte((uint8_t)(iaddress >> (isize*8)));
		i2c_endTransmission(0);
	}

	// clamp to buffer length
	if(quantity > I2C_BUFFER_LENGTH){
		quantity = I2C_BUFFER_LENGTH;
	}
	// perform blocking read into buffer
	uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = read;

	return read;
}

uint8_t i2c_write_byte(uint8_t data)
{
	if(transmitting){
		// in master transmitter mode
		// don't bother if buffer is full
		if(txBufferLength >= I2C_BUFFER_LENGTH){
			// setWriteError();    // c++ stream thing I think
			return 0;
		}
		// put byte in tx buffer
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		// update amount in buffer
		txBufferLength = txBufferIndex;
		}else{
		// in slave send mode
		// reply to master
		twi_transmit(&data, 1);
	}
	return 1;
}

uint8_t i2c_write(const uint8_t *data, uint32_t quantity)
{
	if(transmitting){
		// in master transmitter mode
		for(uint32_t i = 0; i < quantity; ++i){
			if(!i2c_write_byte(data[i]))
			return 0;
		}
		}else{
		// in slave send mode
		// reply to master
		twi_transmit(data, quantity);
	}
	return 1;
}

int i2c_available(void)
{
	return rxBufferLength - rxBufferIndex;
}

int i2c_read(void)
{
	int value = -1;
	
	if(rxBufferIndex < rxBufferLength){
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}

	return value;
}

int i2c_peek(void)
{
	int value = -1;
	
	if(rxBufferIndex < rxBufferLength){
		value = rxBuffer[rxBufferIndex];
	}

	return value;
}

void i2c_onReceive( void (*function)(int) )
{
	user_onReceive = function;
}

void i2c_onRequest( void (*function)(void) )
{
	user_onRequest = function;
}

void onReceiveService(uint8_t* inBytes, int numBytes)
{
	// don't bother if user hasn't registered a callback
	if(!user_onReceive){
		return;
	}
	// don't bother if rx buffer is in use by a master requestFrom() op
	// i know this drops data, but it allows for slight stupidity
	// meaning, they may not have read all the master requestFrom() data yet
	if(rxBufferIndex < rxBufferLength){
		return;
	}
	// copy twi rx buffer into local read buffer
	// this enables new reads to happen in parallel
	for(uint8_t i = 0; i < numBytes; ++i){
		rxBuffer[i] = inBytes[i];
	}
	// set rx iterator vars
	rxBufferIndex = 0;
	rxBufferLength = numBytes;
	// alert user program
	user_onReceive(numBytes);
}

void onRequestService(void)
{
	// don't bother if user hasn't registered a callback
	if(!user_onRequest){
		return;
	}
	// reset tx buffer iterator vars
	// !!! this will kill any pending pre-master sendTo() activity
	txBufferIndex = 0;
	txBufferLength = 0;
	// alert user program
	user_onRequest();
}

void i2c_address_scan(void)
{
	uint8_t error, address;
	int nDevices;
	char addressBuffer[8];

	USART_Transmit_Line("Scanning...");

	nDevices = 0;
	for(address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		i2c_beginTransmission(address);
		error = i2c_endTransmission(1);

		if (error == 0)
		{
			sprintf(addressBuffer, "%.2X", address);
			USART_Transmit_String("I2C device found at address 0x");
			USART_Transmit_String(addressBuffer);
			USART_Transmit_Line("  !");

			nDevices++;
		}
		else if (error==4)
		{
			sprintf(addressBuffer, "%.2X", address);
			USART_Transmit_String("Unknown error at address 0x");
			USART_Transmit_String(addressBuffer);
			USART_Transmit_Line("  !");
		}
		else if (error==5)
		{
			sprintf(addressBuffer, "%.2X", address);
			USART_Transmit_String("Timeout at address 0x");
			USART_Transmit_String(addressBuffer);
			USART_Transmit_Line("  !");
		}
	}
	if (nDevices == 0)
	USART_Transmit_Line("No I2C devices found\n");
	else
	USART_Transmit_Line("done\n");
}