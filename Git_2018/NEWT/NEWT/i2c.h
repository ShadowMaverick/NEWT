/*
 * i2c.h
 *
 * Created: 2018-09-17 15:05:26
 *  Author: Max
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <inttypes.h>

#define I2C_BUFFER_LENGTH 32

#ifndef F_CPU
// Default F_CPU value
#define F_CPU 16000000UL
#endif

void i2c_begin(void);
void i2c_begin_attach(uint8_t);
void i2c_end();
void i2c_setClock(uint32_t);
void i2c_beginTransmission(uint8_t);
uint8_t i2c_endTransmission(uint8_t);
uint8_t i2c_requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
uint8_t i2c_write_byte(uint8_t);
uint8_t i2c_write(const uint8_t *, uint32_t);
int i2c_available(void);
int i2c_read(void);
int i2c_peek(void);
//void i2c_flush(void);
void i2c_onReceive( void (*)(int) );
void i2c_onRequest( void (*)(void) );
void i2c_address_scan(void);

#endif /* I2C_H_ */