/*
* NEWT.c
*
* Created: 2018-09-11 12:31:13
* Author : Max
*/

#include "math.h"

//#define _CALIBRATION
#define F_CPU 16000000UL
#define BAUD 250000UL
#define RECBUFFER_LEN 32

// IMU defines
#define IMU_MAX_ENCVAL 0x7FFFF  //131071 in dec ??
#define IMU_ID 0x77	// Set accelerometer reference vector for setting filter parameter
#define IMU_BUFFER 3
#define IMU_CLOCK 300000
#define IMU_ROLL_AXIS 1			// This is used to select the correct axis
#define IMU_POS_MAX	M_PI		// pi
#define IMU_VEL_MAX 20

// Encoder defines
#define ENCODER_TIMER_HZ 2000000
#define ENCODER_TICK_PER_S 0.0000005f
#define ENCODER_MIN_ANGLE -0.75f
#define ENCODER_MAX_ANGLE 0.75f
#define ENCODER_MID_POINT 8.2857f

// Global clock define
#define TIMER_MAX 0xFFFF

// Control defines
#define PREC_REF 0.0f
#define PREC_TO_ZERO_MAX_PWM 70

// Numerator factors
#define C_OUT -0.4895f
#define C_OUT1 0.9776f
#define C_OUT2 -0.4881f
#define C_IN 564.0201f
#define C_IN1 -1070.9f
#define C_IN2 508.6169f

// Denumerator factors
#define D_OUT1 -1.9673f
#define D_OUT2 0.9673f
#define D_IN1 -1.5742f
#define D_IN2 0.5742f

// LQR factors
#define LQR_ROLL_ANGLE_GAIN 594.5661f
#define LQR_PRECESSION_ANGLE_GAIN -70.6815f
#define LQR_ROLL_RATE_GAIN 118.4434f
#define LQR_PRECESSION_RATE_GAIN 17.2846f

// Precession defines
#define PREC_MAX_TORQUE 69.0f

// Buffers
#define BUFFER_SIZE 100			// Buffers take up BUFFER_SIZE * 24 bytes of SRAM, we have a total of 8kB

// Test
#define TEST_MAX_SPEED 255

#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "i2c.h"

void i2ctest(uint8_t deviceId);

// Initialize Precession motor PWM, Clocks that handle interrupts, IMU,
// SSI communication for encoder

void InitPrecessionMotor();
void InitClocks();
char IMU_init();
void InitSSI();

// Read IMU sensor values
char IMURead_eulerAngle(int); 		//	returns what?
char IMURead_eulerVelocities(int);	//	returns what?

// Handle the control of the system, 
void ControlPID(int);
void ControlLQR(int);
void ControlTest(int);

char EncoderRead(int);
uint32_t ReadSSI();
void shutDown();
void resetDataToRaspberry();
void logDataToRaspberry();
char isSafe();
void precessToZero();

// Encoder variables
float encfloat = 0;            // Encoder value in deg
uint32_t encval = 0;           // Encoder position
uint32_t encval_offset = 0;    // Help variable
uint16_t oldtimer = 0;
uint16_t timer = 0;
float prec_velocity = 0;
int long deltaClock = 1;

// Global IMU variables
float euler_angles[3] = {0, 0, 0};        // Global angles from IMU
float euler_velocities[3] = {0, 0, 0};    // Global velocities from IMU

// Global variables
volatile char flag = 0;                          // Global Execution variable

// Global buffers to send to raspberry
volatile long int counter = 0;			//Auxiliary counter for timer
long int timerBuffer[BUFFER_SIZE];		//Time of data into buffer
float IMUX_angle[BUFFER_SIZE];			//IMU X angle data into buffer
float IMUX_velocity[BUFFER_SIZE];		//IMU X velocity data into buffer
float angleBuffer[BUFFER_SIZE];			//Encoder angle data into buffer
float velocityBuffer[BUFFER_SIZE];		//Encoder velocity data into buffer
float controlBuffer[BUFFER_SIZE];		//Control data into buffer

/*****************************************************************************************
*                                         Interrupts
*                              Control Enabler Interrupt 10ms
*****************************************************************************************/
ISR(TIMER1_COMPA_vect)
{
	flag = 1;               // flag = 1: executes control function
	counter++;
}

int main(void)
{
	resetDataToRaspberry();

	// Starts USART (serial) communication to the raspberry pi
	USART_Init(BAUD);
	// Initialize communication with the IMU
	i2c_begin();
	i2c_setClock(IMU_CLOCK);
	
	// Setup PWM for the precession motor
	InitPrecessionMotor();
	// Initialize timer interrupts
	InitClocks();
	// Initialize communication with the encoder
	InitSSI();

	// Activate interrupts
	sei();

	int iter = 0;
	while (1)
	{
		while (flag == 0);
		if (iter >= 100) {
			break;
		}
		iter++;
		flag = 0;
	}
	
	//while (flag == 0);
	
	//Initialize the IMU
	if (!IMU_init())
	{
		shutDown();
		return 0;
	}

	flag = 0;

	// Wait for message from the pi before we continue
	uint8_t recBuffer[RECBUFFER_LEN] = "";
	uint8_t recLen = 0, last = 'A';

	USART_Transmit_Line("Please enter command");
	while (last != '\n' && recLen < RECBUFFER_LEN)
	{
		last = USART_Receive_Byte();
		recBuffer[recLen] = last;
		recLen++;
	}

	USART_Transmit(recBuffer, recLen);

#ifndef _CALIBRATION
	precessToZero();
#endif

	int buffCount = 0;
	while (1)
	{
		while (flag == 0);
		
		if (!IMURead_eulerAngle(buffCount)) {
			break;
		}
		if (!IMURead_eulerVelocities(buffCount)) {
			break;
		}
#ifndef _CALIBRATION
		if (!EncoderRead(buffCount)) {
			break;
		}
#else
		EncoderRead(buffCount);
#endif

		timerBuffer[buffCount]=counter*0.01;

#ifndef _CALIBRATION
		//ControlTest(buffCount);
		//ControlPID(buffCount);
		ControlLQR(buffCount);
#endif

		flag = 0;
		buffCount++;

		if (buffCount >= BUFFER_SIZE)
		buffCount = 0;
	}
	
	shutDown();
}

/*********************************************************************
*              Initialize precession motor communication,
*********************************************************************/

void InitPrecessionMotor() // Initializes the communication with the precession motor
{
	// Set enable and direction
	DDRH |= _BV(DDH6);		// pinMode(9, OUTPUT);     //Motor Enable
	PORTH |= _BV(PORTH6);	// digitalWrite(9, HIGH);  //Enable ON
	DDRE |= _BV(DDE3);		// pinMode(5, OUTPUT);     //Motor direction
	PORTE &= ~_BV(PORTE3);	// digitalWrite(5, LOW);   //Drirection CW

	// Set up 250 Hz PWM
	DDRG |= _BV(PG5);                       // PG5 as output PWM pin, Percession motor
	DDRB |= _BV(PB7);						// PB7 as output, Velocity motor
	TCCR0A = 0;								// Reset register
	TCCR0B = 0;								// Reset register
	TCCR0A |= _BV(COM0A1) | _BV(COM0B1);	// set non-inverting mode
	TCCR0A |= _BV(WGM01) | _BV(WGM00);      // set Fast PWM Mode
	TCCR0B |= _BV(CS02);					// set prescaler to 256 (244Hz)
	OCR0B  = 0;                             // 0 velocity of percesion motor, 255 max speed 40RPM
	OCR0A  = 161;							// 1.5ms =  161: standstill (Velocity motor)
}

/*********************************************************************
*          Initialize Clocks, interrupt & universal
*********************************************************************/

void InitClocks() // Initializes clock interrupt
{
	// Control interrupt
	TCCR1A = 0; //Clear Register
	TCCR1B = 0; //Clear Register
	TCNT1  = 0; //Clear Register
	OCR1A = 156;            // Compare interrupt time: clk/(prescale * (1 + OCR1A)	1 byte -> max 255
	TCCR1B |= _BV(WGM12);   // CTC mode - Clear Timer on Compare match
	TCCR1B |= _BV(CS12) | _BV(CS10);    // no prescaler - 001: 1 / 010: 8 / 011: 64 / 100: 256 / 101: 1024
	TIMSK1 |= _BV(OCIE1A);  // enable timer compare interrupt

	// Universial clock
	TCCR3A = 0; //Clear Register
	TCCR3B = 0; //Clear Register
	TCNT3  = 0; //Clear Register
	TCCR3B |= _BV(CS31) | _BV(CS30);    // no prescaler - 001: 1 / 010: 8 / 011: 64 / 100: 256 / 101: 1024
}

/*********************************************************************
*						Initialize the IMU
*********************************************************************/

char IMU_init()
{
	//Serial.print("Starting Setup Transmission to IMU");
	i2c_beginTransmission(IMU_ID);
	//Serial.println("before Transmission to IMU");
	i2c_write_byte(0x42);
	i2c_write_byte(0x60);
	//Serial.println("After transmissions");
	int error = i2c_endTransmission(1);
	//Serial.print("Setup Transmission to IMU finished");
	
	if (error == 5)
	{
		USART_Transmit_Line("IMU_init: I2C communication timeout");
		return 0;
	}

	return 1;
}

/*********************************************************************
*              Initalize SPI to mimic SSI
*********************************************************************/

void InitSSI() // Initialize the encoder communication
{
	DDRB = _BV(DDB1) | _BV(DDB3) | _BV(DDB0);   // Configure SCK, MOSI and  Slave Select
	SPCR |= _BV(SPR1);                // divided clock by 128
	SPCR |= _BV(SPE) | _BV(MSTR) | _BV(CPOL);    // Configure SPI as master, with CLK idle high
	encval_offset = ReadSSI();                  // Save initial value
}

/*********************************************************************
*              Read IMU function, gyro values
*********************************************************************/

char IMURead_eulerAngle(int buffCount) // Gets new data from the IMU
{
	union {
		unsigned char bytes[4];
		float angle;
	} temp_angle;

	//Serial.print("Starting Setup Transmission to IMU");
	i2c_beginTransmission(IMU_ID);
	//Serial.println("before Transmission to IMU");
	i2c_write_byte(0x42); // prepare sensor to recieve a TSS command
	i2c_write_byte(0x01); // Read filtered, tared orientation(Euler Angles)
	i2c_write_byte(0x43); // prepares sensor to send out data to master device
	//Serial.println("After transmissions");
	int error = i2c_endTransmission(1);
	//Serial.print("Setup Transmission to IMU finished");
	
	if (error == 5)
	{
		USART_Transmit_Line("IMURead_eulerAngle: I2C communication timeout");
		return 0;
	}

	i2c_requestFrom(IMU_ID, 12, 0, 0, 1);
	for (int i = 0; i < 3; i++)
	{
		temp_angle.angle = 0;
		for (int j = 3; j >= 0; j--)
		{
			temp_angle.bytes[j] = i2c_read();
		}

		if (abs(temp_angle.angle) <= IMU_POS_MAX) {
			euler_angles[i] = -temp_angle.angle;	// Invert to match model
		}
	} 

	IMUX_angle[buffCount] = euler_angles[IMU_ROLL_AXIS];
	return 1;
}

/*********************************************************************
*              Read IMU function, gyro values
*********************************************************************/

char IMURead_eulerVelocities(int buffCount) // Gets new data from the IMU
{
	union {  //a data type that stores different data types
		unsigned char bytes[4];
		float angle_velocity;
	} temp_angle_velocity;

	//Serial.print("Starting Setup Transmission to IMU");
	i2c_beginTransmission(IMU_ID);
	//Serial.println("before Transmission to IMU");
	i2c_write_byte(0x42); //  prepares sensor to receive TSS command and any additional arguments.
	i2c_write_byte(0x26); // Read gyros (vector floatx3) corrected sensor data
	i2c_write_byte(0x43); // prepares sensor to send out data to master device
	
	//Serial.println("After transmissions");
	int error = i2c_endTransmission(1);
	//Serial.print("Setup Transmission to IMU finished");
	
	if (error == 5)
	{
		USART_Transmit_Line("IMURead_eulerVelocities: I2C communication timeout");
		return 0;
	}

	i2c_requestFrom(IMU_ID, 12, 0, 0, 1); // (address, quantity, )
	for (int i = 0; i < 3; i++)
	{
		temp_angle_velocity.angle_velocity = 0;
		for (int j = 3; j >= 0; j--)
		{
			temp_angle_velocity.bytes[j] = i2c_read();
		}

		if (temp_angle_velocity.angle_velocity <= IMU_VEL_MAX) {
			euler_velocities[i] = -temp_angle_velocity.angle_velocity;	// Invert to match model
		}
	}
	
	IMUX_velocity[buffCount] = euler_velocities[IMU_ROLL_AXIS];
	return 1;
}

/*********************************************************************
*                      Control function
*********************************************************************/
/*
void ControlPID(int buffCount) // Calculates and sends control values to the precession motor
{
	// Outer control loop
	float prec_error = PREC_REF - encfloat;
	static float prec_error_old1 = 0.0f;
	static float prec_error_old2 = 0.0f;
	float ref_inner = 0.0f;
	static float ref_inner_old1 = 0.0f;
	static float ref_inner_old2 = 0.0f;

	ref_inner = C_OUT * prec_error + C_OUT1 * prec_error_old1 + C_OUT2 * prec_error_old2 - D_OUT1 * ref_inner_old1 - D_OUT2 * ref_inner_old2;

	prec_error_old2 = prec_error_old1;
	prec_error_old1 = prec_error;
	ref_inner_old2 = ref_inner_old1;
	ref_inner_old1 = ref_inner;

	// Inner control loop
	float roll_error = ref_inner - euler_velocities[IMU_ROLL_AXIS];
	static float roll_error_old1 = 0.0f;
	static float roll_error_old2 = 0.0f;
	float control = 0.0f;
	static float control_old1 = 0.0f;
	static float control_old2 = 0.0f;

	control = C_IN * roll_error + C_IN1 * roll_error_old1 + C_IN2 * roll_error_old2 - D_IN1 * control_old1 - D_IN2 * control_old2;

	roll_error_old2 = roll_error_old1;
	roll_error_old1 = roll_error;
	control_old2 = control_old1;
	control_old1 = control;

	controlBuffer[buffCount] = control;
	// TODO convert from control to OCR0B
	if (control < 0) { // Motor direction
		PORTE |= _BV(PORTE3);	// digitalWrite(5, LOW);   //Drirection CW
	}
	else {
		PORTE &= ~_BV(PORTE3);	// digitalWrite(5, LOW);   //Drirection CCW
	}

	char buffer[32];
	USART_Transmit_String("control: ");
	dtostrf(control, 4, 4, buffer);
	USART_Transmit_String(buffer);
	USART_Transmit_String("\troll: ");
	dtostrf(euler_velocities[IMU_ROLL_AXIS], 4, 4, buffer);
	USART_Transmit_String(buffer);
	USART_Transmit_String("\troll_error: ");
	dtostrf(roll_error, 4, 4, buffer);
	USART_Transmit_String(buffer);
	USART_Transmit_String("\tprec_error: ");
	dtostrf(prec_error, 4, 4, buffer);
	USART_Transmit_Line(buffer);

	control = min(abs(control), PREC_MAX_TORQUE);
	OCR0B = (uint8_t)(255 * (control/PREC_MAX_TORQUE));
}*/

/*********************************************************************
*                      Control function
*********************************************************************/

void ControlLQR(int buffCount) // Calculates and sends control values to the precession motor
{
	// Outer control loop
	float control = -(encfloat * LQR_PRECESSION_ANGLE_GAIN +
	prec_velocity * LQR_PRECESSION_RATE_GAIN +
	euler_angles[IMU_ROLL_AXIS] * LQR_ROLL_ANGLE_GAIN +
	euler_velocities[IMU_ROLL_AXIS] * LQR_ROLL_RATE_GAIN);

	controlBuffer[buffCount] = control;
	// TODO convert from control to OCR0B
	if (control < 0) { // Motor direction
		PORTE |= _BV(PORTE3);	// digitalWrite(5, LOW);   //Drirection CW
	}
	else {
		PORTE &= ~_BV(PORTE3);	// digitalWrite(5, LOW);   //Drirection CCW
	}
	control = min(abs(control), PREC_MAX_TORQUE);
	OCR0B = (uint8_t)(255 * (control/PREC_MAX_TORQUE));
}

/*********************************************************************
*                      Control test function
*********************************************************************/

void ControlTest(int buffCount) // Calculates and sends control values to the precession motor
{
	//if ((encfloat < -0.3f && !(PORTE & _BV(PORTE3))) || (encfloat > 0.3f && PORTE & _BV(PORTE3)))
	//{
	//PORTE = (PORTE & _BV(PORTE3)) ? PORTE & ~_BV(PORTE3) : PORTE | _BV(PORTE3);
	//}

	float control = min(fabs(euler_angles[IMU_ROLL_AXIS]) / M_PI_2, 1.0f);

	//if (encfloat < 0.0f) PORTE |= _BV(PORTE3);
	//else PORTE &= ~_BV(PORTE3);
	//
	//float control = fabs(encfloat);

	char buffer[32];
	USART_Transmit_String("encfloat: ");
	dtostrf(encfloat, 4, 4, buffer);
	USART_Transmit_Line(buffer);
	//USART_Transmit_String("euler_angles[IMU_ROLL_AXIS]: ");
	//dtostrf(euler_angles[IMU_ROLL_AXIS], 4, 4, buffer);
	//USART_Transmit_String(buffer);
	//USART_Transmit_String(" euler_velocities[IMU_ROLL_AXIS]: ");
	//dtostrf(euler_velocities[IMU_ROLL_AXIS], 4, 4, buffer);
	//USART_Transmit_String(buffer);
	//sprintf(buffer, "%lu", encval);
	//unsigned char torquePWM = TEST_MAX_SPEED * control;
	//USART_Transmit_String(" PWM: ");
	//dtostrf(control, 4, 2, buffer);
	//sprintf(buffer, "%u", torquePWM);
	//USART_Transmit_Line(buffer);

	//OCR0B = 0;

	controlBuffer[buffCount] = control;
}

/*********************************************************************
*                    Read encoder function
*********************************************************************/

char EncoderRead(int buffCount) // Requests values from the Encoder
{
	oldtimer = timer;						// Last data point timer
	float oldEncfloat = encfloat;			// Save previous time
	timer = TCNT3;
	float deltaClock = 0.0f;

	if ( (timer - oldtimer) < 0) {
		deltaClock = (timer - oldtimer + TIMER_MAX) * ENCODER_TICK_PER_S;
	}
	else {
		deltaClock = (timer - oldtimer) * ENCODER_TICK_PER_S;
	}

	encval = ReadSSI();												// Get encoder position
	encfloat = ((float)encval * 8 * M_PI)/ (IMU_MAX_ENCVAL + 1);    // Counts to radians
#ifndef _CALIBRATION
	encfloat -= ENCODER_MID_POINT;									// Centers the encoder value so that straight up is 0
	encfloat = -encfloat;											// Inverts value in order to match model
#else
	char buffer[32];
	USART_Transmit_String("encfloat: ");
	dtostrf(encfloat, 4, 4, buffer);
	USART_Transmit_Line(buffer);
#endif
	prec_velocity = (encfloat - oldEncfloat) / deltaClock;			// Calculate the velocity
	angleBuffer[buffCount] = encfloat;								// Encoder angle data into buffer
	velocityBuffer[buffCount] = prec_velocity;

	return encfloat >= ENCODER_MIN_ANGLE && encfloat <= ENCODER_MAX_ANGLE;
}

/*********************************************************************
*						Read SSI via SPI
*********************************************************************/

uint32_t ReadSSI() //Mimic SSI communication
{
	uint8_t u8byteCount;      // Data length counter
	uint8_t u8data;           // Data temporary variable
	uint32_t u32result = 0;   // Data complete variable

	for(u8byteCount = 0; u8byteCount < 4; u8byteCount++)            // Fetch data function
	{
		// send a dummy byte, read the result
		SPDR = 0xFF;                      // Send dummy message
		u32result <<= 8;                  // Shift old result
		while( (SPSR & (1<<SPIF)) == 0 ); // Wait for new data
		u8data = SPDR;                    // Store data
		u32result |= u8data;              // Update complete data variable
	}
	//u32result = u32result >> 7;					// The last seven bits are timeout
	return ((u32result >> 12) & IMU_MAX_ENCVAL);		// Return only absolute position
}

/*********************************************************************
*							precessToZero
*********************************************************************/

void precessToZero()
{
	//return;
	int buffCount = 0;
	double tickToSec = 1.0 / (F_CPU / 64.0);
	uint32_t totalTime = 0;
	uint16_t lastTime = TCNT3;

	int iter = 0;
	// Start up sequence
	while (1)
	{
		if (iter >= 500) {
			break;
		}

		while (flag == 0);
		
		if (!IMURead_eulerAngle(buffCount)) {
			break;
		}
		if (!IMURead_eulerVelocities(buffCount)) {
			break;
		}

		EncoderRead(buffCount);

		if (encfloat < 0.0f) PORTE &= ~_BV(PORTE3);
		else PORTE |= _BV(PORTE3);

		float control = fabs(encfloat);
		unsigned char torquePWM = min(TEST_MAX_SPEED * control, PREC_TO_ZERO_MAX_PWM);


		totalTime += (uint16_t)(TCNT3 - lastTime);
		lastTime = TCNT3;
		OCR0B = torquePWM;
		flag = 0;
		iter++;
		buffCount++;
	}
	
	char buffer[32];
	USART_Transmit_String("Time spent in precessToZero: ");
	sprintf(buffer, "%lu", totalTime);
	USART_Transmit_String(buffer);
	USART_Transmit_String(" ticks, ");
	dtostrf(totalTime * tickToSec, 4, 2, buffer);
	USART_Transmit_String(buffer);
	USART_Transmit_Line(" seconds.");
	OCR0B = 0;
}

/*********************************************************************
*							shutDown
*********************************************************************/

void shutDown()
{
	USART_Transmit_Line("Shutting down");

	char isPositive = prec_velocity > 0.0f;
	
	// Set torque direction
	if (encfloat < 0.0f) PORTE &= ~_BV(PORTE3);
	else PORTE |= _BV(PORTE3);
	OCR0B = 90;

	float lastPrecVel = 10.0f;

	// Start up sequence
	while (1)
	{
		while (flag == 0);

		char posi = prec_velocity > 0.0f;
		if (posi != isPositive || (lastPrecVel < 0.01f && prec_velocity < 0.01f)) {
			break;
		}
		
		lastPrecVel = prec_velocity;
		EncoderRead(0);
		flag = 0;
	}

	USART_Transmit_Line("Breaking done");

#ifndef _CALIBRATION
	precessToZero();
#endif

	//Shut down precession motor enabler
	PORTH &= ~_BV(PORTH6); //digitalWrite(9, LOW);  //Enable OFF

	//Shut down velocity motor enabler
	//Enable OFF

	//Store whatever data is now stored in buffer
	logDataToRaspberry();

	//Software Reset IMU
	i2c_beginTransmission(IMU_ID);
	i2c_write_byte(0xE2);
	i2c_endTransmission(1);

	//Close Serial communication
	//Serial.close();
	//Serial2.close();

	//Disable interrupts
	cli();
}

/*********************************************************************
*						resetDataToRaspberry
*********************************************************************/

void resetDataToRaspberry(){

	counter=0;															//Auxiliary counter for timer
	memset(timerBuffer, 0, sizeof(*timerBuffer) * BUFFER_SIZE);			//Time of data into buffer
	memset(IMUX_angle, 0, sizeof(*IMUX_angle) * BUFFER_SIZE);			//IMU X angle data into buffer
	memset(IMUX_velocity, 0, sizeof(*IMUX_velocity) * BUFFER_SIZE);		//IMU X velocity data into buffer
	memset(angleBuffer, 0, sizeof(*angleBuffer) * BUFFER_SIZE);		//Encoder angle data into buffer
	memset(velocityBuffer, 0, sizeof(*velocityBuffer) * BUFFER_SIZE);	//Encoder velocity data into buffer
	memset(controlBuffer, 0, sizeof(*controlBuffer) * BUFFER_SIZE);			//Control input u, into buffer
}

/*********************************************************************
*						logDataToRaspberry
*********************************************************************/

void logDataToRaspberry(){
	USART_Transmit_Line("\ntimerBuffer:");
	USART_Transmit((uint8_t*)timerBuffer, sizeof(*timerBuffer) * BUFFER_SIZE);
	USART_Transmit_Line("\nIMUX_angle:");
	USART_Transmit((uint8_t*)IMUX_angle, sizeof(*IMUX_angle) * BUFFER_SIZE);
	USART_Transmit_Line("\nIMUX_velocity:");
	USART_Transmit((uint8_t*)IMUX_velocity, sizeof(*IMUX_velocity) * BUFFER_SIZE);
	USART_Transmit_Line("\nangleBuffer:");
	USART_Transmit((uint8_t*)angleBuffer, sizeof(*angleBuffer) * BUFFER_SIZE);
	USART_Transmit_Line("\nvelocityBuffer:");
	USART_Transmit((uint8_t*)velocityBuffer, sizeof(*velocityBuffer) * BUFFER_SIZE);
	USART_Transmit_Line("\ncontrolBuffer:");
	USART_Transmit((uint8_t*)controlBuffer, sizeof(*controlBuffer) * BUFFER_SIZE);
}

/*********************************************************************
*								isSafe
*********************************************************************/

char isSafe()
{
	//if(abs(euler_angles[0]) > 6.28f|| abs(euler_angles[1]) > 6.28f || abs(euler_angles[2]) > 6.28f){
	//return 0;
	//}
	//
	//if(abs(encfloat) > 3.1415/2 || abs(prec_velocity) > 3.1415/2){
	//return 0;
	//}

	return 1;
}

/*********************************************************************
*								i2ctest
*********************************************************************/

void i2ctest(uint8_t deviceId)
{
	i2c_beginTransmission(deviceId);

	i2c_write_byte(0x42);
	i2c_write_byte(0x01);
	i2c_write_byte(0x43);

	int error = i2c_endTransmission(1);

	if (error == 5) {
		USART_Transmit_Line("Transmission timed out");
		return;
	}

	unsigned int length = i2c_requestFrom(deviceId, 12, 0, 0, 1);

	char buffer[32];
	sprintf(buffer, "%d", length);
	//USART_Transmit_String("Number of bytes received: ");
	//USART_Transmit_Line(buffer);

	union {
		uint8_t bytes[4];
		uint32_t  binary;
		float  angle;
	} temp;

	for (int i = 0; i < 3; i++)
	{
		temp.binary = 0;
		for (int j = 0; j < 4; j++) {
			temp.bytes[3-j] = i2c_read();
		}
		//sprintf(buffer, "%.8lX", (temp.binary));
		//USART_Transmit_String(buffer);
		//sprintf(buffer, "%.4X", (uint16_t)temp.binary);
		//USART_Transmit_String(buffer);
		dtostrf(temp.angle, 4, 2, buffer);
		USART_Transmit_String(buffer);
		USART_Transmit_String(", ");
	}
	USART_Transmit_Line("");
}
