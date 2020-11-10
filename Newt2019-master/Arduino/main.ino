#include <Wire.h>
#include <math.h>

#define BAUD 250000UL		//Communication with RPi
#define BUFFER_SIZE 100

#define IMU_ID 0x77
#define IMU_PIN 21              //SCL
#define IMU_MAX_ENCVAL 0x7FFFF  //524287 in dec ??
#define IMU_BUFFER 3
#define IMU_CLOCK 300000	// I2C clock rate 300kHz
#define IMU_ROLL_AXIS 1			// This is used to select the correct axis
#define IMU_POS_MAX	M_PI		// pi
#define IMU_VEL_MAX 20

/** DEFINES FOR AMT33 ENCODER **/
#define AMT22_NOP       0x00 // SPI Command for No operation(the encoder does not care about the content of the first byte, we will define it as a NOP (no-operation))
#define AMT22_RESET     0x60 // SPI Command for Encoder reset
#define AMT22_ZERO      0x70 // SPI Command for setting zero in encoder
#define AMT22_RES12		12 // Setting resolution to 12 Bits for encider
#define AMT22_RES14		14 // Setting resolution to 14 Bits for encoder`


/** DEFINES FOR SPI**/
#define BAUDRATE        115200 //BAUDRATE for SPI communication between AMT22 Enocder and Due
#define CS_ENC_1		31 //Chip select pin for encoder 1
#define CS_ENC_2		33 //Chip select pin for encoder 2
//#define CS_IMU			22 //Chip select pin for IMU
//#define CS_INC			23 //Chip select pin for Inclinometer
#define SPI_MOSI		75 //MOSI pin on Due
#define SPI_MISO		74 //MISO pin on Due
#define SPI_SCLK		76 //Clock pin on Due	

#define RPi_PIN	??		

void setup(){
	/** SPI **/
	/** set pinmodes for SPI **/
    pinMode(SPI_SCLK, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(CS_ENC_1, OUTPUT);
    pinMode(CS_ENC_2, OUTPUT);	
    //pinMode(CS_IMU, OUTPUT);
    //pinMode(CS_INC, OUTPUT);
	
	/**	Set the CS line high which is the default inactive state(CS bar) **/
    digitalWrite(CS_ENC_1, HIGH);    
    digitalWrite(CS_ENC_2, HIGH);
    //digitalWrite(CS_IMU, HIGH);
    //digitalWrite(CS_INC, HIGH);

    pinMode(IMU_PIN,OUTPUT);
    digitalWrite(IMU_PIN,HIGH);
    pinMode(IMU_PIN, INPUT_PULLUP);
	
    pinMode(RPi_PIN, INPUT_PULLUP);
    RPi_FLAG = digitalRead(RPi_PIN);

    /*  **    IMU    **      */

    // Global IMU variables
    float euler_angles[3] = {0, 0, 0};        // Global angles from IMU
    float euler_velocities[3] = {0, 0, 0};    // Global velocities from IMU


    volatile char flag = 0;
    int counter = 0;

    // Global buffers to send to raspberry
    volatile long int counter = 0;			//Auxiliary counter for timer
    long int timerBuffer[BUFFER_SIZE];		//Time of data into buffer
    float IMUX_angle[BUFFER_SIZE];			//IMU X angle data into buffer
    float IMUX_velocity[BUFFER_SIZE];		//IMU X velocity data into buffer
    float angleBuffer_1[BUFFER_SIZE];			//Encoder angle data into buffer
    float angleBuffer_2[BUFFER_SIZE];			//Encoder angle data into buffer
    float velocityBuffer_1[BUFFER_SIZE];		//Encoder velocity data into buffer
    float velocityBuffer_2[BUFFER_SIZE];		//Encoder velocity data into buffer	
    float controlBuffer_1[BUFFER_SIZE];		//Control data into buffer
    float controlBuffer_2[BUFFER_SIZE];	



    /*  **    Init functions     **      */
    char InitIMU();
    void InitPrecessionMotor1();
    void InitPrecessionMotor2();
    void InitVelocityMotor1();
    void InitVelocityMotor2();

    //needed or not?
    void InitEncoder1();        
    void InitEncoder2();
    void InitClocks();

    /*  **    Sensor Reading functions     **      */
    int IMURead_eulerAngle(int); 		//	returns status of I2C communication
    int IMURead_eulerVelocities(int);   

    float EncoderRead(int);     //takes ENCODER_ID (1 or 2) and buffCount

}




void main(){

    Serial.begin(BAUD);
    Wire.begin();       //begin I2C transmission
    Wire.setclock(IMU_CLOCK);

	if (!IMU_init())
	{
		break;
		return 0;
	}

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
}

char IMU_init()
{
	//Serial.print("Starting Setup Transmission to IMU");
	Wire.beginTransmission(IMU_ID);

	Wire.write(0x42); //Prepare sensor to send data
	Wire.write(0x60); // Arduino sends tare with current orientation 

	int error = Wire.endTransmission(1);     //boolean

	if (error!= 0)
	{
		Serial.println("IMU_init: I2C communication error");
		return 0;
	}

	return 1;
}








char IMURead_eulerAngle(int buffCount) // Gets new data from the IMU
{
	union {
		unsigned char bytes[4];
		float angle;
	} temp_angle;

	Wire.beginTransmission(IMU_ID);
	Wire.write(0x42); // prepare sensor to recieve a TSS command
	Wire.write(0x01); // Read filtered, tared orientation(Euler Angles)
	Wire.write(0x43); // prepares sensor to send out data to master device

	int error = i2c_endTransmission(1);

	
	if (error != 0){
		Serial.println("IMURead_eulerAngle: I2C communication error");
		return 0;
	}

	Wire.requestFrom(IMU_ID, 12, 1);
	for (int i = 0; i < 3; i++)
	{
		temp_angle.angle = 0;
		for (int j = 3; j >= 0; j--){
			temp_angle.bytes[j] = i2c_read();
		}

		if (abs(temp_angle.angle) <= IMU_POS_MAX) {
			euler_angles[i] = -temp_angle.angle;	// Invert to match model
		}
	} 

	IMUX_angle[buffCount] = euler_angles[IMU_ROLL_AXIS];
	return 1;
}

	
char IMURead_eulerVelocities(int buffCount) // Gets new data from the IMU
{
	union {  //a data type that stores different data types
		unsigned char bytes[4];
		float angle_velocity;
	} temp_angle_velocity;

	//Serial.print("Starting Setup Transmission to IMU");
	Wire.beginTransmission(IMU_ID);
	//Serial.println("before Transmission to IMU");
	Wire.write_byte(0x42); //  prepares sensor to receive TSS command and any additional arguments.
	Wire.write_byte(0x26); // Read gyros (vector floatx3) corrected sensor data
	Wire.write_byte(0x43); // prepares sensor to send out data to master device
	
	//Serial.println("After transmissions");
	int error = i2c_endTransmission(1);
	//Serial.print("Setup Transmission to IMU finished");
	
	if (error == 5)
	{
		Serial.println("IMURead_eulerVelocities: I2C communication timeout");
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
