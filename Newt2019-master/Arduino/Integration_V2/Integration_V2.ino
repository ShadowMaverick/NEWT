#include <math.h>
#include <SPI.h>
#include <Wire.h>
//#include <Timer.h> // ! Review: not used check and remove 

/******************** DEFINES *****************/

/** DEFINES FOR Serial Comunication with RPi **/
#define BAUD        250000UL		//Communication with RPi
#define BUFFER_SIZE 100         //Buffer size for local array to store log data
#define RPi_PIN  5             //Raspberry Pi interrupt pin   
//#define RPi_FLAG 0             // ! Review: not used check and remove 
//#define RPi_INTERRUPT 0 // pin for low/high signal from RPi // ! Review: not used check and remove 


/** DEFINES FOR IMU **/
#define IMU_ID 0x77
#define IMU_PIN 21              //SCL
#define IMU_MAX_ENCVAL 0x7FFFF  //524287 in dec ??
#define IMU_BUFFER 3
#define IMU_CLOCK 300000	      // I2C clock rate 300kHz
#define IMU_ROLL_AXIS 1			    // This is used to select the correct axis
#define IMU_POS_MAX	M_PI		    // pi
#define IMU_VEL_MAX 20

/** DEFINES FOR AMT33 ENCODER **/
#define AMT22_NOP       0x00    // SPI Command for No operation(the encoder does not care about the content of the first byte, we will define it as a NOP (no-operation))
#define AMT22_RESET     0x60    // SPI Command for Encoder reset
#define AMT22_ZERO      0x70    // SPI Command for setting zero in encoder
#define AMT22_RES12		12        // Setting resolution to 12 Bits for encider
#define RES14		14        // Setting resolution to 14 Bits for encoder
//The following delay might not be needed due to Arduino's inheret delay
#define AMT22_Delay   3         // a minimum delay of 3 microseconds between the first and the second 8 bit data trasnfer

/** DEFINES FOR SPI**/
//#define BAUDRATE        115200  //BAUDRATE for SPI communication between AMT22 Enocder and Due
#define CS_ENC_1		31          //Chip select pin for encoder 1
#define CS_ENC_2		33          //Chip select pin for encoder 2
//#define CS_IMU			22        //Chip select pin for IMU
//#define CS_INC			23        //Chip select pin for Inclinometer
#define SPI_MOSI		75        //MOSI pin on Due
#define SPI_MISO		74        //MISO pin on Due
#define SPI_SCLK		76        //Clock pin on Due	

/** DEFINES FOR I2C**/
//#define I2C_SDA			20        //SDA pin for wire
//#define I2C_SCL			21        //SCL pin for wire
//#define I2C2_SDA		70        //SDA pin for wire 1
//#define I2C2_SCL		09        //SCL pin for wire 1

/** DEFINES FOR Maxon**/
#define MAXON1_PWM	  2         //PWM Speed input pin for maxon motor controller
#define MAXON2_PWM	  3         //PWM Speed input pin for maxon motor controller
#define MAXON_EN      35        //Enable pin for Maxon
#define MAXON1_VEL    A0        //Analog input for Velocity motor 1/Flywheel 1 velocity 
#define MAXON2_VEL    A1        //Analog input for Velocity motor 2/Flywheel 2 velocity
#define MAXON_STOP   39         //Stop pin for both Maxon motor controllers
#define TERM_VELOCITY			5000  //maximum velocity for the flywheel

/** DEFINES FOR Precession motor**/

#define Prec_ENABLE 43
#define Prec1_DIR 47
#define Prec2_DIR 49
#define Prec_STOP 51
#define Prec1_PWM DAC0
#define Prec2_PWM DAC1
#define PREC_MAX_TORQUE 69.0f // from datasheet 

/** DEFINES FOR LQR Controller**/
 
/*Gains for gyro nr 1: */
#define LQR_ROLL_ANGLE_GAIN_GYRO_1 -1574.7
#define LQR_PRECESSION_ANGLE_GAIN_1_GYRO_1 0.2239
#define LQR_PRECESSION_ANGLE_GAIN_2_GYRO_1 146.3534
#define LQR_ROLL_RATE_GAIN_GYRO_1 -296.1532
#define LQR_PRECESSION_RATE_GAIN_1_GYRO_1 96.4304
#define LQR_PRECESSION_RATE_GAIN_2_GYRO_1 -0.4859

/*Gains for gyro n2 2: */
#define LQR_ROLL_ANGLE_GAIN_GYRO_2 1574.7
#define LQR_PRECESSION_ANGLE_GAIN_1_GYRO_2 146.3534
#define LQR_PRECESSION_ANGLE_GAIN_2_GYRO_2 0.2239 
#define LQR_ROLL_RATE_GAIN_GYRO_2 296.1532
#define LQR_PRECESSION_RATE_GAIN_1_GYRO_2 -0.4859
#define LQR_PRECESSION_RATE_GAIN_2_GYRO_2 96.4304 

#define IMU_ROLL_AXIS 1



/************** Global Variables ***************/

// Global IMU variables
float euler_angles[3] = {0, 0, 0};        // Global angles from IMU
float euler_velocities[3] = {0, 0, 0};    // Global velocities from IMU

volatile char flag = 0;


// Global buffers to send to raspberry
volatile long int counter = 0;			      //Auxiliary counter for timer
long int timerBuffer[BUFFER_SIZE];	      //Time of data into buffer
float IMUX_angle[BUFFER_SIZE];			      //IMU X angle data into buffer
float IMUX_velocity[BUFFER_SIZE];		      //IMU X velocity data into buffer
float angleBuffer_1[BUFFER_SIZE];	  		  //Encoder angle data into buffer
float angleBuffer_2[BUFFER_SIZE];		  	  //Encoder angle data into buffer
float velocityBuffer_1[BUFFER_SIZE];		  //Encoder velocity data into buffer
float velocityBuffer_2[BUFFER_SIZE];		  //Encoder velocity data into buffer	
float controlBuffer_1[BUFFER_SIZE];		    //Control data into buffer, from gyro nr 1
float controlBuffer_2[BUFFER_SIZE];	      //Control data into buffer, from gyro nr 2

// Global variable to denote error status 
int codeError;             //error code to specify type of error //! Review: Changed wrror status from buffer to a global var

// Global variables for LQR controller
double encfloat_1; //! Review changed float to double inorder to not loose encoder's resolution
double encfloat_2; //! Review changed float to double inorder to not loose encoder's resolution
double prec_velocity_1;
double prec_velocity_2;
/*float euler_angles[3] = {0, 0, 0};
float euler_velocities[3] = {0, 0, 0};*/
double oldEncfloat_1 = 0; //! Review changed float to double inorder to not loose encoder's resolution
double oldEncfloat_2 = 0; //! Review changed float to double inorder to not loose encoder's resolution
unsigned long enctime_1;	    //Encoder angle data into buffer // ! Review: Changed time from buffer to global variable
unsigned long enctime_2;		  //Encoder angle data into buffer 
unsigned long oldenctime_1;	    //Encoder angle data into buffer 
unsigned long oldenctime_2;		  //Encoder angle data into buffer 


//prec_velocity_1 = (encfloat_1 - oldEncfloat_1) / deltaClock; // ! Review: moved to encoder velocit fucntion (remove after revie) 
//prec_velocity_2 = (encfloat_2 - oldEncfloat_2) / deltaClock;

/** Glabal var for timers **/
volatile boolean Timer_Flag = false;

void tc_setup() {

  PMC->PMC_PCER0 |= PMC_PCER0_PID29;                      // TC2 power ON : Timer Counter 0 channel 2 IS TC2
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // MCK/2, clk on rising edge --> F = 42 MHz
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;       // UP mode with automatic trigger on RC Compare
                      

  TC0->TC_CHANNEL[2].TC_RC = 5250;  //<*********************  Frequency = (Mck/2)/TC_RC  Hz = 8000 Hz

  TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;    // Interrupt enable on RC compare match
 // TC0->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS; // IDR = interrupt disable register
  NVIC_EnableIRQ(TC2_IRQn);      // Enable TC2 interrupts

  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Reset TC2 counter and enable

}

bool initgyro(int lp)
{
//bool lp
//lp = 1  
//mapping velocity 100, 5000 to dutycyle(26 - 10%, 236 - 90%) linearly
int Maxonduty = 26+((TERM_VELOCITY-100)*0.041633);//mapping a,b to c,d; Y = (X-A)/(B-A) * (D-C) + C

digitalWrite(MAXON_EN, LOW); //enables maxon motor controllers

analogWrite(MAXON1_PWM, Maxonduty); //write 90% PWM for maxon 1

analogWrite(MAXON2_PWM, Maxonduty); //write 90% PWM for maxon 2

while (lp)
 {
 
  int Flywheel1_Vel = analogRead(MAXON1_VEL);

  int Flywheel2_Vel = analogRead(MAXON2_VEL);

  // if maxon 1 and maxon 2 speed reach max Velocity set lp to 0
  if (Flywheel1_Vel && Flywheel2_Vel >= 4090) //Change 4090 to match maxon motor driver setup
    {lp = 0;} 
 }

}


void setup(){


  
  /** set pinmodes for SPI IMU**/
    pinMode(IMU_PIN, OUTPUT);
    digitalWrite(IMU_PIN, HIGH);
    pinMode(IMU_PIN, INPUT_PULLUP);

  /** set pinmodes for Rpi**/
    pinMode(RPi_PIN, INPUT);
    //RPi_FLAG = digitalRead(RPi_PIN); in mainloop
    //pinMode(RPi_INTERRUPT, INPUT);

	/** set pinmodes for SPI devices - Encoder**/
    pinMode(SPI_SCLK, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(CS_ENC_1, OUTPUT);
    pinMode(CS_ENC_2, OUTPUT);	
    //pinMode(CS_IMU, OUTPUT);
    //pinMode(CS_INC, OUTPUT);
    digitalWrite(CS_ENC_1, HIGH); // Set the CS line high which is the default inactive state(CS bar)
    digitalWrite(CS_ENC_2, HIGH); // Set the CS line high which is the default inactive state(CS bar)
    //digitalWrite(CS_IMU, HIGH);
    //digitalWrite(CS_INC, HIGH);

	/**set pinmodes for Maxon controller**/
    pinMode(MAXON_EN, OUTPUT);
    digitalWrite(MAXON_EN, HIGH); //Sets the line high as Maxon control enables at 'low'
 
    pinMode(MAXON1_VEL, INPUT);
    pinMode(MAXON2_VEL, INPUT);
    pinMode(MAXON1_PWM, OUTPUT);
    pinMode(MAXON2_PWM, OUTPUT);
    pinMode(MAXON_STOP, OUTPUT);
    analogReadResolution(12);     //change resolution of maxon speed input

	/**set pinmodes for Precession Motor**/
    pinMode(Prec_ENABLE, OUTPUT); //Sets Prec_ENABLE pin 43 to output
    pinMode(Prec_STOP, OUTPUT);   //Sets Prec_STOP pin 51 to output
    pinMode(Prec1_DIR, OUTPUT);   //Sets Prec1_DIR pin 47 to output
    pinMode(Prec1_PWM, OUTPUT);   //Sets Prec1_PWM pin DAC0 to output
    pinMode(Prec2_DIR, OUTPUT);   //Sets Prec2_DIR pin 49 to output
    pinMode(Prec2_PWM, OUTPUT);   //Sets Prec2_PWM pin DAC1 to output
    analogWriteResolution(12);    //change resolution of precession motor speed output

	/**intialize Precession Motor**/    
    digitalWrite(Prec_ENABLE,HIGH); //enable both precession motors
    digitalWrite(Prec_STOP,LOW);    //stop both precession motors
    //initialize precession motor 1:  
    digitalWrite(Prec1_DIR,LOW);    //Sets Precession motor 1 direction to CW
    //initialize precession motor 2:
    digitalWrite(Prec2_DIR,LOW);    //Sets Precession motor 2 direction to CW

  /** Begin commmunications **/

    Serial.begin(BAUD);               //Begin serial comunication with Rpi

    Wire.begin();                     //begin I2C transmission
    Wire.setClock(IMU_CLOCK);

    //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
    //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
    SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
    //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
    //SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
    SPI.begin();
        
    tc_setup(); //Setup timers


  /*Test Rig intialisation*/ 

    // Check encoders availability

    // Check IMU availability - comment: maynot be needed here as IMU availability is checked each turn before transmission
        
    // Set zero position for Encoder  - comment: maynot be needed as Encoder reatains zero position even when turned off

    // intialize gyroscopes and run them to max velocity(TERM_VELOCITY)
    initgyro(1);

    //while (the user has not agreed to beginning of control loop)
    {
    //reset buffer - check
    //don't do anything
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

  int error = Wire.endTransmission(1); //! Review: can i2c_endTransmission be replaced by Wire.endTransmission
  
  if (error != 0){
    Serial.println("IMURead_eulerAngle: I2C communication error");
    return 0;
  }

  Wire.requestFrom(IMU_ID, 12, 1);
  for (int i = 0; i < 3; i++)
  {
    temp_angle.angle = 0;
    for (int j = 3; j >= 0; j--){
      temp_angle.bytes[j] = Wire.read(); //! Review: Can i2c_read be replaced by Wire.read?
    }

    if (abs(temp_angle.angle) <= IMU_POS_MAX) {
      euler_angles[i] = -temp_angle.angle;  // Invert to match model
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
  Wire.write(0x42); //  prepares sensor to receive TSS command and any additional arguments. //! Review: Changed from Wire.write_byte to Wire.write
  Wire.write(0x26); // Read gyros (vector floatx3) corrected sensor data //! Review: Changed from Wire.write_byte to Wire.write
  Wire.write(0x43); // prepares sensor to send out data to master device //! Review: Changed from Wire.write_byte to Wire.write
  
  //Serial.println("After transmissions");
  int error = Wire.endTransmission(1); //! Review: can i2c_endTransmission be replaced by Wire.endTransmission
  //Serial.print("Setup Transmission to IMU finished");
  
  if (error == 5)
  {
    Serial.println("IMURead_eulerVelocities: I2C communication timeout");
    return 0;
  }

  Wire.requestFrom(IMU_ID, 12, 0, 0, 1); // (address, quantity, ) //! Review: changed from i2c_requestFrom to Wire.requestFrom 
  for (int i = 0; i < 3; i++)
  {
    temp_angle_velocity.angle_velocity = 0;
    for (int j = 3; j >= 0; j--)
    {
      temp_angle_velocity.bytes[j] = Wire.read(); //! Review: Can i2c_read be replaced by Wire.read?
    }

    if (temp_angle_velocity.angle_velocity <= IMU_VEL_MAX) {
      euler_velocities[i] = -temp_angle_velocity.angle_velocity;  // Invert to match model
    }
  }
  
  IMUX_velocity[buffCount] = euler_velocities[IMU_ROLL_AXIS];
  return 1;
}

void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}


int ControlLQR(int buffCount) // Calculates and sends control values to the precession motor
{
  float control_1 =-(euler_angles[IMU_ROLL_AXIS]*LQR_ROLL_ANGLE_GAIN_GYRO_1+encfloat_1*LQR_PRECESSION_ANGLE_GAIN_1_GYRO_1+encfloat_2*LQR_PRECESSION_ANGLE_GAIN_2_GYRO_1+euler_velocities[IMU_ROLL_AXIS]*LQR_ROLL_RATE_GAIN_GYRO_1+prec_velocity_1*LQR_PRECESSION_RATE_GAIN_1_GYRO_1+prec_velocity_2*LQR_PRECESSION_RATE_GAIN_2_GYRO_1);

  float control_2=-(euler_angles[IMU_ROLL_AXIS]*LQR_ROLL_ANGLE_GAIN_GYRO_2+encfloat_1*LQR_PRECESSION_ANGLE_GAIN_1_GYRO_2+encfloat_2*LQR_PRECESSION_ANGLE_GAIN_2_GYRO_2+euler_velocities[IMU_ROLL_AXIS]*LQR_ROLL_RATE_GAIN_GYRO_2+prec_velocity_1*LQR_PRECESSION_RATE_GAIN_1_GYRO_2+prec_velocity_2*LQR_PRECESSION_RATE_GAIN_2_GYRO_2);

  controlBuffer_1[buffCount] = control_1; /*global vector that saves control values*/
  controlBuffer_2[buffCount] = control_2;

  if (control_1 < 0) { // Motor direction
   digitalWrite(47, LOW);   //*Direction CW- clockwise*/
  }
  else {
    digitalWrite(47, HIGH); // digitalWrite(5, LOW);   /*Direction CCW-counter clockwise*/
  }

  if (control_2 < 0) { // Motor direction
   digitalWrite(49, LOW);   //*Direction CW- clockwise*/
  }
  else {
    digitalWrite(49, HIGH); // digitalWrite(5, LOW);   /*Direction CCW-counter clockwise*/
  }   //eventually change to one if- statement//

  control_1 = min(abs(control_1), PREC_MAX_TORQUE); /*limits the control output to 70*/
  control_2 = min(abs(control_2), PREC_MAX_TORQUE);
  int pwm_1 = round(4095 * (control_1/PREC_MAX_TORQUE)); 
  int pwm_2 = round(4095 * (control_2/PREC_MAX_TORQUE));
  
  analogWrite(Prec1_PWM, pwm_1);
  analogWrite(Prec2_PWM, pwm_1);
  
  return 1;

}

uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(AMT22_Delay);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}


uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(AMT22_Delay);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == AMT22_RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}


int readPrecessionangle(uint8_t encoder, int buffCount)
{
  //create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;
  //lets return success as 1 in status
  uint8_t status;

  //set attemps counter at 0 so we can try again if we get bad position    
  attempts = 0;

  //set status to success at default
  status = 1;
  //this function gets the encoder position and returns it as a uint16_t
  //send the function either res12 or res14 for your encoders resolution
  encoderPosition = getPositionSPI(encoder, RES14); 

  //if the position returned was 0xFFFF we know that there was an error calculating the checksum
  //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
  while (encoderPosition == 0xFFFF && ++attempts < 3)
  {
    encoderPosition = getPositionSPI(encoder, RES14); //try again
  }

  if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
  {
  status = 0 ;
  }
  else if (encoder == CS_ENC_1)//position was good, print to serial stream
  {
 
  //! review the else if loop
  oldEncfloat_1 = encfloat_1; //moves current enc value to old value before reading a new value
  oldenctime_1 = enctime_1; //moves current enc time to old time before reading a new time
  angleBuffer_1[buffCount] = encoderPosition;
  encfloat_1 = encoderPosition * 0.0003835; // 0.0003835 - convers 14Bit value to radians
  enctime_1 = micros(); //! Review: changed from buffer to global variable
  }
  else
  {
  //!revew the else loop
  oldEncfloat_1 = encfloat_1; //moves current enc value to old value before reading a new value
  oldenctime_1 = enctime_1; //moves current enc time to old time before reading a new time
  angleBuffer_2[buffCount] = encoderPosition;
  encfloat_2 = encoderPosition * 0.0003835; // 0.0003835 - convers 14Bit value to radians
  enctime_2 = micros(); //! Review: changed from buffer to global variable
  }
  
  return status;
}

int readPrecessionvel(int buffCount) //! Review function and confirm
{
 //! Discuss: oldencfloat does not exist for the first loop cycle; needs a mitigation plan. 
 prec_velocity_1 = (encfloat_1 - oldEncfloat_1) / ((enctime_1 - oldenctime_1)*1000000); //1000000 converts microsecond to second 
 prec_velocity_2 = (encfloat_2 - oldEncfloat_2) / ((enctime_2 - oldenctime_2)*1000000); //1000000 converts microsecond to second
 float velocityBuffer_1[buffCount] = prec_velocity_1;
 float velocityBuffer_2[buffCount] = prec_velocity_2;
 return 1;
}


void loop()
{
  if (!IMU_init())
	{
	  Serial.println("IMU_init() did not run");
		//return 0; // it will never return anything because the break is before
	}
	

	while (Timer_Flag)
	{
  	int buffCount = 1;  
 		
		while (flag == 0)
    
    {
		
      if (!IMURead_eulerAngle(buffCount)) { 
        codeError = 1;
        break;
      }
      if (!IMURead_eulerVelocities(buffCount)) {
        codeError = 2;
        break;
      }
      if (!readPrecessionangle(CS_ENC_1, buffCount)) {
        codeError = 3;
        break;
      }  
      if (!readPrecessionangle(CS_ENC_2, buffCount)) {
        codeError = 4;
        break;
      }
      if (!readPrecessionvel(buffCount)) {
        codeError = 5;
        break;
      }
      if (!ControlLQR(buffCount)) {
        codeError= 5;
        break;
      }
      if (buffCount == BUFFER_SIZE){ // ! review: added condition to set flag when buffer limit is met
        flag = 1;
        break;
      }
      if (digitalRead(RPi_PIN) == HIGH){// ! review: added condition to set flag when RPi interupt is set
        flag = 2; 
        break;
      }
      buffCount = buffCount+1;
    }
    // If flag is 1 call logDataToRaspberry() //! review: changes made to cmplete writing buffer logic
    if (flag == 1){
      logDataToRaspberry();
    }
    // If flag is 2 call stop function and break //! review: changes made to call soft stop function
    if (flag ==2){
      softstop();
    }
}



/*char IMU_init()
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
}*/

/*char IMURead_eulerAngle(int buffCount) // Gets new data from the IMU
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
}*/

	
/*char IMURead_eulerVelocities(int buffCount) // Gets new data from the IMU
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
	int error = Wire.endTransmission(1); //! Review: can i2c_endTransmission be replaced by Wire.endTransmission
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
			temp_angle_velocity.bytes[j] = Wire.read(); //! Review: Can i2c_read be replaced by Wire.read?
		}

		if (temp_angle_velocity.angle_velocity <= IMU_VEL_MAX) {
			euler_velocities[i] = -temp_angle_velocity.angle_velocity;	// Invert to match model
		}
	}
	
	IMUX_velocity[buffCount] = euler_velocities[IMU_ROLL_AXIS];
	return 1;
}*/


/*bool initgyro(lp)
{
//bool lp
//lp = 1	
//mapping velocity 100, 5000 to dutycyle(26 - 10%, 236 - 90%) linearly
int Maxonduty = 26+((TERM_VELOCITY-100)*0.041633);//mapping a,b to c,d; Y = (X-A)/(B-A) * (D-C) + C

digitalWrite(MAXON_EN, LOW); //enables maxon motor controllers

analogWrite(MAXON1_PWM, Maxonduty); //write 90% PWM for maxon 1

analogWrite(MAXON2_PWM, Maxonduty); //write 90% PWM for maxon 2

while lp
 {
 
  int Flywheel1_Vel = analogRead(MAXON1_VEL);

  int Flywheel2_Vel = analogRead(MAXON2_VEL);

  // if maxon 1 and maxon 2 speed reach max Velocity set lp to 0
  if (Flywheel1_Vel && Flywheel2_Vel >= 4090) //Change 4090 to match maxon motor driver setup
    {lp = 0;} 
 }

}*/


/*int readPrecessionvel(int buffCount) //! Review function and confirm
{
 //! Discuss: oldencfloat does not exist for the first loop cycle; needs a mitigation plan. 
 prec_velocity_1 = (encfloat_1 - oldEncfloat_1) / ((enctime_1 - oldenctime_1)*1000000); //1000000 converts microsecond to second 
 prec_velocity_2 = (encfloat_2 - oldEncfloat_2) / ((enctime_2 - oldenctime_2)*1000000); //1000000 converts microsecond to second
 float velocityBuffer_1[buffCount] = prec_velocity_1;
 float velocityBuffer_2[buffCount] = prec_velocity_2;
 return 1;
}*/

/*int readPrecessionangle(uint8_t encoder, int buffCount)
{
  //create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;
  //lets return success as 1 in status
  uint8_t status;

  //set attemps counter at 0 so we can try again if we get bad position    
  attempts = 0;

  //set status to success at default
  status = 1;
  //this function gets the encoder position and returns it as a uint16_t
  //send the function either res12 or res14 for your encoders resolution
  encoderPosition = getPositionSPI(encoder, RES14); 

  //if the position returned was 0xFFFF we know that there was an error calculating the checksum
  //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
  while (encoderPosition == 0xFFFF && ++attempts < 3)
  {
    encoderPosition = getPositionSPI(encoder, RES14); //try again
  }

  if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
  {
  status = 0 
  }
  else if (encoder == CS_ENC_1)//position was good, print to serial stream
  {
 
  //! review the else if loop
  oldEncfloat_1 = encfloat_1; //moves current enc value to old value before reading a new value
  oldenctime_1 = enctime_1; //moves current enc time to old time before reading a new time
  angleBuffer_1[buffCount] = encoderPosition;
  encfloat_1 = encoderPosition * 0.0003835; // 0.0003835 - convers 14Bit value to radians
  enctime_1 = micros(); //! Review: changed from buffer to global variable
  }
  else
  {
  //!revew the else loop
  oldEncfloat_1 = encfloat_1; //moves current enc value to old value before reading a new value
  oldenctime_1 = enctime_1; //moves current enc time to old time before reading a new time
  angleBuffer_2[buffCount] = encoderPosition;
  encfloat_2 = encoderPosition * 0.0003835; // 0.0003835 - convers 14Bit value to radians
  enctime_2 = micros(); //! Review: changed from buffer to global variable
  }
	
  return status;
}*/

/*uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(AMT22_Delay);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == AMT_RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}*/


/*uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(AMT22_Delay);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}*/

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
/*void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}


int ControlLQR(int buffCount) // Calculates and sends control values to the precession motor
{
	float control_1 =-(euler_angles[IMU_ROLL_AXIS]*LQR_ROLL_ANGLE_GAIN_GYRO_1+encfloat_1*LQR_PRECESSION_ANGLE_GAIN_1_GYRO_1+encfloat_2*LQR_PRECESSION_ANGLE_GAIN_2_GYRO_1+euler_velocities[IMU_ROLL_AXIS]*LQR_ROLL_RATE_GAIN_GYRO_1+prec_velocity_1*LQR_PRECESSION_RATE_GAIN_1_GYRO_1+prec_velocity_2*LQR_PRECESSION_RATE_GAIN_2_GYRO_1);

	float control_2=-(euler_angles[IMU_ROLL_AXIS]*LQR_ROLL_ANGLE_GAIN_GYRO_2+encfloat_1*LQR_PRECESSION_ANGLE_GAIN_1_GYRO_2+encfloat_2*LQR_PRECESSION_ANGLE_GAIN_2_GYRO_2+euler_velocities[IMU_ROLL_AXIS]*LQR_ROLL_RATE_GAIN_GYRO_2+prec_velocity_1*LQR_PRECESSION_RATE_GAIN_1_GYRO_2+prec_velocity_2*LQR_PRECESSION_RATE_GAIN_2_GYRO_2);

	controlBuffer_1[buffCount] = control_1; /*global vector that saves control values*/
	controlBuffer_2[buffCount] = control_2;

	if (control_1 < 0) { // Motor direction
	 digitalWrite(47, LOW);   //*Direction CW- clockwise*/
	}
	else {
		digitalWrite(47, HIGH);	// digitalWrite(5, LOW);   /*Direction CCW-counter clockwise*/
	}

	if (control_2 < 0) { // Motor direction
	 digitalWrite(49, LOW);   //*Direction CW- clockwise*/
	}
	else {
		digitalWrite(49, HIGH);	// digitalWrite(5, LOW);   /*Direction CCW-counter clockwise*/
	}   //eventually change to one if- statement//

	control_1 = min(abs(control_1), PREC_MAX_TORQUE); /*limits the control output to 70*/
	control_2 = min(abs(control_2), PREC_MAX_TORQUE);
	int pwm_1 = round(4095 * (control_1/PREC_MAX_TORQUE)); 
	int pwm_2 = round(4095 * (control_2/PREC_MAX_TORQUE));
	
	analogWrite(Prec1_PWM, pwm_1);
	analogWrite(Prec2_PWM, pwm_1);
	
  return 1;

}*/


void logDataToRaspberry(){
	Serial.write((uint8_t*)timerBuffer, sizeof(*timerBuffer) * BUFFER_SIZE);
   	Serial.write(", ");
	Serial.write((uint8_t*)IMUX_angle, sizeof(*IMUX_angle) * BUFFER_SIZE);
    	Serial.write(", ");
	Serial.write((uint8_t*)IMUX_velocity, sizeof(*IMUX_velocity) * BUFFER_SIZE);
	Serial.write(", ");
    	Serial.write((uint8_t*)angleBuffer_1, sizeof(*angleBuffer_1) * BUFFER_SIZE);
    	Serial.write(", ");
    	Serial.write((uint8_t*)angleBuffer_2, sizeof(*angleBuffer_2) * BUFFER_SIZE);
	Serial.write(", ");
    	Serial.write((uint8_t*)velocityBuffer_1, sizeof(*velocityBuffer_1) * BUFFER_SIZE);
	Serial.write(", ");
    	Serial.write((uint8_t*)velocityBuffer_2, sizeof(*velocityBuffer_2) * BUFFER_SIZE);
    	Serial.write(", ");
	Serial.write((uint8_t*)controlBuffer_1, sizeof(*controlBuffer_1) * BUFFER_SIZE);
    	Serial.write(", ");
    	Serial.write((uint8_t*)controlBuffer_2, sizeof(*controlBuffer_2) * BUFFER_SIZE);
   	Serial.write("\n");
	
	resetDataToRaspbrryPi();  //! Review: added the reset function as planned
}

void resetDataToRaspberry(){

	/* Keep the last value 
	long int temp_timer = timerBuffer[buffCount];
	float temp_IMUX_angle = IMUX_angle[buffCount];			    
   	float temp_IMUX_velocity = IMUX_velocity[buffCount];		     
	float temp_angleBuffer_1 = angleBuffer_1[buffCount];	  		
	float temp_angleBuffer_2 = angleBuffer_2[buffCount];		  	 
	unsigned long temp_angletimeBuffer_1 = angletimeBuffer_1[buffCount];	 
	unsigned long temp_angletimeBuffer_2 = angletimeBuffer_2[buffCount];		
	float temp_velocityBuffer_1 = velocityBuffer_1[buffCount];		
	float temp_velocityBuffer_2 = velocityBuffer_2[buffCount];			
	float temp_controlBuffer_1 = controlBuffer_1[buffCount];		   
	float temp_controlBuffer_2 = controlBuffer_2[buffCount];	   
	int temp_codeError = codeError[buffCount];     
	*/
	
	counter=1;    //Auxiliary counter for timer
	buffCount = 1; //reset index when data is transmitted
	memset(timerBuffer, 0, sizeof(*timerBuffer) * BUFFER_SIZE);			//Time of data into buffer
	memset(IMUX_angle, 0, sizeof(*IMUX_angle) * BUFFER_SIZE);			//IMU X angle data into buffer
	memset(IMUX_velocity, 0, sizeof(*IMUX_velocity) * BUFFER_SIZE);		//IMU X velocity data into buffer
	memset(angleBuffer_1, 0, sizeof(*angleBuffer_1) * BUFFER_SIZE);		//Encoder angle data into buffer
	memset(angleBuffer_2, 0, sizeof(*angleBuffer_2) * BUFFER_SIZE);	
  memset(velocityBuffer_1, 0, sizeof(*velocityBuffer_1) * BUFFER_SIZE);	//Encoder velocity data into buffer
  memset(velocityBuffer_2, 0, sizeof(*velocityBuffer_2) * BUFFER_SIZE);
	memset(controlBuffer_1, 0, sizeof(*controlBuffer_1) * BUFFER_SIZE);			//Control input u, into buffer
  memset(controlBuffer_2, 0, sizeof(*controlBuffer_2) * BUFFER_SIZE);
	
	/*
	timerBuffer[0] = temp_timer;
	IMUX_angle[0] = temp_IMUX_angle;			    
   	IMUX_velocity[0] = temp_IMUX_velocity ;		     
	angleBuffer_1[0] = temp_angleBuffer_1;	  		
	angleBuffer_2[0] = temp_angleBuffer_2 ;		  	 
	angletimeBuffer_1[0] = temp_angletimeBuffer_1;	 
	angletimeBuffer_2[0] = temp_angletimeBuffer_2;		
	velocityBuffer_1[0] = temp_velocityBuffer_1;		
	velocityBuffer_2[0] = temp_velocityBuffer_2;			
	controlBuffer_1[0] = temp_controlBuffer_1;		   
	controlBuffer_2[0] = temp_controlBuffer_2;	   
	codeError[0] = temp_codeError;  	
	*/
}

/*************  Timer Counter 0 Channel 2 to generate interrupts every 1/8000 second  ************/

/*void tc_setup() {

  PMC->PMC_PCER0 |= PMC_PCER0_PID29;                      // TC2 power ON : Timer Counter 0 channel 2 IS TC2
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // MCK/2, clk on rising edge --> F = 42 MHz
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;       // UP mode with automatic trigger on RC Compare
                      

  TC0->TC_CHANNEL[2].TC_RC = 5250;  //<*********************  Frequency = (Mck/2)/TC_RC  Hz = 8000 Hz

  TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;    // Interrupt enable on RC compare match
 // TC0->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS; // IDR = interrupt disable register
  NVIC_EnableIRQ(TC2_IRQn);      // Enable TC2 interrupts

  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Reset TC2 counter and enable

}*/

void TC2_Handler () {
  
  TC0->TC_CHANNEL[2].TC_SR;  // Clear status register

  Timer_Flag = true;
}


void softstop()
{
 digitalWrite(Prec_ENABLE, HIGH);//Disable Precession
 
 digitalWrite(MAXON_EN, HIGH);//Disable Maxon//Disable Maxon

 while(1)
 {}
}


/** OPTIONAL FUNCTIONS**/

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */


void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up

}
