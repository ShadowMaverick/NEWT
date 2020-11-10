 // --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#define IMU_ID 0x77

int k = 0;

void setup()
{

  pinMode(21,OUTPUT);
  digitalWrite(21,HIGH);
  pinMode(21,INPUT_PULLUP);
  delay(2000);
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  //Serial.println("\nI2C Scanner");
  Wire.setClock(300000);

  
}


void loop()
{
  byte error, address;
  int nDevices;
  int i;
  int j;
  float angles[3];
  union {
    unsigned char bytes[4];
    float angle;
  } temp_angle;

  union {
    unsigned char bytes[4];
    float angle_velocity;
  } temp_angle_velocity;

  //Serial.println("Scanning...");

   // The i2c_scanner uses the return value of
   // the Write.endTransmisstion to see if
   // a device did acknowledge to the address.


   Wire.beginTransmission(IMU_ID);

   //Serial.println("Begin transmission...");
   Wire.write(0x42); //prepare sensor to recieve data
   Wire.write(0x01); //Euler angles 0x01 gyro vector (0x26)
   Wire.write(0x43); //prepare sensor to send data
   Wire.endTransmission();
   //Serial.println("Preparing device for data request...");
   
   Wire.requestFrom(IMU_ID,12,1);   // adress, 12 data bytes, 
   
   //Serial.println("Data request transmit...");

   for(i=0;i<3;i++){

      temp_angle_velocity.angle_velocity=0;
      
      for(j=3;j>=0;j--){
        
        temp_angle_velocity.bytes[j]=Wire.read();
        
      }

      angles[i]=temp_angle_velocity.angle_velocity;
      
      
    }

  Serial.print(angles[0],5); 
  Serial.print(" "); 
  Serial.print(angles[1],5);
  Serial.print(" "); 
  Serial.println(angles[2],5);
   

  //Serial.println("Finished");

  delay(1);           // wait 1 second for next scan

/*
  if(k > 1000)
  {
    Serial.println("Turn me off");
    delay(10000);
    k = 0;
  }
  k++;
*/
}
