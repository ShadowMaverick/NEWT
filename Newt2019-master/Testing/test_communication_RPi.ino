#define RPi_FLAG 0
#define RPi_INTERRUPT 0 // pin for low/high signal from RPi
#define BUFFER_SIZE 10
#define RECBUFFER_LEN 10

int timerBuffer[BUFFER_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
int controlBuffer_2[BUFFER_SIZE] = {11, 22, 33, 44, 55, 66, 77, 88, 99, 110};

void setup(){
    //pinMode(RPi_INTERRUPT, INPUT);
    resetDataToRaspberry(i);
    Serial.begin(250000);

    
}

void loop(){
  
  
   // Wait for message from the pi before we continue
 /* uint8_t recBuffer[RECBUFFER_LEN] = "";
  uint8_t recLen = 0, last = 'A';
  Serial.println("Please enter command");
  while (last != '\n' && recLen < RECBUFFER_LEN)
  {
    last = Serial.read();
    recBuffer[recLen] = last;
    recLen++;
  }
  Serial.write(recBuffer, recLen);
  */
  
    // RPi_FLAG = digitalRead(RPi_INTERRUPT);      //place in main, before while-loop
    
    for(int i =1; i < 5; i++ ){
        logDataToRaspberry(i);   
    }
}


void logDataToRaspberry(int i){

    Serial.write(timerBuffer[i]);
    Serial.write(", ");
    Serial.write(controlBuffer_2[i]);
    Serial.write("\n");
}


void resetDataToRaspberry(int NewVal){                              //Auxiliary counter for timer
  memset(timerBuffer, NewVal, sizeof(*timerBuffer) * BUFFER_SIZE);      //Time of data into buffer
  memset(controlBuffer_2, NewVal, sizeof(*controlBuffer_2) * BUFFER_SIZE);  
}
