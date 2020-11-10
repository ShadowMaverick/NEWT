//#define RPi_FLAG 0
//#define RPi_INTERRUPT 0 // pin for low/high signal from RPi
const int BUFFER_SIZE = 10;
int timerBuffer[BUFFER_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1, 1 ,1};
int controlBuffer_2[BUFFER_SIZE];
    
void setup(){
    //pinMode(RPi_INTERRUPT, INPUT);
    Serial.begin(9600);

}

void loop(){
    // RPi_FLAG = digitalRead(RPi_INTERRUPT);      //place in main, before while-loop
    for(int i =1; i < 5; i++ ){
        logDataToRaspberry();
        resetDataToRaspberry(i);
    }
}
void logDataToRaspberry(){
    Serial.write((uint8_t*)timerBuffer, sizeof(*timerBuffer) * BUFFER_SIZE);
    Serial.print(", ");
    Serial.write((uint8_t*)controlBuffer_2, sizeof(*controlBuffer_2) * BUFFER_SIZE);
    Serial.print("\n");
    Serial.print("log data");
}

void resetDataToRaspberry(int NewVal){                              //Auxiliary counter for timer
    memset(timerBuffer, NewVal, sizeof(*timerBuffer) * BUFFER_SIZE);      //Time of data into buffer
    memset(controlBuffer_2, NewVal, sizeof(*controlBuffer_2) * BUFFER_SIZE); 
    Serial.print("reset data"); 
}
