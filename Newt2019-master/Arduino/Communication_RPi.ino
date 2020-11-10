#define RPi_FLAG 0
#define RPi_INTERRUPT 0 // pin for low/high signal from RPi

void InitRPi(){
    pinMode(RPi_INTERRUPT, INPUT);
    
}

RPi_FLAG = digitalRead(RPi_INTERRUPT);      //place in main, before while-loop

void logDataToRaspberry(){
    for (int i = 0; i < BUFFER_SIZE; i ++){
      Serial.print(timerBuffer[i]);
      Serial.print(", ");
      Serial.print(IMUX_angle[i]);
      Serial.print(", ");
      Serial.print(IMUX_velocity[i]);
      Serial.print(", ");
      Serial.print(angleBuffer_1[i]);
      Serial.print(", ");
      Serial.print(angleBuffer_2[i];
      Serial.print(", ");
      Serial.print(velocityBuffer_1[i]);
      Serial.print(", ");
      Serial.print(velocityBuffer_2[i]);
      Serial.print(", ");
      Serial.print(controlBuffer_1[i]);
      Serial.print(", ");
      Serial.print(controlBuffer_2[i]);
      Serial.print("\n");
      }
}

void resetDataToRaspberry(){
	/* 
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
	codeError[0] = temp_codeError;  */
}
