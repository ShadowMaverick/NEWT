
/*InitPrecessionMotor
precessToZero*/

#define Prec_ENABLE 43
#define Prec1_DIR 47
#define Prec2_DIR 49
#define Prec_STOP 51
#define Prec1_PWM DAC0
#define Prec2_PWM DAC1


#define PREC_MAX_TORQUE 69.0f // from datasheet 
analogWriteResolution(12); //change resolution of precession motor speed output

void InitPrecessionMotor();


void InitPrecessionMotor()
{ 
  pinMode(Prec_ENABLE,OUTPUT); //enable both precession motors
  digitalWrite(Prec_ENABLE,HIGH); //enable is on
  pinMode(Prec_STOP,OUTPUT); //enable both precession motors
  digitalWrite(Prec_STOP,LOW); //enable is on
  
  //initialize precession motor 1:  
  pinMode(Prec1_DIR,OUTPUT); //defines which pin for direction of motor nr 1
  digitalWrite(Prec1_DIR,LOW); //Direction CW
  pinMode(Prec1_PWM, OUTPUT);   // define PWM output
  
  //initialize precession motor 2:
  pinMode(Prec2_DIR,OUTPUT); //defines which pin for direction of motor nr 2
  digitalWrite(Prec2_DIR,LOW); //Direction CW
  pinMode(Prec2_PWM, OUTPUT);   // define PWM output

  //Q: Should we add the rest of the lines here, also where do we define DAC1 and DAC2?
}
