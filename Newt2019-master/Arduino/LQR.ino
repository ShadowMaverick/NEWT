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

float controlBuffer_1[BUFFER_SIZE];		//Control data into buffer, from gyro nr 1
float controlBuffer_2[BUFFER_SIZE];		//Control data into buffer, from gyro nr 2

float encfloat_1;
float encfloat_2;
float prec_velocity_1;
float prec_velocity_2;
float euler_angles[3] = {0, 0, 0};
float euler_velocities[3] = {0, 0, 0};
float oldEncfloat_1 = encfloat_1;
float oldEncfloat_2 = encfloat_2;
prec_velocity_1 = (encfloat_1 - oldEncfloat_1) / deltaClock;
prec_velocity_2 = (encfloat_2 - oldEncfloat_2) / deltaClock;




void ControlLQR(int buffCount) // Calculates and sends control values to the precession motor
{
	float control_1 = -(euler_angles[IMU_ROLL_AXIS] * LQR_ROLL_ANGLE_GAIN_GYRO_1+
		encfloat_1 * LQR_PRECESSION_ANGLE_GAIN_1_GYRO_1 + encfloat_2* LQR_PRECESSION_ANGLE_GAIN_2_GYRO_1 +
		euler_velocities[IMU_ROLL_AXIS] * LQR_ROLL_RATE_GAIN_GYRO_1
	prec_velocity_1 * LQR_PRECESSION_RATE_GAIN_1_GYRO_1§ + prec_velocity_2 * LQR_PRECESSION_RATE_GAIN_2_GYRO_1);

	float control_2= -(euler_angles[IMU_ROLL_AXIS] * LQR_ROLL_ANGLE_GAIN_GYRO_2+
		encfloat_1 * LQR_PRECESSION_ANGLE_GAIN_1_GYRO_2 + encfloat_2* LQR_PRECESSION_ANGLE_GAIN_2_GYRO_2 +
		euler_velocities[IMU_ROLL_AXIS] * LQR_ROLL_RATE_GAIN_GYRO_2
	prec_velocity_1 * LQR_PRECESSION_RATE_GAIN_1_GYRO_2 + prec_velocity_2 * LQR_PRECESSION_RATE_GAIN_2_GYRO_2);

	controlBuffer_1[buffCount] = control_1; /*global vector that saves control values*/
	controlBuffer_2[buffCount] = control_2;

	if (control_1 < 0) { // Motor direction
	 digitalWrite(47, LOW);   //*Drirection CW- clockwise*/
	}
	else {
		digitalWrite(47, HIGH);	// digitalWrite(5, LOW);   /*Drirection CCW-counter clockwise*/
	}

	if (control_2 < 0) { // Motor direction
	 digitalWrite(49, LOW);   //*Drirection CW- clockwise*/
	}
	else {
		digitalWrite(49, HIGH);	// digitalWrite(5, LOW);   /*Drirection CCW-counter clockwise*/
	}   //eventually change to one if- statement//

	control_1 = min(abs(control_1), PREC_MAX_TORQUE); /*limits the control output to 70*/
	control_2 = min(abs(control_2), PREC_MAX_TORQUE);
	int pwm_1 = round(4095 * (control_1/PREC_MAX_TORQUE)); 
	int pwm_2 = round(4095 * (control_2/PREC_MAX_TORQUE));
	
	analogWrite(Prec1_PWM, pwm_1);
	analogWrite(Prec2_PWM, pwm_1);
	
}
