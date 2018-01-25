#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           left,          tmotorVex393TurboSpeed_MC29, PIDControl, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port3,           right,         tmotorVex393TurboSpeed_MC29, PIDControl, encoderPort, I2C_2)
#pragma config(Motor,  port4,           lift,          tmotorVex393TurboSpeed_MC29, PIDControl, encoderPort, I2C_3)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

void gyroinit() {
	SensorType[gyro] = sensorNone;
	wait1Msec(300);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
}

void rotateBot(int anglex10){
	float startAngle = SensorValue[gyro];
	//Adjust these values, note them in this comment so we have a rough idea of what works
	// kp : VAL ki : VAL kd : VAL
	float kp = 0.9;
	float ki = 0; //try leaving as zero
	float kd = 2.8;

	float speed = 0; //idk if this is supposed to be assigned something
	int error2 = 1, integral = 0;
	float error = anglex10 - (SensorValue[gyro] - startAngle);
	float gyro0 = SensorValue[gyro];

	while (abs(error) > 15) {
		error = anglex10 - (SensorValue[in1] - startAngle);
		float speed = 0.7*(error*kp) + ((error - error2)*kd);


		
		motor[right] = -speed;
		motor[left] = speed;

		error2 = error;
		wait1Msec(20);
	}
	motor[right] = 0;
	motor[left] = 0;
}

void rotateLeft (int angle) {
	rotateBot(angle*10);
}

void rotateRight (int angle) {
	rotateBot(-angle*10);
}

task main()
{
	gyroinit();
	rotateRight(90);
	
	
	wait1Msec(500);
	//rotateLeft(180);
	//wait1Msec(500);
	//rotateRight(90);
	//wait1Msec(500);
	//rotateRight(180);

	//rotateLeft(90);

	//wait1Msec(500);
	//rotateLeft(90);
  //rotateRight(180);

}
