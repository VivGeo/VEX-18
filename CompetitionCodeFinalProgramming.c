#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           fClaw,         tmotorVex393TurboSpeed_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           frontL,        tmotorVex393TurboSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port3,           backL,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           backR,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           frontR,        tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port6,           leftForkLift,  tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port7,           rightForkLift, tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           armLift,       tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port9,           claw,          tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, None)


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)


//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

 float forkKp = 1.5;
float forkRange = 980;
float forkDown = -100;
float forkCurrentValue = 0;
float forkError = 0;
float forkTarget = 0;

float armError = 0;
int powerArm = 0;
float armCurrentValue = 0;
float armIntegral = 0;
float range = -1681;
float armKp = 14;
float armKd = 60;
float armKi = 1;
float target = 0;
float lastError = 0;
float derivative = 0;
float armIntegralActiveZone = 100;
float armIntegralLimits = 2000;

//CONSTANT VALUES USED FOR TUNING
float kp = 0.15; //0.15
float ki = 0;//0.005
float kd = 2;//2
float ks = 0.8; // ratio of left drive motors to right

//CONSTANT VALUES (INCHES)
float inchesPerTile = 24.25;
float ticksPerInch = 20.00275229;

//PROPORTIONAL, INTEGRATON, DERIVATIVE LOOP VARIABLES
float proportionalL;
float derivativeL;
float integralL;
float integralLimit=40;
float integralActiveZone=100;

//ERROR VARIARBLES
float errorPL;
float errorIL;
float errorDL;
float powerDrive;

//CONVERT TILES TO TICKS
float tilesToTicks(float tiles)
{
	return tiles*inchesPerTile*ticksPerInch;
}
void initializeArmForkClaw(){
	resetMotorEncoder(leftForkLift);
	resetMotorEncoder(armLift);
	resetMotorEncoder(claw);
	wait1Msec(100);
}
void initialize(){
	//ENCODERS SET TO 0
	nMotorEncoder[frontR]=0;
	nMotorEncoder[frontL]=0;
	wait1Msec(100);
}

void moveStraight(float numOfTiles){
	clearTimer(T1);
	initialize();
	int dummyCounter = 0;
	float targetTicks = tilesToTicks(numOfTiles);

	while(time1[T1]<1500 && dummyCounter<60){
		//PROPORTIONAL
		errorPL = targetTicks - nMotorEncoder[frontL];
		proportionalL = errorPL*kp;

		//TIMER
		if (abs(errorPL)<25){
			dummyCounter+=1;
		}

		//INTEGRAL
		if(abs(errorPL)<integralActiveZone)
		{
			errorIL+=errorPL;
			} else {
			errorIL = 0;
		}
		integralL=errorIL*ki;
		if(integralL > integralLimit)
		{
			integralL=integralLimit;
		}

		//DERIVATIVE
		derivativeL=(errorPL-errorDL)*kd;
		errorDL=errorPL;
		if(errorPL==0)
		{
			derivativeL=0;
		}

		//POWER FOR MOTORS AND OVERALL POWER
		powerDrive = proportionalL+derivativeL+integralL;
		motor[backR] = powerDrive;
		motor[backL] = powerDrive*ks;
		motor[frontR] = powerDrive;
		motor[frontL] = powerDrive*ks;

		wait1Msec(20);
	}
}

task forkLiftPID(){
	while (true){
		forkCurrentValue = getMotorEncoder(leftForkLift);
		forkError = forkTarget - forkCurrentValue;
		if (abs(forkError) <= 10){
			forkError = 0;
		}
		motor[leftForkLift] = ((forkKp*forkError);
		motor[rightForkLift] = ((forkKp*forkError);
		wait1Msec(10);
	}
}

void clawOpen(){
	motor[claw] = 127;
	wait1Msec(800);
}

void clawClosing(){
	motor[claw] = 0;
}

task armPidControl(){
	while (true){
		armCurrentValue = getMotorEncoder(armLift);
		armError = target - armCurrentValue;
		if (armError != 0)
		{
			derivative = armError-lastError;
		}
		else {
			lastError = 0;
			derivative = 0;
			armIntegral = 0;
		}

		if (abs(armError) < armIntegralActiveZone){
			armIntegral = armIntegral + armError;
			if (abs(armError) <= 2){
				armIntegral = 0;
			}
		}
		else {
			armIntegral = 0;
		}
		if (abs(armIntegral) > armIntegralLimits){
			armIntegral = armIntegralLimits;
		}
		powerArm = (int)((armKp*armError + armKd*derivative + armKi*armIntegral)*(127/range));
		motor[armLift] = powerArm;
		wait1Msec(20);
		lastError = armError;
	}
}

void gyroinit() {
	SensorType[gyro] = sensorNone;
	wait1Msec(300);
	SensorType[gyro] = sensorGyro;
	wait1Msec(2000);
}

void rotateBot(int anglex10){
	float startAngle = SensorValue[in1];
	//perfect values - p: .9, i - 0, d:4.4 (speed at .7)
	float kpRotate = 0.9;
	float kdRotate = 2.8;

	float speed = 0;
	int errorGyro2 = 1, integral = 0;
	float errorGyro = anglex10 - (SensorValue[in1] - startAngle);
	float gyro0 = SensorValue[in1];

	while (abs(errorGyro) > 15) {
		errorGyro = anglex10 - (SensorValue[in1] - startAngle);
		float speed = 0.7*(errorGyro*kpRotate) + ((errorGyro - errorGyro2)*kdRotate);

		motor[frontL] = -speed;
		motor[backL] = -speed;
		motor[frontR] = speed;
		motor[backR] = speed;

		errorGyro2 = errorGyro;
		wait1Msec(20);
	}
	motor[frontL] = 0;
	motor[backL] = 0;
	motor[frontR] = 0;
	motor[backR] = 0;
}

void rotateLeft (int angle) {
	rotateBot(angle*10);
}

void rotateRight (int angle) {
	rotateBot(-angle*10);
}

void closeClaw(){
	motor[fClaw]=50;
	wait1Msec(1000);
	motor[fClaw]=20;
}

void openClaw(){
	motor[fClaw]=-50;
	wait1Msec(1000);
	motor[fClaw]=0;
}



task autonomous()
{


	initializeArmForkClaw();
	gyroinit();
	startTask(forkLiftPID);

	moveStraight(1.7);
	delay(100);

	forkTarget = forkRange;
	delay(800);

	closeClaw();

	rotateRight(168);
	delay(50);

	openClaw();

	moveStraight(3); //2.915
	delay(500);

	rotateRight(130);
	delay(500);

	forkTarget = forkDown;
	delay(800);

	moveStraight(3);
	delay(500);

	forkTarget = forkRange;
	delay(800);

	closeClaw();

	moveStraight(-4);
	delay(500);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Remove this function call once you have "real" code.
    UserControlCodePlaceholderForTesting();
  }
}
