#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_5,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           frontClaw,     tmotorVex393TurboSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           frontL,        tmotorVex393TurboSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port3,           backL,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           backR,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           frontR,        tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port6,           leftLift,      tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port7,           rightLift,     tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           armLift,       tmotorVex393TurboSpeed_MC29, openLoop, encoderPort, I2C_4)
#pragma config(Motor,  port9,           claw,          tmotorVex393TurboSpeed_MC29, PIDControl, reversed, encoderPort, I2C_5)
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


float error = 0;
int power = 0;
float armCurrentValue = 0;
float integral = 0;
float range = -1681;
float Kp = 14; 	//4.6	30 	30
float Kd = 60;	//1 	50	60
float Ki = 1;		//0.3 0.6 1
float target = 0;
float lastError = 0;
float derivative = 0;
float integralActiveZone = 100;
float integralLimits = 2000; // 50/Ki
float armToFront = -970 //1316, 1300
float armToBack = -350;

bool clawOpen = false;

float forkKp = 1.5;
float forkRange = 980;
float forkDown = 100; //91
float forkCurrentValue = 0;
float forkError = 0;
float forkTarget = 0;
int dir=0;
bool open=true;

void initialize(){
	resetMotorEncoder(leftLift);
	resetMotorEncoder(armLift);
	resetMotorEncoder(claw);
}
task manualControl(){
	int threshold=30;
	while(true){
		//DRIVE
		if(abs(vexRT[Ch3])>threshold){
			motor[frontL]=vexRT[Ch3];
			motor[backL]=vexRT[Ch3];
		}
		else{
			motor[frontL]=0;
			motor[backL]=0;
		}
		if(abs(vexRT[Ch2])>threshold){
			motor[frontR]=vexRT[Ch2];
			motor[backR]=vexRT[Ch2];
		}
		else{
			motor[frontR]=0;
			motor[backR]=0;
		}
		//FORK
		if(vexRT[Btn5U]==1){
			forkTarget=forkRange;
		}
		if(vexRT[Btn5D]==1){
			forkTarget=forkDown;
		}
		//CLAW
		if(vexRT[Btn8D]==1){
			clawOpen=true;
			wait1Msec(800);
			clawOpen=false;
		}
		//ARM
		if(vexRT[Btn6U]==1){
			target=armToFront;
		}
		if(vexRT[Btn6D]==1){
			target=armToBack;
		}
		if(vexRT[Btn7D]==1){
			if(open){
				dir=1;
			}
			else{
				dir=-1;
			}
		}
	}
}
task armPidControl(){
	while (true){
		armCurrentValue = getMotorEncoder(armLift);
		error = target - armCurrentValue;
		if (error != 0)
		{
			derivative = error-lastError;
		}
		else {
			lastError = 0;
			derivative = 0;
			integral = 0;
		}

		if (abs(error) < integralActiveZone){
			integral = integral + error;
			if (abs(error) <= 2){
				integral = 0;
			}
		}
		else {
			integral = 0;
		}
		if (abs(integral) > integralLimits){
			integral = integralLimits;
		}
		power = (int)((Kp*error + Kd*derivative + Ki*integral)*(127/range))+ 10;
		motor[armLift] = power;
		wait1Msec(20);
		lastError = error;
	}
}

task forkLiftPID(){
	while (true){
		forkCurrentValue = getMotorEncoder(leftLift);
		forkError = forkTarget - forkCurrentValue;
		if (abs(forkError) <= 10){
			forkError = 0;
		}
		motor[leftLift] = (int)((forkKp*forkError)*(127/forkRange));
		motor[rightLift] = (int)((forkKp*forkError)*(127/forkRange));
		wait1Msec(20);
	}
}
task theClaw(){
	while(true){
		if(clawOpen){
			motor[claw]=-127;
			wait1Msec(100);
			motor[claw]=0;
		}
		else{
			motor[claw]=0;
		}
	}
}
task fclaw(){
	while(true){
		if(dir==1){
			motor[frontClaw]=-50;
			wait1Msec(1000);
			motor[frontClaw]=-10;
			open=false;
		}
		else if(dir==-1){
			motor[frontClaw]=50;
			wait1Msec(1000);
			open=true;
			motor[frontClaw]=0;
			}
		dir=0;
	}
}

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

task autonomous()
{
  // ..........................................................................
  // Insert user code here.
  // ..........................................................................

  // Remove this function call once you have "real" code.
  AutonomousCodePlaceholderForTesting();
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

  initialize();
	startTask(manualControl);
	startTask(forkLiftPID);
	startTask(armPidControl);
	startTask(fclaw);
	startTask(theClaw);
	while(true);
}
