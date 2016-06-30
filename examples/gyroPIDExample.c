#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"
#include "../libGyro/NERD_Gyro.c"
#include "../libPID/NERD_PID.c"

#define SET_ANGLE 90

Gyro gyro;
PID gyroPid;

//power left drive motors
void
driveL(int val){
	val = abs(val) > 127 ? 127 * val/abs(val) : val;
	motor[port2] = val;
}

//power right drive motors
void
driveR(int val){
	val = abs(val) > 127 ? 127 * val/abs(val) : val;
	motor[port3] = val;
}

//gyro turn to target angle
void
gyroTurn(float fTarget){
	if(abs(fTarget) < 40)
		pidInit(gyroPid, 3.0, 0.0, 0.15, 3.0, 30.0);
	bool bAtGyro = false;
	long liAtTargetTime = nPgmTime;
	long liTimer = nPgmTime;
	float fGyroAngle = 0;

	while(!bAtGyro){
		//Calculate the delta time from the last iteration of the loop
		float fDeltaTime = (float)(nPgmTime - liTimer)/1000.0;
		//Reset loop timer
		liTimer = nPgmTime;

		fGyroAngle += gyroGetRate(gyro) * fDeltaTime;

		//Calculate the output of the PID controller and output to drive motors
		float driveOut = pidCalculate(gyroPid, fTarget, fGyroAngle);
		driveL(-driveOut);
		driveR(driveOut);

		//Stop the turn function when the angle has been within 3 degrees of the desired angle for 350ms
		if(abs(fTarget - fGyroAngle) > 3)
			liAtTargetTime = nPgmTime;
		if(nPgmTime - liAtTargetTime > 350){
			bAtGyro = true;
			driveL(0);
			driveR(0);
		}
	}

	//Reinitialize the PID constants to their original values in case they were changed
	pidInit(gyroPid, 2, 0, 0.15, 2, 20.0);
}

//Calibrate gyro and initialize PID controller
void
pre_auton(){
	//Allow gyro to settle and then init/calibrate (Takes a total of around 2 seconds)
	delay(1100);
	gyroInit(gyro, 1);

	/*Initialize PID controller for gyro
	 * kP = 2, kI = 0, kD = 0.15
	 * epsilon = 0
	*/
	pidInit(gyroPid, 2, 0, 0.15, 2, 20.0);
}

task
autonomous(){
	//Turn the robot by the desired angle
	gyroTurn(SET_ANGLE);
}

task
usercontrol(){
}
