#include "./libPID/NERD_PID.c"
#include "./libToolkit/TrueSpeed.h"
#include "./libGyro/NERD_Gyro.c"

int leftDrive [10];
int rightDrive [10];
int lift [10];

PID leftDrivePID;
PID rightDrivePID;
PID liftPID;
PID driveGyroPID;

Gyro driveGyro;

float liftSetPoint;
int liftSensorPort;

float leftDriveSetPoint;
int leftDriveSensorPort;

float rightDriveSetPoint;
int rightDriveSensorPort;

float driveGyroSetPoint;
int driveGyroSensorPort;
float driveGyroAngle;

bool liftHoldRunning = false;
bool driveHoldRunning = false;

void driveStop ();
void liftStop ();

void
setLeftDriveMotors (int drive0,
	int drive1 = -1,
	int drive2 = -1,
	int drive3 = -1,
	int drive4 = -1,
	int drive5 = -1,
	int drive6 = -1,
	int drive7 = -1,
	int drive8 = -1,
	int drive9 = -1) {
		leftDrive[0] = drive0;
		leftDrive[1] = drive1;
		leftDrive[2] = drive2;
		leftDrive[3] = drive3;
		leftDrive[4] = drive4;
		leftDrive[5] = drive5;
		leftDrive[6] = drive6;
		leftDrive[7] = drive7;
		leftDrive[8] = drive8;
		leftDrive[9] = drive9;
}

void
setRightDriveMotors (int drive0,
	int drive1 = -1,
	int drive2 = -1,
	int drive3 = -1,
	int drive4 = -1,
	int drive5 = -1,
	int drive6 = -1,
	int drive7 = -1,
	int drive8 = -1,
	int drive9 = -1) {
		rightDrive[0] = drive0;
		rightDrive[1] = drive1;
		rightDrive[2] = drive2;
		rightDrive[3] = drive3;
		rightDrive[4] = drive4;
		rightDrive[5] = drive5;
		rightDrive[6] = drive6;
		rightDrive[7] = drive7;
		rightDrive[8] = drive8;
		rightDrive[9] = drive9;
}

void
setLiftMotors (int lift0,
	int lift1 = -1,
	int lift2 = -1,
	int lift3 = -1,
	int lift4 = -1,
	int lift5 = -1,
	int lift6 = -1,
	int lift7 = -1,
	int lift8 = -1,
	int lift9 = -1) {
		lift[0] = lift0;
		lift[1] = lift1;
		lift[2] = lift2;
		lift[3] = lift3;
		lift[4] = lift4;
		lift[5] = lift5;
		lift[6] = lift6;
		lift[7] = lift7;
		lift[8] = lift8;
		lift[9] = lift9;
}

void
driveLift (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;

	for (int i = 0; i < 10; ++i)
		if (lift[i] != 0)
			motor[lift[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

void
driveLeftDrive (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;

	for (int i = 0; i < 10; ++i)
		if (leftDrive[i] != 0)
			motor[leftDrive[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

void
driveRightDrive (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;

	for (int i = 0; i < 10; ++i)
		if (rightDrive[i] != 0)
			motor[rightDrive[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

task
taskLiftHold () {
	liftHoldRunning = true;
	while (true) {
		driveLift(pidCalculate(liftPID, liftSetPoint, SensorValue(liftSensorPort)));
	}
}

task
taskDriveHold () {
	driveHoldRunning = true;
	while (true) {
		driveLeftDrive (pidCalculate (leftDrivePID, leftDriveSetPoint, SensorValue (leftDriveSensorPort)));
		driveRightDrive (pidCalculate (rightDrivePID, rightDriveSetPoint, SensorValue (rightDriveSensorPort)));
	}
}

task
taskDriveGyroHold () {
	long lastTime = nPgmTime;

	while (true) {
		float dT = (nPgmTime - lastTime) / 1000.0;
		lastTime = nPgmTime;
		driveGyroAngle += gyroGetRate (driveGyro) * dT;

		int out = pidCalculate (driveGyroPID, driveGyroSetPoint, driveGyroAngle);

		driveLeftDrive (out);
		driveRightDrive (-out);
	}
}

void
liftInit (float kP, float kI, float kD, float inner, float outer, int sensorPort) {
	pidInit (liftPID, kP, kI, kD, inner, outer);
	liftSensorPort = sensorPort;
}

void
driveInit (float kP, float kI, float kD, float inner, float outer, int sensorPortLeft, int sensorPortRight) {
	pidInit (leftDrivePID, kP, kI, kD, inner, outer);
	pidInit (rightDrivePID, kP, kI, kD, inner, outer);

	leftDriveSensorPort = sensorPortLeft;
	rightDriveSensorPort = sensorPortRight;
}

void
driveGyroInit (float kP, float kI, float kD, float inner, float outer, int sensorPort) {
	pidInit (driveGyroPID, kP, kI, kD, inner, outer);

	driveGyroSensorPort = sensorPort;
	driveGyroAngle = 0.0;

	gyroInit (driveGyro, driveGyroSensorPort);
}

void
liftHold (float setPoint) {
	liftSetPoint = setPoint;

	liftStop ();

	startTask (taskLiftHold);
}

void
driveHold (float setPoint) {
	leftDriveSetPoint = setPoint;
	rightDriveSetPoint = setPoint;

	driveStop ();

	startTask (taskDriveHold);
}

void
driveHold (float setPointLeft, float setPointRight) {
	leftDriveSetPoint = setPointLeft;
	rightDriveSetPoint = setPointRight;

	driveStop ();

	startTask (taskDriveHold);
}

void
driveGyroHold (float setPoint) {
	driveGyroSetPoint = setPoint;

	driveStop ();

	startTask (taskDriveGyroHold);
}

void
liftStop () {
	stopTask (taskLiftHold);
	driveLift (0);
	liftHoldRunning = false;
}

void
liftHoldStop () {
	if (liftHoldRunning) {
		stopTask (taskLiftHold);
		driveLift (0);
		liftHoldRunning = false;
	}
}

void
driveStop () {
	stopTask (taskDriveHold);
	stopTask (taskDriveGyroHold);
	driveLeftDrive (0);
	driveRightDrive (0);
	driveHoldRunning = false;
}

void
driveHoldStop () {
	if (driveHoldRunning) {
		stopTask (taskDriveHold);
		stopTask (taskDriveGyroHold);
		driveLeftDrive (0);
		driveRightDrive (0);
		driveHoldRunning = false;
	}
}

void
liftGoTo (float setPoint, float range) {
	bool atValue = false;
	long atTime = nPgmTime;

	SensorValue [liftSensorPort] = 0;

	liftHold (setPoint);

	while (!atValue) {
		if (fabs(setPoint - SensorValue(liftSensorPort)) > range)
			atTime = nPgmTime;
		else if (nPgmTime - atTime > 500)
			atValue = true;
	}
}

void
driveGoTo (float setPoint, float range) {
	bool atValue = false;
	long atTime = nPgmTime;

	SensorValue [leftDriveSensorPort] = 0;
	SensorValue [rightDriveSensorPort] = 0;

	driveHold (setPoint);

	while (!atValue) {
		if (fabs(setPoint - SensorValue(leftDriveSensorPort)) > range && fabs(setPoint - SensorValue(rightDriveSensorPort)) > range)
			atTime = nPgmTime;
		else if (nPgmTime - atTime > 500)
			atValue = true;
	}
}

void
driveTurnLeft (float setPoint, float range) {
	bool atValue = false;
	long atTime = nPgmTime;

	SensorValue [leftDriveSensorPort] = 0;
	SensorValue [rightDriveSensorPort] = 0;

	driveHold (-setPoint, setPoint);

	while (!atValue) {
		if (fabs(setPoint - SensorValue(leftDriveSensorPort)) > range && fabs(setPoint - SensorValue(rightDriveSensorPort)) > range)
			atTime = nPgmTime;
		else if (nPgmTime - atTime > 500)
			atValue = true;
	}
}

void
driveTurnRight (float setPoint, float range) {
	long atTime = nPgmTime;

	SensorValue [leftDriveSensorPort] = 0;
	SensorValue [rightDriveSensorPort] = 0;

	driveHold (setPoint, -setPoint);

	while (true) {
		if (fabs(setPoint - SensorValue(leftDriveSensorPort)) > range && fabs(setPoint - SensorValue(rightDriveSensorPort)) > range)
			atTime = nPgmTime;
		else if (nPgmTime - atTime > 500)
			break;
	}
}

void
driveGyroTurn (float setPoint, float range) {
	long atTime = nPgmTime;

	driveGyroHold (setPoint);

	while (true) {
		if (fabs (setPoint - driveGyroAngle) > range)
			atTime = nPgmTime;
		else if (nPgmTime - atTime > 500)
			break;
	}
}
