#include "./driveHelper.h"

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
driveGyroHold (float setPoint) {
	driveGyroSetPoint = setPoint;

	driveStop ();
	driveGyroAngle = 0;

	startTask (taskDriveGyroHold);
}

void
driveGyroInit (float kP, float kI, float kD, float inner, float outer, int sensorPort) {
	pidInit (driveGyroPID, kP, kI, kD, inner, outer);

	driveGyroSensorPort = sensorPort;
	driveGyroAngle = 0.0;

	gyroInit (driveGyro, driveGyroSensorPort);
}

void
driveGyroTurn (float setPoint, float range) {
	long atTime = nPgmTime;

	driveGyroHold (setPoint);

	while (true) {
		if (fabs (setPoint - driveGyroAngle) > range)
			atTime = nPgmTime;
		else if (nPgmTime - atTime > 200)
			break;
	}
	stopTask (taskDriveGyroHold);
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
driveInit (float kP, float kI, float kD, float inner, float outer, int sensorPortLeft, int sensorPortRight) {
	pidInit (leftDrivePID, kP, kI, kD, inner, outer);
	pidInit (rightDrivePID, kP, kI, kD, inner, outer);

	if (driveSlavePID.m_uliLastTime == 0)
		pidInit (driveSlavePID, 0.3, 0, 0.03, 0, 0);

	leftDriveSensorPort = sensorPortLeft;
	rightDriveSensorPort = sensorPortRight;
}

void
driveSlaveInit (float kP, float kI, float kD, float inner, float outer) {
	pidInit (driveSlavePID, kP, kI, kD, inner, outer);
}

void
driveLeftDrive (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;

	for (int i = 0; i < 10; ++i)
		if (leftDrive[i] != -1)
			motor[leftDrive[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

void
driveRightDrive (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;

	for (int i = 0; i < 10; ++i)
		if (rightDrive[i] != -1)
			motor[rightDrive[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
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

task
taskDriveHold () {
	driveHoldRunning = true;
	while (true) {
		if (leftDriveSetPoint == rightDriveSetPoint) {
			float driveOut = pidCalculate (leftDrivePID, leftDriveSetPoint, SensorValue (leftDriveSensorPort));
			float slaveOut = pidCalculate (driveSlavePID, SensorValue (leftDriveSensorPort), SensorValue (rightDriveSensorPort));
			driveLeftDrive (driveOut);
			driveRightDrive (driveOut + slaveOut);
		} else {
			driveLeftDrive (pidCalculate (leftDrivePID, leftDriveSetPoint, SensorValue (leftDriveSensorPort)));
			driveRightDrive (pidCalculate (rightDrivePID, rightDriveSetPoint, SensorValue (rightDriveSensorPort)));
		}
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

		driveLeftDrive (-out);
		driveRightDrive (out);
	}
}
