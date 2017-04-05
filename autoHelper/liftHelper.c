#include "./liftHelper.h"

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
		if (lift[i] != -1)
			motor[lift[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

task
taskLiftHold () {
	liftHoldRunning = true;
	while (true) {
		driveLift(pidCalculate(liftPID, liftSetPoint, SensorValue(liftSensorPort)));
	}
}

void
liftInit (float kP, float kI, float kD, float inner, float outer, int sensorPort) {
	pidInit (liftPID, kP, kI, kD, inner, outer);
	liftSensorPort = sensorPort;
}

void
liftHold (float setPoint) {
	liftSetPoint = setPoint;

	liftStop ();

	startTask (taskLiftHold);
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