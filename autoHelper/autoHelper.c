int lift [10];
int aux1 [10];
int aux2 [10];

PID liftPID;
PID aux1PID;
PID aux2PID;

float liftSetPoint;
int liftSensorPort;

float aux1SetPoint;
int aux1SensorPort;

float aux2SetPoint;
int aux2SensorPort;

bool liftHoldRunning = false;
bool aux1HoldRunning = false;
bool aux2HoldRunnin = false;

void liftStop ();
void aux1Stop ();
void aux2Stop ();

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
setAux1Motors (int aux0,
	int aux1 = -1,
	int aux2 = -1,
	int aux3 = -1,
	int aux4 = -1,
	int aux5 = -1,
	int aux6 = -1,
	int aux7 = -1,
	int aux8 = -1,
	int aux9 = -1) {
		aux1[0] = aux0;
		aux1[1] = aux1;
		aux1[2] = aux2;
		aux1[3] = aux3;
		aux1[4] = aux4;
		aux1[5] = aux5;
		aux1[6] = aux6;
		aux1[7] = aux7;
		aux1[8] = aux8;
		aux1[9] = aux9;
}

void
setAux2Motors (int aux0,
	int aux1 = -1,
	int aux2 = -1,
	int aux3 = -1,
	int aux4 = -1,
	int aux5 = -1,
	int aux6 = -1,
	int aux7 = -1,
	int aux8 = -1,
	int aux9 = -1) {
		aux2[0] = aux0;
		aux2[1] = aux1;
		aux2[2] = aux2;
		aux2[3] = aux3;
		aux2[4] = aux4;
		aux2[5] = aux5;
		aux2[6] = aux6;
		aux2[7] = aux7;
		aux2[8] = aux8;
		aux2[9] = aux9;
}

void
driveLift (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;

	for (int i = 0; i < 10; ++i)
		if (lift[i] != -1)
			motor[lift[i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

void
driveAux1 (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;
	for (int i = 0; i < 10; ++i)
		if (aux1 [i] != -1)
			motor [aux1 [i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

void
driveAux2 (int speed) {
	if (fabs (speed) > 127)
		speed = sgn (speed) * 127;
	for (int i = 0; i < 10; ++i)
		if (aux2 [i] != -1)
			motor [aux2 [i]] = sgn (speed) * TrueSpeed [fabs (speed)];
}

task
taskLiftHold () {
	liftHoldRunning = true;
	while (true) {
		driveLift(pidCalculate(liftPID, liftSetPoint, SensorValue(liftSensorPort)));
	}
}

task
taskAux1Hold () {
	aux1HoldRunning = true;
	while (true) {
		driveAux1(pidCalculate(aux1PID, aux1SetPoint, SensorValue(aux1SensorPort)));
	}
}

task
taskAux2Hold () {
	aux2HoldRunning = true;
	while (true) {
		driveAux2(pidCalculate(aux2PID, aux2SetPoint, SensorValue(aux2SensorPort)));
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