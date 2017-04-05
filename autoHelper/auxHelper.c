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

void 
aux1Hold (float setPoint) {
	aux1SetPoint = setPoint;
	aux1Stop ();
	startTask (taskAux1Hold);
}

void 
aux2Hold (float setPoint) {
	aux2SetPoint = setPoint;
	aux2Stop ();
	startTask (taskAux2Hold);
}

void 
aux1Stop () {
	stopTask (taskAux1Hold);
	driveAux1 (0);
	aux1HoldRunning = false;
}

void 
aux1HoldStop () {
	if (aux1HoldRunning) {
		stopTask (taskAux1Hold);
		driveAux1 (0);
		aux1HoldRunning = false;
	}
}

void 
aux2Stop () {
	stopTask (taskAux2Hold);
	driveAux2 (0);
	aux2HoldRunning = false;
}

void 
aux2HoldStop () {
	if (aux2HoldRunning) {
		stopTask (taskAux2Hold);
		driveAux2 (0);
		aux2HoldRunning = false;
	}
}

task
taskAux1Hold () {
	aux1HoldRunning = true;
	while (true) {
		driveAux1 (pidCalculate(aux1PID, aux1SetPoint, SensorValue(aux1SensorPort)));
	}
}

task
taskAux2Hold () {
	aux2HoldRunning = true;
	while (true) {
		driveAux2 (pidCalculate(aux2PID, aux2SetPoint, SensorValue(aux2SensorPort)));
	}
}