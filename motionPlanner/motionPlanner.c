#ifndef TRUESPEED_H
#define TRUESPEED_H
const unsigned int TrueSpeed[128] =
{
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 88, 89, 89, 90, 90,127,127,127
};
#endif

//----- PID controller -----//

#ifndef NERD_PID_h
#define NERD_PID_h


typedef struct{
	float m_fKP;
	float m_fKI;
	float m_fKD;
	float m_fEpsilonInner;
	float m_fEpsilonOuter;
	float m_fSigma;
	float m_fLastValue;
	unsigned long m_uliLastTime;
	float m_fLastSetPoint;
} PID;


#endif

#ifndef PID_C
#define PID_C
/**
 * initialize pid structure, set parameters
 *
 * @param pid instance of PID structure
 * @param fKP PID KP constant
 * @param fKI PID KI constant
 * @param fKD PID KD constant
 * @param fEpsilonInner inner bound of PID I summing cutoff
 * @param fEpsilonOuter outer bound of PID I summing cutoff
 */
void
pidInit (PID pid, float fKP, float fKI, float fKD, float fEpsilonInner, float fEpsilonOuter) {
	pid.m_fKP = fKP;
	pid.m_fKI = fKI;
	pid.m_fKD = fKD;
	pid.m_fEpsilonInner = fEpsilonInner;
	pid.m_fEpsilonOuter = fEpsilonOuter;
	pid.m_fSigma = 0;
	pid.m_fLastValue = 0;
	pid.m_uliLastTime = nPgmTime;
}

/**
 * initialize pid structure, set parameters based on another PID structure
 *
 * @param pid  instance of PID structure
 * @param toCopy  PID instance to copy settings from
 */
void pidInit (PID pid, PID toCopy) {
	pid.m_fKP = toCopy.m_fKP;
	pid.m_fKI = toCopy.m_fKI;
	pid.m_fKD = toCopy.m_fKD;
	pid.m_fEpsilonInner = toCopy.m_fEpsilonInner;
	pid.m_fEpsilonOuter = toCopy.m_fEpsilonOuter;
	pid.m_fSigma = 0;
	pid.m_fLastValue = 0;
	pid.m_uliLastTime = nPgmTime;
}

/**
 * calculate pid output
 *
 * @param pid instance of PID structure
 * @param fSetPoint set point of PID controller
 * @param fProcessVariable sensor/feedback value
 *
 * @return output value constrained from -127 to 127
 */
float
pidCalculate (PID pid, float fSetPoint, float fProcessVariable) {
	float fDeltaTime = (float)(nPgmTime - pid.m_uliLastTime) / 1000.0;
	pid.m_uliLastTime = nPgmTime;

	float fDeltaPV = 0;
	if(fDeltaTime > 0)
		fDeltaPV = (fProcessVariable - pid.m_fLastValue) / fDeltaTime;
	pid.m_fLastValue = fProcessVariable;

	float fError = fSetPoint - fProcessVariable;
	
	if(fabs(fError) > pid.m_fEpsilonInner && fabs(fError) < pid.m_fEpsilonOuter)
		pid.m_fSigma += fError * fDeltaTime;

	if (fabs (fError) > pid.m_fEpsilonOuter)
		pid.m_fSigma = 0;

	float fOutput = fError * pid.m_fKP
					+ pid.m_fSigma * pid.m_fKI
					- fDeltaPV * pid.m_fKD;

	return fOutput;
}
#endif

//----- Motion Planner -----//

#ifndef NERD_MOTIONPLANNER
#define NERD_MOTIONPLANNER

#define CASCADE_MULTIPLIER 4

typedef struct {
	PID positionController;
	PID velocityController;
	
	int *sensor;
	int compFilter [5];
	
	char controllerSetting;
	char cascadeCounter;
	short motorOutput;
	int lastSensorValue;
	unsigned int lastSensorTime;
	
	float positionSet;
	float velocitySet;
	float sensorRate;
	float scaleFactor;
} motionController;

motionController* motorController [10];
motionController* uniqueControllers [10];

//sensor variable
int rawSensorValue [20];

int*
getRawSensor (int port) {
	if (port < 0 || port > 19)
		return NULL;
	return &rawSensorValue [port];
}

void
createMotionController (int motorPort, int *sensor, PID positionPID, PID velocityPID) {
	if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort])
		return;

	int i;

	motionController controller;
	controller.sensor = sensor;
	controller.sensorRate = 0;
	controller.lastSensorValue = *sensor;
	controller.lastSensorTime = nPgmTime;
	controller.positionSet = *sensor;
	controller.velocitySet = 0;
	controller.cascadeCounter = CASCADE_MULTIPLIER;
	controller.motorOutput = 0;
	controller.controllerSetting = 0b00;
	controller.scaleFactor = 1.0;

	motorController [motorPort] = &controller;

	pidInit (motorController [motorPort]->positionController, positionPID.m_fKP, positionPID.m_fKI, positionPID.m_fKD, positionPID.m_fEpsilonInner, positionPID.m_fEpsilonOuter);
	pidInit (motorController [motorPort]->velocityController, velocityPID.m_fKP, velocityPID.m_fKI, velocityPID.m_fKD, velocityPID.m_fEpsilonInner, velocityPID.m_fEpsilonOuter);

	for (i = 0; i < 10; ++i) {
		if (uniqueControllers[i])
			continue;

		uniqueControllers [i] = &controller;
		break;
	}
}

void
createMotionController (int motorPort, int *sensor) {
	PID posPID;
	PID velPID;

	pidInit (posPID, 6.5, 0.65, 0.0065, 30, 100);
	pidInit (velPID, 0.12, 0.5, 0.0006, 50, 500);

	createMotionController (motorPort, sensor, posPID, velPID);
}

void
setMotionSlave (int motorPort, int masterPort) {
	if (motorController [masterPort] == NULL)
		return;
	motorController [motorPort] = motorController [masterPort];
}

void
setPositionSimple (int motorPort, float position) {
	motorController[motorPort]->controllerSetting = 0b01;
	motorController[motorPort]->positionSet = position;
}

void
setVelocity (int motorPort, float velocity) {
	motorController[motorPort]->controllerSetting = 0b10;
	motorController[motorPort]->velocitySet = velocity;
}

void
setPosition (int motorPort, float position) {
	motorController[motorPort]->controllerSetting = 0b11;
	motorController[motorPort]->positionSet = position;
	motorController[motorPort]->cascadeCounter = CASCADE_MULTIPLIER;
}

void
setPower (int motorPort, float power) {
	motorController[motorPort]->controllerSetting = 0b00;

	power = power * 1.27;

	if (power < -127)
		power = -127;
	if (power > 127)
		power = 127;

	power = sgn (power) * TrueSpeed [(int) (fabs (power))];
	motorController[motorPort]->motorOutput = (int) power;
}

void
setUnitsPerDegree (int motorPort, float units) {
	if (units == 0)
		return;
	if (motorController [motorPort] == NULL)
		return;

	motionController *m = motorController [motorPort];
	m->scaleFactor = 1.0/units;
}

task rawSensorMonitor () {
	int i;

	while (1) {
		for (i = 0; i < 20; ++i) {
			rawSensorValue [i] = SensorValue [i];
		}
	}
}

task motionPlanner () {
	int i;
	int lastTime = nPgmTime;

	startTask (rawSensorMonitor);

	while (1) {
		for (i = 0; i < 10; ++i) {
			if (uniqueControllers [i] == NULL)
				continue;

			if (uniqueControllers [i]->lastSensorTime != nPgmTime) {
				float dT = (float) (nPgmTime - uniqueControllers [i]->lastSensorTime) / 1000.0;
				int d = *(uniqueControllers [i]->sensor) - uniqueControllers [i]->lastSensorValue;
				float sensorRate =  ((float)d)/dT;

				for (int j = 4; j > 0; --j) {
					uniqueControllers [i]->compFilter [j] = uniqueControllers [i]->compFilter [j-1]; 
				}
				uniqueControllers [i]->compFilter [0] = sensorRate;

				sensorRate = uniqueControllers [i]->compFilter [0] * 0.5 + uniqueControllers [i]->compFilter [1] * 0.25 + uniqueControllers [i]->compFilter [2] * 0.125 + uniqueControllers [i]->compFilter [3] * 0.0625 + uniqueControllers [i]->compFilter [4] * 0.0625;
				uniqueControllers [i]->sensorRate = sensorRate;
				uniqueControllers [i]->lastSensorValue = *(uniqueControllers [i]->sensor);
				uniqueControllers [i]->lastSensorTime = nPgmTime;
			}

			if (uniqueControllers [i]->controllerSetting == 0b01) { //position set
				float output = pidCalculate (uniqueControllers [i]->positionController, uniqueControllers [i]->positionSet, *(uniqueControllers [i]->sensor));

				uniqueControllers [i]->motorOutput = output;
			} else if (uniqueControllers [i]->controllerSetting == 0b10) { //velocity set
				float output = pidCalculate (uniqueControllers [i]->velocityController, uniqueControllers [i]->velocitySet, uniqueControllers [i]->sensorRate);

				uniqueControllers [i]->motorOutput = output;
			} else if (uniqueControllers [i]->controllerSetting == 0b11) { //position + velocity set
				if (uniqueControllers [i]->cascadeCounter >= CASCADE_MULTIPLIER) {
					uniqueControllers [i]->velocitySet = uniqueControllers [i]->scaleFactor * pidCalculate (uniqueControllers [i]->positionController, uniqueControllers [i]->positionSet, *(uniqueControllers [i]->sensor));
					uniqueControllers [i]->cascadeCounter = 0;
				}
				uniqueControllers [i]->cascadeCounter++;

				float velOutput = pidCalculate (uniqueControllers [i]->velocityController, uniqueControllers [i]->velocitySet, uniqueControllers [i]->sensorRate);
				uniqueControllers [i]->motorOutput = velOutput;
			}
		}

		for (i = 0; i < 10; ++i) {
			if (motorController [i] == NULL)
				continue;

			if (motorController [i]->motorOutput > 127)
				motorController [i]->motorOutput = 127;
			else if (motorController [i]->motorOutput < -127)
				motorController [i]->motorOutput = -127;

			motor[i] = sgn(motorController [i]->motorOutput) * TrueSpeed[(int) fabs (motorController [i]->motorOutput)];
		}

		while (nPgmTime - lastTime < 20) {
			delay (1);
		}
		lastTime = nPgmTime;
	}
}

#endif
