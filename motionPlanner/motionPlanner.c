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
	int m_fEpsilonInner;
	int m_fEpsilonOuter;
	int m_fSigma;
	int m_fLastValue;
	unsigned long m_uliLastTime;
	int m_fLastSetPoint;
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
pidInit (PID pid, float fKP, float fKI, float fKD, int fEpsilonInner, int fEpsilonOuter) {
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
pidCalculate (PID pid, int fSetPoint, int fProcessVariable) {
	float fDeltaTime = nPgmTime - pid.m_uliLastTime;
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

typedef struct {
	PID positionController;
	PID velocityController;

	int *sensor;
	int compFilter [5];

	char profileSetting;
	char cycleCounter;
	short motorOutput;
	int lastSensorValue;
	unsigned int lastTime;

	int finalPosition;
	int positionSet;
	int velocitySet;
	int velocityRead;

	int vMax;
	int t1;
	int t2;
	int t4;
	int cycleTime;
	int positionRate;
	int n;

	int FL1Length;
	char FL1 [100];

	int FL2Length;
	char FL2 [100];
} motionProfiler;

motionProfiler* motorController [10];
motionProfiler* uniqueControllers [10];

//sensor variable
int rawSensorValue [20];

int*
getRawSensor (int port) {
	if (port < 0 || port > 19)
		return NULL;
	return &rawSensorValue [port];
}

void
createMotionProfiler (int motorPort, int *sensor, int vMax, int t1, int t2, int cycleTime, int positionRate) {
	if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort])
		return;

	int i;

	motionProfiler controller;
	controller.cycleCounter = 0;
	controller.sensor = sensor;
	controller.velocityRead = 0;
	controller.lastSensorValue = *sensor;
	controller.lastTime = nPgmTime;
	controller.positionSet = *sensor;
	controller.velocitySet = 0;
	controller.motorOutput = 0;
	controller.profileSetting = 0b00;

	controller.vMax = vMax;
	controller.t1 = t1;
	controller.t2 = t2;
	controller.cycleTime = cycleTime;
	controller.positionRate = positionRate;

	motorController [motorPort] = &controller;

	pidInit (motorController [motorPort]->positionController, 1, 0.1, 0.01, 0, 0);
	pidInit (motorController [motorPort]->velocityController, 1, 0.1, 0.01, 0, 0);

	for (i = 0; i < 10; ++i) {
		if (uniqueControllers[i])
			continue;

		uniqueControllers [i] = &controller;
		break;
	}
}

void
createMotionProfiler (int motorPort, int *sensor, int vMax) {
	createMotionProfiler (motorPort, sensor, vMax, 200, 100, 20, 4);
}

void
setMotionSlave (int motorPort, int masterPort) {
	if (motorController [masterPort] == NULL)
		return;
	motorController [motorPort] = motorController [masterPort];
}

/*void
setVelocity (int motorPort, float velocity) {
	motorController[motorPort]->controllerSetting = 0b10;
	motorController[motorPort]->velocitySet = velocity;
}*/

void
setPosition (int motorPort, int position) {
	int distance = position - *(motorController [motorPort]->sensor);

	motorController[motorPort]->profileSetting = 0b11;
	motorController [motorPort]->finalPosition = position;
	motorController [motorPort]->t4 = 1000 * distance / motorController [motorPort]->vMax;
	motorController [motorPort]->n = ceil ((float) motorController [motorPort]->t4 / motorController [motorPort]->cycleTime);

	int FL1Length = ceil ((float) motorController [motorPort]->t1 / motorController [motorPort]->cycleTime);
	//motorController [motorPort]->FL1 =
	//motorController [motorPort]->sizeFL2 = ceil ((float) motorController [motorPort]->t2 / motorController [motorPort]->cycleTime);

	motorController [motorPort]->cycleCounter = 0;
}

void
setPower (int motorPort, float power) {
	motorController[motorPort]->profileSetting = 0b00;

	power = power * 1.27;

	if (power < -127)
		power = -127;
	if (power > 127)
		power = 127;

	power = sgn (power) * TrueSpeed [(int) (fabs (power))];
	motorController[motorPort]->motorOutput = (int) power;
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

	startTask (rawSensorMonitor);

	while (1) {
		for (i = 0; i < 10; ++i) {
			if (uniqueControllers [i] == NULL)
				continue;

			if (nPgmTime - uniqueControllers [i]->lastTime < uniqueControllers [i]->cycleTime) {
				continue;
			}

			int sensorValue = *(uniqueControllers [i]->sensor);

			//get motion profile output

			//filter 1
			/*if (uniqueControllers [i]->cycleCounter < uniqueControllers [i]->n && uniqueControllers [i]->FL1 < 1) {
				uniqueControllers [i]->FL1 += 1.0/(uniqueControllers [i]->sizeFL1);

				if (uniqueControllers [i]->FL1 > 1) {
					uniqueControllers [i]->FL1 = 1;
				}
			} else if (uniqueControllers [i] > 0) {
				uniqueControllers [i]->FL1 -= 1.0/(uniqueControllers [i]->sizeFL1);

				if (uniqueControllers [i]->FL1 < 0){
					uniqueControllers [i]->FL1 = 0;
				}
			}*/

			//filter 2


			//do position PID if cycle includes it

			//get sensor velocity, ticks per cycle
			int sensorRate = sensorValue - uniqueControllers [i]->lastSensorValue;

			for (int j = 4; j > 0; --j) {
				uniqueControllers [i]->compFilter [j] = uniqueControllers [i]->compFilter [j-1];
			}
			uniqueControllers [i]->compFilter [0] = sensorRate;

			sensorRate = uniqueControllers [i]->compFilter [0] * 0.5 + uniqueControllers [i]->compFilter [1] * 0.25 + uniqueControllers [i]->compFilter [2] * 0.125 + uniqueControllers [i]->compFilter [3] * 0.0625 + uniqueControllers [i]->compFilter [4] * 0.0625;
			uniqueControllers [i]->velocityRead = sensorRate;
			uniqueControllers [i]->lastSensorValue = sensorValue;

			//do velocity PID


			//set motor PWM output
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
	}
}

#endif
