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


typedef struct {
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

float
pidCalculateWithSigma (PID pid, int fSetPoint, int fProcessVariable, int pvSigma) {
	float fDeltaTime = nPgmTime - pid.m_uliLastTime;
	pid.m_uliLastTime = nPgmTime;

	float fDeltaPV = 0;
	if(fDeltaTime > 0)
		fDeltaPV = (fProcessVariable - pid.m_fLastValue) / fDeltaTime;
	pid.m_fLastValue = fProcessVariable;

	float fError = fSetPoint - fProcessVariable;

	pid.m_fSigma = pvSigma;

	float fOutput = fError * pid.m_fKP
					+ pvSigma * pid.m_fKI
					- fDeltaPV * pid.m_fKD;

	return fOutput;
}

float
pidCalculateWithRate (PID pid, int fSetPoint, int fProcessVariable, float pvDelta) {
	float fDeltaTime = nPgmTime - pid.m_uliLastTime;
	pid.m_uliLastTime = nPgmTime;

	pid.m_fLastValue = fProcessVariable;

	float fError = fSetPoint - fProcessVariable;

	if(fabs(fError) > pid.m_fEpsilonInner && fabs(fError) < pid.m_fEpsilonOuter)
		pid.m_fSigma += fError * fDeltaTime;

	if (fabs (fError) > pid.m_fEpsilonOuter)
		pid.m_fSigma = 0;

	float fOutput = fError * pid.m_fKP
					+ pid.m_fSigma * pid.m_fKI
					- pvDelta * pid.m_fKD;

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
	int velocityFilter [5];
	int velocityRead;

	char profileSetting;
	char cycleCounter;
	short motorOutput;
	int lastSensorValue;
	unsigned int lastTime;

	int finalPosition; //target position, in sensor units
	float positionSet; //position set point
	float velocitySet; //velocity set point
	float accelSet; //acceleration output
	float jerk; //rate of acceleration change

	int vMax; //max rate of system, in sensor units/second
	int t1; //time for velocity ramping
	int t2; //time for acceleration ramping
	int t4; //estimated time assuming constant max velocity
	int tMax; //total estimated time of profile
	
	int cycleTime;
	int positionCycles; //amount of cycles to wait before new position update
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
createMotionProfiler (int motorPort, int *sensor, int vMax, int t1, int t2, int cycleTime, int positionCycles) {
	if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort])
		return;

	int i;

	motionProfiler controller;
	controller.sensor = sensor;
	controller.velocityRead = 0;
	controller.profileSetting = 0x00;
	controller.cycleCounter = 0;
	controller.motorOutput = 0;
	controller.lastSensorValue = *sensor;
	controller.lastTime = nPgmTime;
	controller.vMax = vMax;

	controller.velocityFilter[0] = 0;
	controller.velocityFilter[1] = 0;
	controller.velocityFilter[2] = 0;
	controller.velocityFilter[3] = 0;
	controller.velocityFilter[4] = 0;

	controller.t1 = t1;
	controller.t2 = t2;

	controller.cycleTime = cycleTime;
	controller.positionCycles = positionCycles;

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

void
setPosition (int motorPort, int position) {
	motionProfiler *profile = motorController [motorPort];

	int distance = position - *(profile->sensor);

	profile->t4 = 1000 * fabs (distance) / profile->vMax;
	
	if (profile->t4 < profile->t1 + profile->t2) {
		//short movement profile, not implemented yet...
		return;
	}

	profile->profileSetting = 0b11;
	profile->finalPosition = position;
	profile->tMax = profile->t1 + profile->t2 + profile->t4;
	profile->jerk = sgn (distance) * profile->vMax/(profile->t1/1000.0)/(profile->t2/1000.0);
	profile->accelSet = 0;
	profile->velocitySet = 0;
	profile->positionSet = 0;

	profile->motorOutput = 0;
	profile->cycleCounter = 0;
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

			motionProfiler *profile = uniqueControllers [i];

			if (nPgmTime - profile->lastTime < profile->cycleTime) {
				continue;
			}

			int sensorValue = *(profile->sensor);

			if (profile->profileSetting == 0b11) {
				//get motion profile output
				if (profile->cycleCounter < profile->t2 / profile->cycleTime) {
					//J+
					profile->accelSet += profile->jerk * (profile->cycleTime/1000.0);
				} else if (profile->cycleCounter >= profile->t1 / profile->cycleTime && profile->cycleCounter < (profile->t1 + profile->t2) / profile->cycleTime) {
					//J-
					profile->accelSet -= profile->jerk * (profile->cycleTime/1000.0);
				} else if (profile->cycleCounter >= profile->t4 / profile->cycleTime && profile->cycleCounter < (profile->t4 + profile->t2) / profile->cycleTime) {
					//J-
					profile->accelSet -= profile->jerk * (profile->cycleTime/1000.0);
				} else if (profile->cycleCounter >= (profile->t4 + profile->t1) / profile->cycleTime && profile->cycleCounter < profile->tMax) {
					//J+
					profile->accelSet += profile->jerk * (profile->cycleTime/1000.0);
				} 

				if (profile->cycleCounter < profile->tMax / profile->cycleTime) {
					profile->velocitySet += profile->accelSet * (profile->cycleTime/1000.0);
					profile->positionSet += profile->velocitySet * (profile->cycleTime/1000.0);
					
					profile->cycleCounter++;
				} else {
					profile->velocitySet = 0;
					profile->positionSet = profile->finalPosition;
					profile->accelSet = 0;
				}

				//get sensor velocity, ticks per cycle
				int sensorRate = sensorValue - uniqueControllers [i]->lastSensorValue;

				for (int j = 4; j > 0; --j) {
					profile->velocityFilter [j] = profile->velocityFilter [j-1];
				}
				profile->velocityFilter [0] = sensorRate;

				sensorRate = profile->velocityFilter [0] * 0.5 + profile->velocityFilter [1] * 0.25 + profile->velocityFilter [2] * 0.125 + profile->velocityFilter [3] * 0.0625 + profile->velocityFilter [4] * 0.0625;
				uniqueControllers [i]->velocityRead = sensorRate;
				uniqueControllers [i]->lastSensorValue = sensorValue;

				//do position PID if cycle includes it
				float positionOut = pidCalculateWithRate (profile->positionController, round (profile->positionSet), *(profile->sensor), sensorRate);

				//do velocity PID
				float velocityOut = pidCalculateWithSigma (profile->velocityController, round (profile->velocitySet + positionOut), sensorRate, *(profile->sensor));

				//set motor PWM output
				profile->motorOutput = velocityOut;

				if (profile->motorOutput > 127)
					profile->motorOutput = 127;
				else if (profile->motorOutput < -127)
					profile->motorOutput = -127;

				profile->lastTime = nPgmTime;
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
	}
}

#endif
