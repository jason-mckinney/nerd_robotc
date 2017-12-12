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
pidCalculateWithVelocitySet (PID pid, int fSetPoint, int fProcessVariable, int velocitySet) {
	float fDeltaTime = nPgmTime - pid.m_uliLastTime;
	pid.m_uliLastTime = nPgmTime;

	float fDeltaPV = 0;
	if(fDeltaTime > 0)
		fDeltaPV = (fProcessVariable - pid.m_fLastValue) / fDeltaTime + velocitySet;
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
pidCalculateVelocity (PID pid, int fSetPoint, int fProcessVariable) {
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

	float fOutput = fSetPoint * pid.m_fKP
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

	float Kv;
	float Ka;
	int *sensor;
	int velocityFilter [5];
	int velocityRead;

	char profileSetting;
	int cycleCounter;
	short motorOutput;
	float positionOut;
	int lastSensorValue;
	unsigned int lastTime;

	int finalPosition; //target position, in sensor units
	float positionSet; //position set point
	float velocitySet; //velocity set point
	float accelSet; //acceleration output
	float jerk; //rate of acceleration change

	char planComplete;

	int vMax; //max rate of system, in sensor units/second
	int t1; //time for velocity ramping
	int t2; //time for acceleration ramping
	int t4; //estimated time assuming constant max velocity
	int tMax; //total estimated time of profile

	float cycleTime;
	int positionCycles; //amount of cycles to wait before new position update
} motionProfiler;

motionProfiler profilerPool[10]; //  because RobotC is trash we need to allocate space for profilers at compile time instead of instantiating them as we need them
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
createMotionProfiler (int motorPort, int *sensor, int vMax, float Ka, int t1, int t2, int cycleTime, int positionCycles) {
	if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort] != NULL)
		return;

	if (t1 % cycleTime != 0 || t2 % cycleTime != 0)
		return;

	int i;
	motionProfiler *controller;

	for (i = 0; i < 10; i++) {
		if (uniqueControllers[i] != NULL)
			continue;

		uniqueControllers [i] = &(profilerPool [i]);
		controller = uniqueControllers [i];
		break;
	}

	motorController [motorPort] = controller;
	controller->sensor = sensor;
	controller->velocityRead = 0;
	controller->profileSetting = 0x00;
	controller->cycleCounter = 0;
	controller->motorOutput = 0;
	controller->lastSensorValue = *sensor;
	controller->lastTime = nPgmTime;
	controller->vMax = vMax;
	controller->positionOut = 0;
	controller->Ka = Ka;// 0.015;

	controller->velocityFilter[0] = 0;
	controller->velocityFilter[1] = 0;
	controller->velocityFilter[2] = 0;
	controller->velocityFilter[3] = 0;
	controller->velocityFilter[4] = 0;

	controller->t1 = t1;
	controller->t2 = t2;

	controller->cycleTime = cycleTime;
	controller->positionCycles = positionCycles;

	pidInit (controller->positionController, 0.5, 0, 0, 15, 100);
	pidInit (controller->velocityController, 0.1764, 0, 0, 50, 500);
}

void
createMotionProfiler (int motorPort, int *sensor, int vMax) {
	createMotionProfiler (motorPort, sensor, vMax, 0.0, 600, 300, 20, 4);
}

void
setPositionController (int motorPort, float kP, float kI, float kD, float innerBand, float outerBand) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController [motorPort];
	pidInit (profile->positionController, kP, kI, kD, innerBand, outerBand);
}

void
setVelocityController (int motorPort, float kP, float kI, float kD, float innerBand, float outerBand) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController [motorPort];
	pidInit (profile->velocityController, kP, kI, kD, innerBand, outerBand);
}

void
setMotionSlave (int motorPort, int masterPort) {
	if (motorController [masterPort] == NULL)
		return;
	motorController [motorPort] = motorController [masterPort];
}

void
setPosition (int motorPort, int position) {
	if (motorController [motorPort] == NULL)
		return;

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
	profile->planComplete = 0;
}

void
setPWMOutput (int motorPort, int output) {
	if (motorController [motorPort] == NULL)
		return;

	if (output > 127)
		output = 127;
	if (output < -127)
		output = -127;

	motorController[motorPort]->profileSetting = 0b00;
	motorController[motorPort]->motorOutput = output;
}

void
setVelocity (int motorPort, int velocity) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController[motorPort];
	profile->profileSetting = 0b10;
	profile->velocitySet = velocity;
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

			float deltaT = (nPgmTime - profile->lastTime)/1000.0;
			profile->lastTime = nPgmTime;
			int sensorV = *(profile->sensor);

			//get sensor velocity, ticks per second
			float sensorRate = (sensorV - profile->lastSensorValue) / deltaT;

			for (int j = 4; j > 0; --j) {
				profile->velocityFilter [j] = profile->velocityFilter [j-1];
			}
			profile->velocityFilter [0] = sensorRate;

			sensorRate = profile->velocityFilter [0] * 0.5 + profile->velocityFilter [1] * 0.25 + profile->velocityFilter [2] * 0.125 + profile->velocityFilter [3] * 0.0625 + profile->velocityFilter [4] * 0.0625;
			profile->velocityRead = sensorRate;
			profile->lastSensorValue = sensorV;

			if (profile->profileSetting == 0b11) {
				//get motion profile output
				if (profile->planComplete == 0) {
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
					} else {
						profile->velocitySet = 0;
						profile->positionSet = profile->finalPosition;
						profile->accelSet = 0;
						profile->planComplete = 1;
					}
				}

				profile->cycleCounter++;

				//do position PID if cycle includes it
				if (profile->cycleCounter % profile->positionCycles == 0) {
				 	profile->positionOut = pidCalculate (profile->positionController, profile->positionSet, *(profile->sensor));
				}

				//do velocity PID
				float velocityOut = pidCalculateVelocity (profile->velocityController, profile->positionOut + profile->velocitySet, profile->velocityRead) + profile->accelSet * profile->Ka;
				//float velocityOut =  profile->velocitySet * profile->Kv + profile->accelSet * profile->Ka;//pidCalculate (profile->velocityController, profile->velocitySet + profile->positionOut, profile->velocityRead);

				//set motor PWM output
				profile->motorOutput = velocityOut;

				if (profile->motorOutput > 127)
					profile->motorOutput = 127;
				else if (profile->motorOutput < -127)
					profile->motorOutput = -127;

				/*datalogDataGroupStart();
				datalogAddValue (0, profile->accelSet);
				datalogAddValue (1, profile->velocitySet);
				datalogAddValue (2, profile->positionSet);
				datalogAddValue (3, profile->motorOutput);
				datalogAddValue (4, profile->positionSet - *(profile->sensor));
				datalogAddValue (5, profile->velocityRead);
				datalogDataGroupEnd();*/

			} else if (profile->profileSetting == 0b10) {
				//do velocity PID
				float velocityOut = pidCalculateVelocity (profile->velocityController, profile->velocitySet, profile->velocityRead);

				//set motor PWM output
				profile->motorOutput = velocityOut;

				if (profile->motorOutput > 127)
					profile->motorOutput = 127;
				else if (profile->motorOutput < -127)
					profile->motorOutput = -127;
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
