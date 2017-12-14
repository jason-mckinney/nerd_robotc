/// \cond IGNORE
#ifndef TRUESPEED_H
#define TRUESPEED_H

/**
 * TrueSpeed lookup table maps linear motor input to logarithmic motor output in order
 * to improve motor control
 */
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

#ifndef NERD_PID_h
#define NERD_PID_h

/**
 * PID controller data structure
 */

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float innerIntegralBand;
	float outerIntegralBand;
	float sigma;
	float lastValue;
	unsigned long lastTime;
	float lastSetPoint;
} PID;

#endif

#ifndef PID_C
#define PID_C

/// \endcond
/**
 * initialize pid structure, set parameters
 *
 * @param pid instance of PID structure
 * @param Kp  PID Kp constant
 * @param Ki  PID Ki constant
 * @param Kd  PID Kd constant
 * @param innerIntegralBand  inner bound of PID I summing cutoff
 * @param outerIntegralBand  outer bound of PID I summing cutoff
 */
void
pidInit (PID pid, float Kp, float Ki, float Kd, float innerIntegralBand, float outerIntegralBand) {
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
	pid.innerIntegralBand = innerIntegralBand;
	pid.outerIntegralBand = outerIntegralBand;
	pid.sigma = 0;
	pid.lastValue = 0;
	pid.lastTime = nPgmTime;
}

/**
 * initialize pid structure, set parameters based on another PID structure
 *
 * @param pid  instance of PID structure
 * @param toCopy  PID instance to copy settings from
 */
void pidInitCopy (PID pid, PID toCopy) {
	pid.Kp = toCopy.Ki;
	pid.Ki = toCopy.Ki;
	pid.Kd = toCopy.Kd;
	pid.innerIntegralBand = toCopy.innerIntegralBand;
	pid.outerIntegralBand = toCopy.outerIntegralBand;
	pid.sigma = 0;
	pid.lastValue = 0;
	pid.lastTime = nPgmTime;
}

/**
 * calculate pid output
 *
 * @param pid  instance of PID structure
 * @param setPoint  set point of PID controller
 * @param processVariable  sensor/feedback value
 *
 * @return  output value of the control loop
 */
float
pidCalculate (PID pid, int setPoint, int processVariable) {
	float deltaTime = (nPgmTime - pid.lastTime)/1000.0;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	if(deltaTime > 0)
		deltaPV = (processVariable - pid.lastValue) / deltaTime;
	pid.lastValue = processVariable;

	float error = setPoint - processVariable;

	if(fabs(error) > pid.innerIntegralBand && fabs(error) < pid.outerIntegralBand)
		pid.sigma += error * deltaTime;

	if (fabs (error) > pid.outerIntegralBand)
		pid.sigma = 0;

	float output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

	return output;
}

/**
 * calculate PID output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error
 *
 * @param pid  the PID controller to use for the calculation
 * @param setPoint  the set point of the system
 * @param processVariable  the value of the feedback sensor in the system
 * @param velocitySet  the velocity set point of the system
 *
 * @return  the output value of the control loop
 */
float
pidCalculateWithVelocitySet (PID pid, int setPoint, int processVariable, int velocitySet) {
	float deltaTime = (nPgmTime - pid.lastTime)/1000.0;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	if(deltaTime > 0)
		deltaPV = (processVariable - pid.lastValue) / deltaTime + velocitySet;
	pid.lastValue = processVariable;

	float error = setPoint - processVariable;

	if(fabs(error) > pid.innerIntegralBand && fabs(error) < pid.outerIntegralBand)
		pid.sigma += error * deltaTime;

	if (fabs (error) > pid.outerIntegralBand)
		pid.sigma = 0;

	float output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

	return output;
}

/**
 * calculate PID output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.
 *
 * @param pid  the PID controller to use for the calculation
 * @param setPoint  the set point of the system
 * @param processVariable  the value of the feedback sensor in the system
 *
 * @return  the output value of the control loop
 */
float
pidCalculateVelocity (PID pid, int setPoint, int processVariable) {
	float deltaTime = (nPgmTime - pid.lastTime)/1000.0;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	if(deltaTime > 0)
		deltaPV = (processVariable - pid.lastValue) / deltaTime;
	pid.lastValue = processVariable;

	float error = setPoint - processVariable;

	if(fabs(error) > pid.innerIntegralBand && fabs(error) < pid.outerIntegralBand)
		pid.sigma += error * deltaTime;

	if (fabs (error) > pid.outerIntegralBand)
		pid.sigma = 0;

	float output = setPoint * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

	return output;
}
#endif


/// \cond IGNORE
#ifndef NERD_MOTIONPLANNER
#define NERD_MOTIONPLANNER

#define SETTING_PWM 0x0
#define SETTING_1D_SHORT_MOVE 0x1
#define SETTING_VELOCITY 0x2
#define SETTING_1D_MOVE 0x3
#define SETTING_FOLLOW 0x4

struct motionProfiler {
	PID positionController;
	PID velocityController;

	float Kv;
	float Ka;
	float Kf;
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
	float tMax; //total estimated time of profile

	float cycleTime;
	int positionCycles; //amount of cycles to wait before new position update
	motionProfiler *toFollow;
};

motionProfiler profilerPool[10]; //  because of ROBOTC not being true C we need to allocate space for profilers at compile time instead of instantiating them as we need them
motionProfiler* motorController [10];
motionProfiler* uniqueControllers [10];

//sensor variable
int rawSensorValue [20];
/// \endcond


/**
 * return a pointer to a ROBOTC sensor
 *
 * @param port  the sensor value to get a pointer to
 *
 * @return  the pointer to the sensor value
 */
int*
getRawSensor (int port) {
	if (port < 0 || port > 19)
		return NULL;
	return &rawSensorValue [port];
}

/**
 * create a motion profile for a motor/sensor pair. PID controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/vMax
 *
 * @param motorPort  the motor port to create a profile for
 * @param sensor  a pointer to the sensor value to monitor. This can be a pointer to any integer, or a "raw" sensor value using getRawSensor().
 * @param vMax  the maximum velocity to use when calculating moves
 * @param Ka  acceleration constant used when ramping up/down velocity during moves
 * @param t1  time to spend at peak acceleration at the beginning/end of a move. This and jerkLimit will determine the shape of the motion curve
 * @param jerkLimit  the amount to limit the jerk by (0.0 to 1.0). The higher this value is, the more curved the acceleration profile will be. This should usually be 0.5
 * @param cycleTime  polling rate/sample period of system. Polling frequency = 1000/cycleTime. Note that ports 2-9 on cortex only can update at a frequency of 18.5Hz, so values less than ~20 here will offer diminishing returns.
 * @param positionCycles  cycles to skip for position updates during moves. This will generally be 3-5
 */
void
createMotionProfile (int motorPort, int *sensor, int vMax, float Ka, int t1, float jerkLimit, int cycleTime, int positionCycles) {
	if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort] != NULL)
		return;

	if (jerkLimit > 1.0)
		jerkLimit = 1.0;
	if (jerkLimit < 0)
		jerkLimit = 0;

	int t2 = t1/2.0 * jerkLimit;
	t2 -= t2 % cycleTime;

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
	controller->profileSetting = SETTING_PWM;
	controller->cycleCounter = 0;
	controller->motorOutput = 0;
	controller->lastSensorValue = *sensor;
	controller->lastTime = nPgmTime;
	controller->vMax = vMax;
	controller->positionOut = 0;
	controller->Ka = Ka;
	controller->Kf = 0.0;

	controller->velocityFilter[0] = 0;
	controller->velocityFilter[1] = 0;
	controller->velocityFilter[2] = 0;
	controller->velocityFilter[3] = 0;
	controller->velocityFilter[4] = 0;

	controller->t1 = t1;
	controller->t2 = t2;

	controller->cycleTime = cycleTime;
	controller->positionCycles = positionCycles;

	pidInit (controller->positionController, 0, 0, 0, 30, 150);
	pidInit (controller->velocityController, 127.0/vMax, 0, 0, 50, 500);
}

/**
 * create a motion profile for a motor/sensor pair using default timing settings and a Ka of 0. This will provide simple feedforward position/velocity control
 *
 * @param motorPort  the motor port to create a profile for
 * @param sensor  a pointer to the sensor value to monitor. This can be a pointer to any integer, or a "raw" sensor value using getRawSensor().
 * @param vMax  the maximum velocity to use when calculating moves
 */
void
createDefaultMotionProfile (int motorPort, int *sensor, int vMax) {
	createMotionProfile (motorPort, sensor, vMax, 0.0, 600, 0.5, 25, 4);
}

/**
 * set the position PID controller for the specified motor's motion profile
 *
 * @param motorPort  the motor to update
 * @param Kp  proportional gain
 * @param Ki  integral gain
 * @param Kd  derivative gain
 * @param innerBand  the inner integral deadBand value
 * @param outerBand  the outer integral deadBand value
 */
void
setPositionController (int motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController [motorPort];
	pidInit (profile->positionController, Kp, Ki, Kd, innerBand, outerBand);
}

/**
 * set the velocity PID controller for the specified motor's motion profile
 *
 * @param motorPort  the motor to update
 * @param Kp  proportional gain
 * @param Ki  integral gain
 * @param Kd  derivative gain
 * @param innerBand  the inner integral deadBand value
 * @param outerBand  the outer integral deadBand value
 */
void
setVelocityController (int motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController [motorPort];
	pidInit (profile->velocityController, Kp, Ki, Kd, innerBand, outerBand);
}


/**
 * set a motor to copy another motor's motion profile and mirror its output value
 *
 * @param motorPort  the motor to have mirror another motor
 * @param masterPort  the motor to mirror
 */
void
setMotionSlave (int motorPort, int masterPort) {
	if (motorController [masterPort] == NULL)
		return;
	motorController [motorPort] = motorController [masterPort];
}

/**
 * set a motion profile's follow constant
 *
 * @param motorPort  the motor to update
 * @param Kf  the follow constant value
 */
void
setKf (int motorPort, float Kf) {
	motorController [motorPort]->Kf = Kf;
}

/**
 * set a motor to follow another motor's output while attempting to match the value of the sensor paired with the target motor.
 *
 * @param motorPort  the follower motor
 * @param leaderPort  the motor to follow
 */
void
followMotor (int motorPort, int leaderPort) {
	if (motorController [leaderPort] == NULL || motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->toFollow = motorController [leaderPort];
	motorController [motorPort]->profileSetting = SETTING_FOLLOW;
}

/**
 * issue a move command to the specified position. This will currently do nothing if the move is considered to be a "short move", ie. the motor is unable to fully ramp up to max velocity during the move. Note that this is an absolute position command, so two consecutive moves to 4000 are not equivalent to a single move to 8000.
 *
 * @param motorPort  the motor to issue the move command to
 * @param position  the position to move to
 */
void
setPosition (int motorPort, int position) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController [motorPort];

	int distance = position - *(profile->sensor);

	profile->t4 = 1000 * fabs (distance) / profile->vMax;

	if (profile->t4 < profile->t1 + profile->t2) {
		profile->profileSetting = SETTING_1D_SHORT_MOVE;
		profile->finalPosition = position;
		profile->tMax = 2000.0 * fabs(profile->finalPosition) / profile->vMax;
		profile->jerk = sgn (distance);
		profile->accelSet = 0;
		profile->velocitySet = 0;
		profile->positionSet = 0;

		profile->motorOutput = 0;
		profile->cycleCounter = 0;
		profile->planComplete = 0;
		profile->positionController.lastTime = nPgmTime;
		profile->velocityController.lastTime = nPgmTime;
		profile->positionController.lastValue = *(profile->sensor);
		profile->velocityController.lastValue = profile->velocityRead;
	} else {
		profile->profileSetting = SETTING_1D_MOVE;
		profile->finalPosition = position;
		profile->tMax = profile->t1 + profile->t4;
	
		if (profile->t2 != 0) 
			profile->jerk = sgn (distance) * profile->vMax/((profile->t1 - profile->t2)/1000.0)/(profile->t2/1000.0);
		else
			profile->jerk = sgn (distance) * profile->vMax/((profile->t1 - profile->t2)/1000.0);

		profile->accelSet = 0;
		profile->velocitySet = 0;
		profile->positionSet = 0;

		profile->motorOutput = 0;
		profile->cycleCounter = 0;
		profile->planComplete = 0;
		profile->positionController.lastTime = nPgmTime;
		profile->velocityController.lastTime = nPgmTime;
		profile->positionController.lastValue = *(profile->sensor);
		profile->velocityController.lastValue = profile->velocityRead;
	}
}

/**
 * set a motor's output to a value from -127 to 127
 *
 * @param motorPort  the motor to set the output value of
 * @param output  output value to set
 */
void
setPWMOutput (int motorPort, int output) {
	if (motorController [motorPort] == NULL)
		return;

	if (output > 127)
		output = 127;
	if (output < -127)
		output = -127;

	motorController[motorPort]->profileSetting = SETTING_PWM;
	motorController[motorPort]->motorOutput = output;
}

/**
 * set a motor's velocity to the specified value
 *
 * @param motorPort  the motor to set
 * @param velocity  desired velocity
 */
void
setVelocity (int motorPort, int velocity) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController[motorPort];
	profile->profileSetting = SETTING_VELOCITY;
	profile->velocitySet = velocity;
	profile->velocityController.lastTime = nPgmTime;
	profile->velocityController.lastValue = profile->velocityRead;
}

/// @private
void
updateMotors () {
	int i;

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

/// @private
void
measureVelocity (motionProfiler *profile) {
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
}

/// @private
void
velocityUpdate (motionProfiler *profile) {
	//do velocity PID
	float velocityOut = pidCalculateVelocity (profile->velocityController, profile->velocitySet, profile->velocityRead);

	//set motor PWM output
	profile->motorOutput = velocityOut;

	if (profile->motorOutput > 127)
		profile->motorOutput = 127;
	else if (profile->motorOutput < -127)
		profile->motorOutput = -127;
}

/// @private
void
followerUpdate (motionProfiler *profile) {
	int error = *(profile->toFollow->sensor) - *(profile->sensor);
	profile->motorOutput = profile->toFollow->motorOutput + error * profile->Kf;

	if (profile->motorOutput > 127)
		profile->motorOutput = 127;
	if (profile->motorOutput < -127)
		profile->motorOutput = -127;
}

/// @private
void
positionUpdate (motionProfiler *profile) {
	//get motion profile output
	if (profile->planComplete == 0) {
		if (profile->t2 != 0) {
			if (profile->cycleCounter < profile->t2 / profile->cycleTime) {
				//J+
				profile->accelSet += profile->jerk * (profile->cycleTime/1000.0);
			} else if (profile->cycleCounter >= (profile->t1 - profile->t2) / profile->cycleTime && profile->cycleCounter < profile->t1 / profile->cycleTime) {
				//J-
				profile->accelSet -= profile->jerk * (profile->cycleTime/1000.0);
			} else if (profile->cycleCounter >= profile->t4 / profile->cycleTime && profile->cycleCounter < (profile->t4 + profile->t2) / profile->cycleTime) {
				//J-
				profile->accelSet -= profile->jerk * (profile->cycleTime/1000.0);
			} else if (profile->cycleCounter >= (profile->t4 + profile->t1 - profile->t2) / profile->cycleTime && profile->cycleCounter < profile->tMax) {
				//J+
				profile->accelSet += profile->jerk * (profile->cycleTime/1000.0);
			}
		} else {
			if (profile->cycleCounter < profile->t1 / profile->cycleTime) {
				profile->accelSet = profile->jerk;
			} else if (profile->cycleCounter > profile->t4 / profile->cycleTime && profile->cycleCounter < profile->tMax / profile->cycleTime) {
				profile->accelSet = -1 * profile->jerk;
			} else {
				profile->accelSet = 0;
			}
		}
		if (profile->cycleCounter < profile->tMax / profile->cycleTime) {
			profile->velocitySet += profile->accelSet * (profile->cycleTime/1000.0);

			profile->positionSet += profile->velocitySet * (profile->cycleTime/1000.0);
			if (fabs(profile->positionSet) > fabs(profile->finalPosition))
				profile->positionSet = profile->finalPosition;
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
	 	profile->positionOut = pidCalculateWithVelocitySet (profile->positionController, profile->positionSet, *(profile->sensor), profile->velocitySet);
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
}

/// @private
void
shortPositionUpdate (motionProfiler *profile) {
	//get motion profile output
	if (profile->planComplete == 0) {
		if (profile->cycleCounter * profile->cycleTime < profile->tMax/2.0) {
			profile->accelSet = sgn (profile->jerk) * (profile->vMax*2.0/(profile->tMax/1000.0));
		} else if (profile->cycleCounter * profile->cycleTime < profile->tMax) {
			profile->accelSet = sgn (profile->jerk) * (-1.0*profile->vMax*2.0/(profile->tMax/1000.0));
		}

		if (profile->cycleCounter < profile->tMax / profile->cycleTime) {
			profile->velocitySet += profile->accelSet * (profile->cycleTime/1000.0);
			profile->positionSet += profile->velocitySet * (profile->cycleTime/1000.0);

			if ((profile->jerk < 0 && profile->positionSet < profile->finalPosition) || (profile->jerk > 0 && profile->positionSet > profile->finalPosition))
				profile->positionSet = profile->finalPosition;
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
	 	profile->positionOut = pidCalculateWithVelocitySet (profile->positionController, profile->positionSet, *(profile->sensor), profile->velocitySet);
	}

	//do velocity PID
	float velocityOut = pidCalculateVelocity (profile->velocityController, profile->positionOut + profile->velocitySet, profile->velocityRead) + profile->accelSet * profile->Ka;

	//set motor PWM output
	profile->motorOutput = velocityOut;

	if (profile->motorOutput > 127)
		profile->motorOutput = 127;
	else if (profile->motorOutput < -127)
		profile->motorOutput = -127;
}

/// @private
task rawSensorMonitor () {
	int i;

	while (1) {
		for (i = 0; i < 20; ++i) {
			rawSensorValue [i] = SensorValue [i];
		}
	}

	abortTimeslice ();
}

/// @private
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

			measureVelocity (profile);

			if (profile->profileSetting == SETTING_1D_MOVE) {
				positionUpdate (profile);
			} else if (profile->profileSetting == SETTING_1D_SHORT_MOVE) {
				shortPositionUpdate (profile);
			} else if (profile->profileSetting == SETTING_VELOCITY) {
				velocityUpdate (profile);
			} else if (profile->profileSetting == SETTING_FOLLOW) {
				followerUpdate (profile);
			}
		}

		updateMotors ();
	}

	abortTimeslice ();
}

#endif
