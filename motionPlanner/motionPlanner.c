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

#ifndef NERD_PID
#define NERD_PID

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

#define MOVE_BUFFER_SIZE 10

#define SETTING_INACTIVE 0x0
#define SETTING_ACTIVE 0x1
#define SETTING_ACTIVEPOSITION 0x2
#define SETTING_MIRROR_REVERSE 0x3

struct Move {
	long startTime;
	long timeLimit;
	float targetVelocity;
	char moveNotExecuted;
}

struct motionProfiler {
	PID positionController;
	PID velocityController;

	float Kv;
	float Ka;
	float jerkLimit;
	void *sensor;
	int velocityFilter [5];
	int velocityRead;

	char profileSetting;
	short motorOutput;
	float positionOut;
	float lastSensorValue;
	long lastMeasureTime;
	long lastComputeTime;
	char cycleCounter;
	long moveStartTime;
	float aMax;
	float positionTarget;

	float velocityTarget;

	float positionSet; //position set point
	float velocitySet; //velocity set point
	float accelSet; //acceleration output
	float jerk; //rate of acceleration change

	float vMax; //max rate of system, in sensor units/second
	int accelTime; //time for velocity ramping
	int following; //motor port to mirror

	// + jerk
	long t1; //time to set jerk to 0
	// 0 jerk
	long t2; //time to set jerk to decelerate (accelSet -= jerk)
	// - jerk
	long t3; //time to set jerk to 0
	// 0 jerk

	Move moveBuffer [MOVE_BUFFER_SIZE]; //random access buffer for queueing move commands

	float cycleTime;
	int positionCycles; //amount of cycles to wait before new position update
};

void setPositionController (int motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand);
void setVelocityController (int motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand);

motionProfiler profilerPool[10]; //  because of ROBOTC not being true C we need to allocate space for profilers at compile time instead of instantiating them as we need them
motionProfiler* motorController [10];
motionProfiler* uniqueControllers [10];

//sensor variable
int rawSensorValue [20];
/// \endcond

float
convert_to_float (void *var) {
	int *i = var;
	float *f = var;

	if (*i == *var)
		return *var;

	return *f;
}

void
queueMove (motionProfiler *profile, long startTime, float targetVelocity) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->moveBuffer [i].moveNotExecuted == 0) {
			profile->moveBuffer [i].startTime = startTime;
			profile->moveBuffer [i].targetVelocity = targetVelocity;
			profile->moveBuffer [i].moveNotExecuted = 1;
			profile->moveBuffer [i].timeLimit = profile->accelTime;
			return;
		}
	}
}

void
queueMoveWithTimeLimit (motionProfiler *profile, long startTime, float targetVelocity, long timeLimit) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->moveBuffer [i].moveNotExecuted == 0) {
			profile->moveBuffer [i].startTime = startTime;
			profile->moveBuffer [i].targetVelocity = targetVelocity;
			profile->moveBuffer [i].moveNotExecuted = 1;
			profile->moveBuffer [i].timeLimit = timeLimit;
			return;
		}
	}
}

char
hasMoveQueued (motionProfiler *profile) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i)
		if (profile->moveBuffer[i].moveNotExecuted == 1)
			return 1;

	return 0;
}

void
clearMoveQueue (motionProfiler *profile) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->moveBuffer [i].moveNotExecuted)
			profile->moveBuffer [i].moveNotExecuted = 0;
	}
}

/**
 * return a pointer to a ROBOTC sensor
 *
 * @param port  the sensor value to get a pointer to
 *
 * @return  the pointer to the sensor value
 */
int*
getSensorPointer (int port) {
	if (port < 0 || port > 19)
		return NULL;
	return &rawSensorValue [port];
}

/**
 * create a motion profile for a motor/sensor pair. PID controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/vMax
 *
 * @param motorPort  the motor port to create a profile for
 * @param sensor  a pointer to the sensor value to monitor. This can be a pointer to any integer, or a "raw" sensor value using getSensorPointer().
 * @param vMax  the maximum velocity to use when calculating moves
 * @param Ka  acceleration constant used when ramping up/down velocity during moves
 * @param t1  time to spend at peak acceleration at the beginning/end of a move. This and jerkLimit will determine the shape of the motion curve
 * @param jerkLimit  the amount to limit the jerk by (0.0 to 1.0). The higher this value is, the more curved the acceleration profile will be. This should usually be 0.5
 * @param cycleTime  polling rate/sample period of system. Polling frequency = 1000/cycleTime. Note that ports 2-9 on cortex only can update at a frequency of 18.5Hz, so values less than ~20 here will offer diminishing returns.
 * @param positionCycles  cycles to skip for position updates during moves. This will generally be 3-5
 */
void
setMotionProfileCustom (int motorPort, void *sensor, float vMax, float Ka, int t1, float jerkLimit, int cycleTime, int positionCycles) {
	if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort] != NULL)
		return;

	if (jerkLimit > 1.0)
		jerkLimit = 1.0;
	if (jerkLimit < 0)
		jerkLimit = 0;

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
	controller->jerkLimit = jerkLimit;
	controller->sensor = sensor;
	controller->velocityRead = 0;
	controller->profileSetting = SETTING_INACTIVE;
	controller->motorOutput = 0;
	controller->lastSensorValue = convert_to_float(sensor);
	controller->lastMeasureTime = nPgmTime;
	controller->lastComputeTime = nPgmTime;
	controller->vMax = vMax;
	controller->positionOut = 0;
	controller->Ka = Ka;

	controller->velocityFilter[0] = 0;
	controller->velocityFilter[1] = 0;
	controller->velocityFilter[2] = 0;
	controller->velocityFilter[3] = 0;
	controller->velocityFilter[4] = 0;

	controller->accelTime = t1;
	controller->t1 = 0;
	controller->t2 = 0;
	controller->t3 = 0;

	controller->cycleTime = cycleTime;
	controller->positionCycles = positionCycles;

	pidInit (controller->positionController, 0, 0, 0, 30, 150);
	pidInit (controller->velocityController, 127.0/vMax, 0, 0, 50, 500);
}

void
setMotionProfile (int motorPort, tSensors sensor, float vMax, float Ka, int t1, float jerkLimit, int cycleTime, int positionCycles) {
	if (sensor < in1 || sensor > dgtl12)
		return;

	int *sensor_pointer = getSensorPointer(sensor);
	setMotionProfileCustom(motorPort, sensor_pointer, vMax, Ka, t1, jerkLimit, cycleTime, positionCycles);
}

void
setSimpleMotionProfile (int motorPort, tSensors sensor, float vMax) {
	if (sensor < in1 || sensor > dgtl12)
		return;

	int *sensor_pointer = getSensorPointer(sensor);
	setMotionProfileCustom (motorPort, sensor_pointer, vMax, 0.0, 600, 0.5, 20, 4);
	setVelocityController (motorPort, 127.0/vMax, 0.0, 0.0, 50, 400);
	setPositionController (motorPort, 5.0, 0.0, 0.0, 30, 150);
}

void
setSimpleMotionProfileCustom (int motorPort, void *sensor, float vMax) {
	setMotionProfileCustom (motorPort, sensor, vMax, 0.0, 600, 0.5, 20, 4);

	setVelocityController (motorPort, 127.0/vMax, 0.0, 0.0, 50, 400);
	setPositionController (motorPort, 5.0, 0.0, 0.0, 30, 150);
}

void
updateMotionProfileCustom (int motorPort, void *sensor, float vMax, float Ka, int t1, float jerkLimit, int cycleTime, int positionCycles) {
	if (motorPort < 0 || motorPort > 9)
		return;
	if (motorController[motorPort] == NULL)
		return;

	motionProfiler *controller = motorController[motorPort];

	if (jerkLimit > 1.0)
		jerkLimit = 1.0;
	if (jerkLimit < 0)
		jerkLimit = 0;

	controller->sensor = sensor;
	controller->vMax = vMax;
	controller->Ka = Ka;
	controller->accelTime = t1;
	controller->cycleTime = cycleTime;
	controller->jerkLimit = jerkLimit;
	controller->positionCycles = positionCycles;
}

void
updateMotionProfile (int motorPort, tSensors sensor, float vMax, float Ka, int t1, float jerkLimit, int cycleTime, int positionCycles) {
	if (sensor < in1 || sensor > dgtl12)
		return;

	int *sensor_pointer = getSensorPointer(sensor);

	updateMotionProfileCustom(motorPort, sensor_pointer, vMax, Ka, t1, jerkLimit, cycleTime, positionCycles);
}

void
profileSetVMax (int motorPort, float vMax) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->vMax = vMax;
	motorController [motorPort]->velocityController.Kp = 127.0 / vMax;
}

void
profileSetKa (int motorPort, float Ka) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->Ka = Ka;
}

void
profileSetAccelTime (int motorPort, int t1) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->accelTime = t1;
}

void
profileSetCycleTime (int motorPort, int cycleTime) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->cycleTime = cycleTime;
}

void
profileSetPositionFrequency (int motorPort, int positionCycles) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->positionCycles = positionCycles;
}

void
profileSetJerkLimit (int motorPort, float jerkLimit) {
	if (motorController [motorPort] == NULL)
		return;

	if (jerkLimit > 1.0)
		jerkLimit = 1.0;
	else if (jerkLimit < 0.0)
		jerkLimit = 0.0;

	motorController [motorPort]->jerkLimit = jerkLimit;
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

void
setMotionSlaveReversed (int motorPort, int masterPort) {
	if (motorController [masterPort] == NULL)
		return;

	if (motorController[motorPort] == NULL)
		setSimpleMotionProfileCustom (motorPort, motorController[masterPort]->sensor, motorController[masterPort]->vMax);

	motorController[motorPort]->following = masterPort;
	motorController[motorPort]->profileSetting = SETTING_MIRROR_REVERSE;
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

	float distance = position - convert_to_float(profile->sensor);
	float initialVelocity = profile->velocityRead;
	float velocityError = sgn (distance) * profile->vMax - initialVelocity;
	float rampUpTime = fabs((sgn(distance) * profile->vMax - initialVelocity)/profile->vMax * profile->accelTime);
	float rampUpDist = 0.0005 * (rampUpTime-profile->accelTime) * initialVelocity + profile->accelTime / 2000.0 * profile->vMax * sgn (velocityError);
	float rampDownDist = profile->accelTime * profile->vMax / 2000.0 * sgn (velocityError);
	float cruiseTime = sgn (distance) * (distance - rampUpDist - rampDownDist) / profile->vMax * 1000.0;
	long decelTime = cruiseTime + rampUpTime + nPgmTime;

	clearMoveQueue(profile);
	queueMoveWithTimeLimit (profile, nPgmTime, sgn(distance) * profile->vMax, decelTime - nPgmTime);
	queueMoveWithTimeLimit (profile, decelTime, 0, decelTime - nPgmTime);
	profile->profileSetting = SETTING_ACTIVEPOSITION;
	profile->positionTarget = position;
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

	clearMoveQueue (motorController[motorPort]);
	motorController[motorPort]->profileSetting = SETTING_INACTIVE;
	motorController[motorPort]->motorOutput = output;
	motorController[motorPort]->velocitySet = 0;
	motorController[motorPort]->positionTarget = 0;
	motorController[motorPort]->positionSet = 0;
}

/**
 * set a motor's velocity to the specified value
 *
 * @param motorPort  the motor to set
 * @param velocity  desired velocity
 */
void
setVelocity (int motorPort, float velocity) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfiler *profile = motorController[motorPort];
	profile->profileSetting = SETTING_ACTIVE;
	profile->positionSet = convert_to_float (profile->sensor);
	profile->positionTarget = convert_to_float (profile->sensor);

	clearMoveQueue (profile);
	queueMove (profile, nPgmTime, velocity);
}

/// @private
void
updateMotors () {
	int i;

	for (i = 0; i < 10; ++i) {
		if (motorController [i] == NULL)
			continue;

		if (motorController[i]->profileSetting == SETTING_MIRROR_REVERSE)
			motorController[i]->motorOutput = -motorController[motorController[i]->following]->motorOutput;

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
	float deltaT = (nPgmTime - profile->lastMeasureTime)/1000.0;
	int sensorV = convert_to_float (profile->sensor);

	//get sensor velocity, ticks per second
	float sensorRate = deltaT == 0 ? 0 : (sensorV - profile->lastSensorValue) / deltaT;

	for (int j = 4; j > 0; --j) {
		profile->velocityFilter [j] = profile->velocityFilter [j-1];
	}
	profile->velocityFilter [0] = sensorRate;

	sensorRate = profile->velocityFilter [0] * 0.5 + profile->velocityFilter [1] * 0.25 + profile->velocityFilter [2] * 0.125 + profile->velocityFilter [3] * 0.0625 + profile->velocityFilter [4] * 0.0625;
	profile->velocityRead = sensorRate;
	profile->lastSensorValue = sensorV;
}

void
startMove (motionProfiler *profile, float targetVelocity, long timeLimit) {
	profile->velocityTarget = targetVelocity;
	float velocityError = targetVelocity - profile->velocityRead;
	long rampUpTime = fabs(velocityError / profile->vMax * profile->accelTime);

	if (rampUpTime > timeLimit)
		rampUpTime = timeLimit;

	long jerkTime = rampUpTime / 2.0 * profile->jerkLimit;

	if (rampUpTime < jerkTime) {
		jerkTime = 0;
	}

	if (rampUpTime != 0)
		profile->aMax = velocityError / (rampUpTime - jerkTime) * 1000.0;
	else
		profile->aMax = 0;

	if (jerkTime > 0)
		profile->jerk = profile->aMax / jerkTime * 1000.0;
	else
		profile->jerk = 0;

	profile->t1 = jerkTime;
	profile->t2 = rampUpTime - jerkTime;
	profile->t3 = rampUpTime;

	profile->moveStartTime = -1;
}

/// @private
void
profileUpdate (motionProfiler *profile) {
	if (profile->moveStartTime == -1)
		profile->moveStartTime = nPgmTime;

	float moveTime = nPgmTime - profile->moveStartTime;
	float deltaTime = nPgmTime - profile->lastComputeTime;

	if (profile->lastComputeTime < profile->moveStartTime) {
		deltaTime -= profile->moveStartTime - profile->lastMeasureTime;
		if (deltaTime < 0)
			deltaTime = 0;
	}
	profile->lastComputeTime = nPgmTime;

	if (nPgmTime < (unsigned long) (profile->moveStartTime + profile->t1)) { // t0
		profile->accelSet = profile->jerk * moveTime / 1000.0;
	} else if (nPgmTime > (unsigned long) (profile->moveStartTime + profile->t1) && nPgmTime < (unsigned long) (profile->moveStartTime + profile->t2)) { // t1
		profile->accelSet = profile->aMax;
	} else if (nPgmTime > (unsigned long) (profile->moveStartTime + profile->t2) && nPgmTime < (unsigned long) (profile->moveStartTime + profile->t3)) { // t2
		profile->accelSet = profile->aMax - profile->jerk * (moveTime - profile->t2) / 1000.0;
	} else if (nPgmTime > (unsigned long) (profile->moveStartTime + profile->t3)) { // t3
		profile->accelSet = 0;
		profile->velocitySet = profile->velocityTarget;
	}

	profile->velocitySet += profile->accelSet * deltaTime/1000.0;
	profile->positionSet += profile->velocitySet * deltaTime/1000.0;

	//do position PID if cycle includes it
	if (profile->cycleCounter % profile->positionCycles == 0) {
	 	profile->positionOut = pidCalculateWithVelocitySet (profile->positionController, profile->positionSet, convert_to_float (profile->sensor), profile->velocitySet);
	}
	profile->cycleCounter++;

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

			if (nPgmTime - profile->lastMeasureTime < profile->cycleTime) {
				continue;
			}

			measureVelocity (profile);
			profile->lastMeasureTime = nPgmTime;

			if (profile->profileSetting == SETTING_ACTIVE || profile->profileSetting == SETTING_ACTIVEPOSITION) {
				for (int j = 0; j < MOVE_BUFFER_SIZE; ++j) {
					if (profile->moveBuffer[j].moveNotExecuted && (unsigned long) (profile->moveBuffer[j].startTime) < nPgmTime) {
						startMove (profile, profile->moveBuffer[j].targetVelocity, profile->moveBuffer[j].timeLimit);
						profile->moveBuffer[j].moveNotExecuted = 0;

						break;
					}
				}

				if (profile->profileSetting == SETTING_ACTIVEPOSITION && (unsigned long) profile->t3 < nPgmTime && profile->velocitySet == 0 && !hasMoveQueued(profile))
					profile->positionSet = profile->positionTarget;

				profileUpdate (profile);
 			}
		}

		updateMotors ();
	}

	abortTimeslice ();
}

#endif
