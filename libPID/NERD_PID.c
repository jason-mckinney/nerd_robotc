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