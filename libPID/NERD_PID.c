/* -----------------------------------------------------------------------------
                                  MIT License

                        Copyright (c) 2018 Jason McKinney

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

--------------------------------------------------------------------------------
	NERD_PID.c

	Created:  2016-06-30
	
	Minor Revisions:
	-	v1.0.0  Initial Release

--------------------------------------------------------------------------------	
	The author asks that proper attribution be given for this software should the
	source be unavailable (for example, if compiled into a binary/used on a robot).

	The author can be contacted via email at jason_at_jmmckinney_dot_net
	or on the VEX Forum as jmmckinney.
-------------------------------------------------------------------------------- */


#ifndef NERD_PID
#define NERD_PID

/**
 * PID controller data structure
 */
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float sigma;
	float lastValue;
	unsigned long lastTime;
	float lastSetPoint;
} PID;

/**
 * initialize pid structure, set parameters
 *
 * pid instance of PID structure
 * Kp  proportional gain
 * Ki  integral gain
 * Kd  derivative gain
 * innerIntegralBand  inner bound of PID I summing cutoff
 * outerIntegralBand  outer bound of PID I summing cutoff
 */
void
pidInit (PID pid, float Kp, float Ki, float Kd) {
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
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
pidCalculate (PID pid, float setPoint, float processVariable) {
	float deltaTime = (nPgmTime - pid.lastTime)*0.001;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	
  if(deltaTime > 0) {
		deltaPV = (processVariable - pid.lastValue) / deltaTime;
  }
	
  pid.lastValue = processVariable;

	float error = setPoint - processVariable;

  float output = error * pid.Kp + pid.sigma * pid.Ki - deltaPV * pid.Kd;

	if (!(fabs(output) >= 1.0 && ((error >= 0 && pid.sigma >= 0) || (error < 0 && pid.sigma < 0)))) {
    pid.sigma += error * deltaTime;
  }

	output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

  if (output > 1.0) {
    output = 1.0;
  }

  if (output < -1.0) {
    output = -1.0;
  }

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
pidCalculateWithVelocitySet (PID pid, float setPoint, float processVariable, float velocitySet) {
	float deltaTime = (nPgmTime - pid.lastTime)*0.001;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	
  if(deltaTime > 0) {
		deltaPV = (processVariable - pid.lastValue) / deltaTime + velocitySet;
  }
	
  pid.lastValue = processVariable;

	float error = setPoint - processVariable;

  float output = error * pid.Kp + pid.sigma * pid.Ki - deltaPV * pid.Kd;

	if (!(fabs(output) >= 1.0 && ((error >= 0 && pid.sigma >= 0) || (error < 0 && pid.sigma < 0)))) {
    pid.sigma += error * deltaTime;
  }

	output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

  if (output > 1.0) {
    output = 1.0;
  }

  if (output < -1.0) {
    output = -1.0;
  }

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
pidCalculateVelocity (PID pid, float setPoint, float processVariable) {
	float deltaTime = (nPgmTime - pid.lastTime)*0.001;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	
  if(deltaTime > 0) {
		deltaPV = (processVariable - pid.lastValue) / deltaTime;
  }
	
  pid.lastValue = processVariable;

	float error = setPoint - processVariable;

  float output = error * pid.Kp + pid.sigma * pid.Ki - deltaPV * pid.Kd;

	if (!(fabs(output) >= 1.0 && ((error >= 0 && pid.sigma >= 0) || (error < 0 && pid.sigma < 0)))) {
    pid.sigma += error * deltaTime;
  }

	output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

  if (output > 1.0) {
    output = 1.0;
  }

  if (output < -1.0) {
    output = -1.0;
  }

	return output;
}
#endif