#ifndef NERD_GYRO
#define NERD_GYRO


struct gyroConfig{
	float stdDev;
	float avg;
	float voltsPerDPS;
	char gyroFlipped;
};

typedef struct {
	struct gyroConfig config;
	int portNum;

	#ifdef NERD_MOTIONPLANNER
	long lastGyroRead;
	int angle;
	int negativeAngle;
	float floatAngle;
	motionProfiler lastLeftProfile;
	motionProfiler lastRightProfile;
	motionProfiler leftProfile;
	motionProfiler rightProfile;
	#endif
} Gyro;

//ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 3

#define GYRO_OVERSAMPLE 1

//points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 2000

float calibrationBuffer [GYRO_CALIBRATION_POINTS];

float gyroGetRate (Gyro gyro);

/**
 * Gyro motion planner add on
 */
#ifdef NERD_MOTIONPLANNER
Gyro *turningGyro;
int leftTurnPort;
int rightTurnPort;

void setupGyroPositionController (Gyro gyro, float Kp, float Ki, float Kd, int innerBand, int outerBand){
	pidInit (gyro.leftProfile.positionController, Kp, Ki, Kd, innerBand, outerBand);
	pidInit (gyro.rightProfile.positionController, Kp, Ki, Kd, innerBand, outerBand);
}

void setupGyroVelocityController (Gyro gyro, float Kp, float Ki, float Kd, int innerBand, int outerBand){
	pidInit (gyro.leftProfile.velocityController, Kp, Ki, Kd, innerBand, outerBand);
	pidInit (gyro.rightProfile.velocityController, Kp, Ki, Kd, innerBand, outerBand);
}

void setupGyroMotionProfile (Gyro gyro, int vMax, float Ka, int t1, float jerkLimit, int cycleTime, int positionCycles) {
	if (jerkLimit > 1.0)
		jerkLimit = 1.0;
	if (jerkLimit < 0)
		jerkLimit = 0;

	int t2 = t1/2.0 * jerkLimit;
	t2 -= t2 % cycleTime;

	if (t1 % cycleTime != 0 || t2 % cycleTime != 0)
		return;

	gyro.angle = 0;
	gyro.negativeAngle = 0;
	gyro.floatAngle = 0.0;
	gyro.leftProfile.sensor = &(gyro.angle);
	gyro.leftProfile.velocityRead = 0;
	gyro.leftProfile.profileSetting = SETTING_PWM;
	gyro.leftProfile.cycleCounter = 0;
	gyro.leftProfile.motorOutput = 0;
	gyro.leftProfile.lastSensorValue = *(gyro.leftProfile.sensor);
	gyro.leftProfile.lastTime = nPgmTime;
	gyro.leftProfile.vMax = vMax;
	gyro.leftProfile.positionOut = 0;
	gyro.leftProfile.Ka = Ka;
	gyro.leftProfile.Kf = 0.0;

	gyro.leftProfile.velocityFilter[0] = 0;
	gyro.leftProfile.velocityFilter[1] = 0;
	gyro.leftProfile.velocityFilter[2] = 0;
	gyro.leftProfile.velocityFilter[3] = 0;
	gyro.leftProfile.velocityFilter[4] = 0;

	gyro.leftProfile.t1 = t1;
	gyro.leftProfile.t2 = t2;

	gyro.leftProfile.cycleTime = cycleTime;
	gyro.leftProfile.positionCycles = positionCycles;

	pidInit (gyro.leftProfile.positionController, 0, 0, 0, 30, 150);
	pidInit (gyro.leftProfile.velocityController, 127.0/vMax, 0, 0, 50, 500);

	gyro.rightProfile.sensor = &(gyro.negativeAngle);
	gyro.rightProfile.velocityRead = 0;
	gyro.rightProfile.profileSetting = SETTING_PWM;
	gyro.rightProfile.cycleCounter = 0;
	gyro.rightProfile.motorOutput = 0;
	gyro.rightProfile.lastSensorValue = *(gyro.rightProfile.sensor);
	gyro.rightProfile.lastTime = nPgmTime;
	gyro.rightProfile.vMax = vMax;
	gyro.rightProfile.positionOut = 0;
	gyro.rightProfile.Ka = Ka;
	gyro.rightProfile.Kf = 0.0;

	gyro.rightProfile.velocityFilter[0] = 0;
	gyro.rightProfile.velocityFilter[1] = 0;
	gyro.rightProfile.velocityFilter[2] = 0;
	gyro.rightProfile.velocityFilter[3] = 0;
	gyro.rightProfile.velocityFilter[4] = 0;

	gyro.rightProfile.t1 = t1;
	gyro.rightProfile.t2 = t2;

	gyro.rightProfile.cycleTime = cycleTime;
	gyro.rightProfile.positionCycles = positionCycles;

	pidInit (gyro.rightProfile.positionController, 0, 0, 0, 30, 150);
	pidInit (gyro.rightProfile.velocityController, 127.0/vMax, 0, 0, 50, 500);
}

task gyroTurnTask () {
	while (true) {
		float deltaTime = (nPgmTime - turningGyro->lastGyroRead)/1000.0;

		if (deltaTime > 0) {
			turningGyro->floatAngle += gyroGetRate (*turningGyro) * deltaTime;
			turningGyro->angle = turningGyro->floatAngle * 100;
			turningGyro->negativeAngle -= turningGyro->floatAngle * -100;
			turningGyro->lastGyroRead = nPgmTime;
		}

		delay (5);
	}
}

void
gyroTurn (Gyro gyro, int leftPort, int rightPort, float angle) {
	if (motorController [leftPort] == NULL && motorController [rightPort] == NULL)
		return;

	leftTurnPort = leftPort;
	rightTurnPort = rightPort;

	setPWMOutput (leftPort, 0);
	setPWMOutput (rightPort, 0);

	memcpy (&(gyro.lastLeftProfile), motorController [leftPort], sizeof (motionProfiler));
	memcpy (&(gyro.lastRightProfile), motorController [rightPort], sizeof (motionProfiler));
	memcpy (motorController [leftPort], &(gyro.leftProfile), sizeof (motionProfiler));
	memcpy (motorController [rightPort], &(gyro.rightProfile), sizeof (motionProfiler));

	gyro.angle = 0;
	gyro.negativeAngle = 0;
	gyro.floatAngle = 0;
	gyro.lastGyroRead = nPgmTime;
	turningGyro = &gyro;
	setPosition (leftPort, angle * 100);
	setPosition (rightPort, angle * -100);
	startTask (gyroTurnTask);
}

void
gyroTurnStop () {
	setPWMOutput (leftTurnPort, 0);
	setPWMOutput (rightTurnPort, 0);

	memcpy (motorController[leftTurnPort], &(turningGyro->lastLeftProfile), sizeof (motionProfiler));
	memcpy (motorController[rightTurnPort], &(turningGyro->lastRightProfile), sizeof (motionProfiler));

	stopTask (gyroTurnTask);
}
#endif


/**
 * generate calibration data for the gyro by collecting
 * zero movement data for reference when reading data later
 *
 * @param gyro instance of gyro structure
 */
void
gyroCalibrate (Gyro gyro){
	float rawAverage = 0.0;
	float stdDev = 0.0;

	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float raw = SensorValue (gyro.portNum);
		rawAverage += raw;
		calibrationBuffer [i] = raw;
		delay (1);
	}
	rawAverage /= GYRO_CALIBRATION_POINTS;
	gyro.config.avg = rawAverage;

	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		stdDev += fabs (rawAverage - calibrationBuffer [i]);
	stdDev /= (float) GYRO_CALIBRATION_POINTS;

	gyro.config.stdDev = stdDev;

	/*
	 * Datasheet from VEX indicates that the sensitivity of the gyro is 1.1mV/dps
	 * and the cortex ADC for raw analog reads ranges from 0-4095 for 0v-5v
	 * readings. The gyro is scaled from the nominal 2.7v-3.6v operating range
	 * that the actual chip has to work on the cortex's 5v logic voltage. The scale multiplier
	 * value is in the ballpark of 1.515.
	 */
	gyro.config.voltsPerDPS = 0.0011 * 1.515;
}

/**
 * initialize gyro and run the calibration subroutine
 *
 * @param gyro instance of gyro structure
 * @param portNum the port number of the gyro
 */
void
gyroInit (Gyro gyro, int portNum, char gyroFlipped) {
	gyro.portNum = portNum;
	gyro.config.gyroFlipped = gyroFlipped;
	gyroCalibrate (gyro);
}

/**
 * calculate filtered gyro rate data, ignoring anything within
 * GYRO_STD_DEVS standard deviations of the average gyro
 * rate value at zero motion
 *`
 * @param gyro instance of gyro structure
 *
 * @return gyro rate, in degrees per second
 */
float
gyroGetRate (Gyro gyro){
	float gyroRead = 0.0;

	#if defined (GYRO_OVERSAMPLE)
		if (GYRO_OVERSAMPLE > 0) {
			int sampleSum = 0;
			int nSamples = pow (4, GYRO_OVERSAMPLE);

			for (int i = 0; i < nSamples; ++i)
				sampleSum += SensorValue(gyro.portNum);
			gyroRead = (float) sampleSum / (float) nSamples;
		}
		else
			gyroRead = SensorValue (gyro.portNum);
	#else
		gyroRead = SensorValue (gyro.portNum);
	#endif

	//Difference from zero-rate value or the average calibration read
	float gyroDiff = gyroRead - gyro.config.avg;

	//Difference fro zero-rate value, in volts
	float gyroVoltage = gyroDiff * 5.0 / 4095.0;

	if (fabs (gyroDiff) > GYRO_STD_DEVS * gyro.config.stdDev)
		if (gyro.config.gyroFlipped)
			return -1 * gyroVoltage / gyro.config.voltsPerDPS;
		else
			return gyroVoltage / gyro.config.voltsPerDPS;
	return 0;
}

#endif
