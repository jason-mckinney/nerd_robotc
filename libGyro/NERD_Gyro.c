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
} Gyro;

//ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 3

#define GYRO_OVERSAMPLE 1

//points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 2000

float calibrationBuffer [GYRO_CALIBRATION_POINTS];

float gyroGetRate (Gyro gyro);

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
