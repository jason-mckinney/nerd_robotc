#include "NERD_Gyro.h"

//ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 4

#define GYRO_OVERSAMPLE 2

//points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 2000

float rgfRaw[GYRO_CALIBRATION_POINTS];

/**
 * generate calibration data for the gyro by collecting
 * zero movement data for reference when reading data later
 *
 * @param gyro instance of gyro structure
 */
void
gyroCalibrate (Gyro gyro){
	float fRawAverage = 0.0;
	float fStdDev = 0.0;

	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float fRaw = SensorValue (gyro.m_iPortNum);
		fRawAverage += fRaw;
		rgfRaw [i] = fRaw;
		delay (1);
	}
	fRawAverage /= GYRO_CALIBRATION_POINTS;
	gyro.m_config.m_fAvg = fRawAverage;

	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		fStdDev += fabs (fRawAverage - rgfRaw [i]);
	fStdDev /= (float) GYRO_CALIBRATION_POINTS;

	gyro.m_config.m_fStdDev = fStdDev;

	/*
	 * Datasheet from VEX indicates that the sensitivity of the gyro is 1.1mV/dps
	 * and the cortex ADC for raw analog reads ranges from 0-4095 for 0v-5v
	 * readings. The gyro is scaled from the nominal 2.7v-3.6v operating range
	 * that the actual chip has to work on the cortex's 5v logic voltage. The scale multiplier
	 * value is in the ballpark of 1.515.
	 *///1.0608
	float zeroRate = fRawAverage * 5.0 / 4095.0;
	gyro.m_config.m_fVoltsPerDPS = (0.0011 * 1.515) * (2.2725 / zeroRate);
}

/**
 * initialize gyro and run the calibration subroutine
 *
 * @param gyro instance of gyro structure
 * @param iPortNum the port number of the gyro
 */
void
gyroInit (Gyro gyro, int iPortNum) {
	gyro.m_iPortNum = iPortNum;
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
	float fGyroRead = 0.0;

	#if defined (GYRO_OVERSAMPLE)
		if (GYRO_OVERSAMPLE > 0) {
			int sampleSum = 0;
			int nSamples = pow (4, GYRO_OVERSAMPLE);

			for (int i = 0; i < nSamples; ++i)
				sampleSum += SensorValue(gyro.m_iPortNum);
			fGyroRead = (float) sampleSum / (float) nSamples;
		}
		else
			fGyroRead = SensorValue (gyro.m_iPortNum);
	#else
		fGyroRead = SensorValue (gyro.m_iPortNum);
	#endif

	//Difference from zero-rate value or the average calibration read
	float fGyroDiff = fGyroRead - gyro.m_config.m_fAvg;

	//Difference fro zero-rate value, in volts
	float fGyroVoltage = fGyroDiff * 5.0 / 4095.0;

	if (fabs (fGyroDiff) > GYRO_STD_DEVS * gyro.m_config.m_fStdDev)
		if (gyro.m_config.m_bGyroFlipped)
			return -1 * fGyroVoltage / gyro.m_config.m_fVoltsPerDPS;
		else
			return fGyroVoltage / gyro.m_config.m_fVoltsPerDPS;
	return 0;
}
