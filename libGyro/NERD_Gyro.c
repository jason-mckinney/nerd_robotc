#include "NERD_Gyro.h"

// = (1.1mV/dps / 1.511) (3.3V sensor stepped up to 5v, so I scale it back to datasheet specs)
#define GYRO_MULTIPLIER 0.73461357377273

//ignore data within 2 standard deviations of no motion average
#define GYRO_STD_DEVS 3

//points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 2000

float rgfRates[GYRO_CALIBRATION_POINTS];

/**
 * calculate raw gyro rate, in degrees per second
 *
 * @param gyro instance of gyro structure
 *
 * @return raw gyro rate, in degrees per second
 */
float
gyroGetRawRate (Gyro gyro) {
	float fGyroRate = (float) SensorValue (in1) * GYRO_MULTIPLIER; //Multiplier to scale gyro values to 0-4095
	return fGyroRate;
}

/**
 * generate calibration data for the gyro by collecting
 * zero movement data for reference when reading data later
 *
 * @param gyro instance of gyro structure
 */
void
gyroCalibrate (Gyro gyro){
	float fRateAverage = 0.0;
	float fStdDev = 0.0;
	//float rgfRates[GYRO_CALIBRATION_POINTS];

	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float fRate = gyroGetRawRate (gyro);
		fRateAverage += fRate;
		rgfRates [i] = fRate;
		delay (1);
	}
	fRateAverage /= GYRO_CALIBRATION_POINTS;
	gyro.m_config.m_fAvg = fRateAverage;

	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		fStdDev += fabs (fRateAverage - rgfRates [i]);
	fStdDev /= GYRO_CALIBRATION_POINTS;

	gyro.m_config.m_fStdDev = fStdDev;
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
	float fGyroRate = gyroGetRate (gyro);

	if (fabs (fGyroRate - gyro.m_config.m_fAvg) > gyro.m_config.m_fStdDev)
		return fGyroRate - gyro.m_config.m_fAvg;

	return 0;
}
