#include "NERD_Gyro.h"

//ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 5

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
	//float rgfRates[GYRO_CALIBRATION_POINTS];

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
	gyro.m_config.m_fVoltsPerDPS = (0.0011 * (fRawAverage*5/4095)/1.71625741);
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
	float fGyroRead = SensorValue (gyro.m_iPortNum);
	
	if (fabs (fGyroRead - gyro.m_config.m_fAvg) > GYRO_STD_DEVS * gyro.m_config.m_fStdDev) 
		return (fGyroRead - gyro.m_config.m_fAvg) * 0.001221 / gyro.m_config.m_fVoltsPerDPS;

	return 0;
}
