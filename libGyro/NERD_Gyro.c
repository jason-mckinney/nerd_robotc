#include "NERD_Gyro.h"

#define GYRO_MULTIPLIER 0.73461357377273 // = (1.1mV/dps / 1.511) (3.3V sensor stepped up to 5v, so I scale it back to datasheet specs)
#define GYRO_STD_DEVS 3 //ignore data within 2 standard deviations of no motion average
#define GYRO_CALIBRATION_POINTS 2000 //points or time in mSec that the gyro calibrates for

float rgfRates[GYRO_CALIBRATION_POINTS];

//calculate raw gyro rate, in degrees per second
//
//returns: raw gyro rate, in degrees per second
//arguments: pointer to Gyro structure
float
gyroGetRawRate (Gyro pGyro) {
	float fGyroRate = (float) SensorValue (in1) * GYRO_MULTIPLIER; //Multiplier to scale gyro values to 0-4095
	return fGyroRate;
}

//generate calibration data for the gyro by collecting
//zero movement data for reference when reading data later
//
//arguments: pointer to gyro structure
void
gyroCalibrate(Gyro pGyro){
	float fRateAverage = 0.0;
	float fStdDev = 0.0;
	//float rgfRates[GYRO_CALIBRATION_POINTS];

	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float fRate = gyroGetRawRate(pGyro);
		fRateAverage += fRate;
		rgfRates[i] = fRate;
		delay(1);
	}
	fRateAverage /= GYRO_CALIBRATION_POINTS;
	pGyro->m_config.m_fAvg = fRateAverage;

	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		fStdDev += fabs(fRateAverage - rgfRates[i]);
	fStdDev /= GYRO_CALIBRATION_POINTS;

	pGyro->m_config.m_fStdDev = fStdDev;
}

//initialize gyro and run the calibration subroutine
//
//arguments: pointer to Gyro structure, port number of gyro
void
gyroInit(Gyro pGyro, int iPortNum){
	pGyro->m_iPortNum = iPortNum;
	gyroCalibrate(pGyro);
}

//calculate filtered gyro rate data, ignoring anything within
//GYRO_STD_DEVS standard deviations of the average gyro
//rate value at zero motion
//
//returns: gyro rate, in degrees per second
//arguments: pointer to gyro structure
float
gyroGetRate(Gyro pGyro){
	float fGyroRate = gyroGetRate(pGyro);

	if(fabs (fGyroRate - pGyro->m_config.m_fAvg) > pGyro->m_config.m_fStdDev)
		return fGyroRate - pGyro->m_config.m_fAvg;

	return 0;
}
