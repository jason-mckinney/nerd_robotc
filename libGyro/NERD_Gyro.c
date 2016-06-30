#include "NERD_Gyro.h"

void gyroInit(Gyro pGyro, int iPortNum){
	pGyro->m_iPortNum = iPortNum;
	gyroCalibrate(pGyro);
}

float gyroGetRawRate(Gyro pGyro){
	float fGyroRate = (float)SensorValue(pGyro->m_iPortNum) 
						* (5.0/4095.0) //0-5V sensor mapped to 0-4095 sensorValue
						* GYRO_MULTIPLIER; //Multiplier to scale gyro values to 0-4096
	return fGyroRate;
}

float gyroGetRate(Gyro pGyro){
	float fGyroRate = gyroGetRawRate(pGyro);

	if(abs(fGyroRate) 
		> pGyro->m_config.m_fAvg 
			+ (GYRO_STD_DEVS * pGyro->m_config.m_fStdDev))
		return fGyroRate;

	return 0;
}

void gyroCalibrate(Gyro pGyro){
	float fRateAverage = 0.0;
	float fStdDev = 0.0;
	float rgfRates[GYRO_CALIBRATION_POINTS];

	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float fRate = gyroGetRawRate(pGyro);
		fRateAverage += fRate;
		rgfRates[i] = fRate;
		delay(1);
	}
	fRateAverage /= GYRO_CALIBRATION_POINTS;
	pGyro->m_config.m_fAvg = fRateAverage;

	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		fStdDev += abs(fRateAverage - rgfRates[i]);
	fStdDev /= GYRO_CALIBRATION_POINTS;

	pGyro->m_config.m_fStdDev = fStdDev;
}