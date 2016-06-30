#ifndef NERD_Gyro_h
#define NERD_Gyro_h

#define GYRO_MULTIPLIER 0.00072799 // = (1.1mV/dps / 1.511) (3.3V sensor stepped up to 5v, so I scale it back to datasheet specs)
#define GYRO_STD_DEVS 2 //ignore data within 2 standard deviations of no motion average
#define GYRO_CALIBRATION_POINTS 2000 //points or time in mSec that the gyro calibrates for

struct SGyroConfig{
	float m_fStdDev; //Standard deviation of the gyro read with no motion
	float m_fAvg; //Average gyro read with no motion
};

typedef struct {
	struct SGyroConfig m_config;
	float m_fAngle;
	int m_iPortNum;
} SGyro, *Gyro;

void gyroInit(Gyro, int);
void gyroCalibrate(Gyro)
float gyroGetRawRate(Gyro);
float gyroGetRate(Gyro);
float gyroGetAngle(Gyro);

#endif