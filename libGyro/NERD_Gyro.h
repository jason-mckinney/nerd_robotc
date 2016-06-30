#ifndef NERD_Gyro_h
#define NERD_Gyro_h

struct SGyroConfig{
	float m_fStdDev; //Standard deviation of the gyro read with no motion
	float m_fAvg; //Average gyro read with no motion
};

typedef struct {
	struct SGyroConfig m_config;
	float m_fAngle;
	int m_iPortNum;
} SGyro, *Gyro;

#endif
