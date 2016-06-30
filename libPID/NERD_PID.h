#ifndef NERD_PID_h
#define NERD_PID_h

typedef struct{
	float m_fKP;
	float m_fKI;
	float m_fKD;
	float m_fEpsilon;
	float m_fSigma;
	float m_fLastValue;
	unsigned long m_uliLastTime;
	float m_fLastSetPoint;
} SPID, *PID;

void pidInit(PID, float, float, float, float); //SPID*, kP, kI, kD, Epsilon
float pidCalculate(PID, float, float); //SPID*, setPoint, processVariable

#endif