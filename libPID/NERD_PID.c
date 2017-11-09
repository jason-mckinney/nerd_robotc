#include "NERD_pid.h"
#ifndef PID_C
#define PID_C
/**
 * initialize pid structure, set parameters
 *
 * @param pid instance of PID structure
 * @param fKP PID KP constant
 * @param fKI PID KI constant
 * @param fKD PID KD constant
 * @param fEpsilonInner inner bound of PID I summing cutoff
 * @param fEpsilonOuter outer bound of PID I summing cutoff
 */
void
pidInit (PID pid, float fKP, float fKI, float fKD, float fEpsilonInner, float fEpsilonOuter) {
	pid.m_fKP = fKP;
	pid.m_fKI = fKI;
	pid.m_fKD = fKD;
	pid.m_fEpsilonInner = fEpsilonInner;
	pid.m_fEpsilonOuter = fEpsilonOuter;
	pid.m_fSigma = 0;
	pid.m_fLastValue = 0;
	pid.m_uliLastTime = nPgmTime;
}

/**
 * initialize pid structure, set parameters based on another PID structure
 *
 * @param pid  instance of PID structure
 * @param toCopy  PID instance to copy settings from
 */
void pidInit (PID pid, PID toCopy) {
	pid.m_fKP = toCopy.m_fKP;
	pid.m_fKI = toCopy.fKI;
	pid.m_fKD = toCopy.fKD;
	pid.m_fEpsilonInner = toCopy.fEpsilonInner;
	pid.m_fEpsilonOuter = toCopy.fEpsilonOuter;
	pid.m_fSigma = 0;
	pid.m_fLastValue = 0;
	pid.m_uliLastTime = nPgmTime;
}

/**
 * calculate pid output
 *
 * @param pid instance of PID structure
 * @param fSetPoint set point of PID controller
 * @param fProcessVariable sensor/feedback value
 *
 * @return output value constrained from -127 to 127
 */
float
pidCalculate (PID pid, float fSetPoint, float fProcessVariable) {
	float fDeltaTime = (float)(nPgmTime - pid.m_uliLastTime) / 1000.0;
	pid.m_uliLastTime = nPgmTime;

	float fDeltaPV = 0;
	if(fDeltaTime > 0)
		fDeltaPV = (fProcessVariable - pid.m_fLastValue) / fDeltaTime;
	pid.m_fLastValue = fProcessVariable;

	float fError = fSetPoint - fProcessVariable;

	if(fabs(fError) > pid.m_fEpsilonInner && fabs(fError) < pid.m_fEpsilonOuter)
		pid.m_fSigma += fError * fDeltaTime;

	if (fabs (fError) > pid.m_fEpsilonOuter)
		pid.m_fSigma = 0;

	float fOutput = fError * pid.m_fKP
					+ pid.m_fSigma * pid.m_fKI
					- fDeltaPV * pid.m_fKD;

	fOutput = fabs(fOutput) > 127 ? 127 * fOutput/fabs(fOutput) : fOutput;
	return fOutput;
}
#endif
