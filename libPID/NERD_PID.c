#include "NERD_PID.h"

//initialize PID structure, set parameters
//
//arguments: pointer to PID structure, kP constant, kI constant, kD constant,
//             EpsilonInner, EpsilonOuter constants (inner and outer ranges of set point in which to stop summing for I component)
void
pidInit(PID pPID, float fKP, float fKI, float fKD, float fEpsilonInner, float fEpsilonOuter){
	pPID->m_fKP = fKP;
	pPID->m_fKI = fKI;
	pPID->m_fKD = fKD;
	pPID->m_fEpsilonInner = fEpsilonInner;
	pPID->m_fEpsilonOuter = fEpsilonOuter;
	pPID->m_fSigma = 0;
	pPID->m_fLastValue = 0;
	pPID->m_uliLastTime = nPgmTime;
}

//calculate PID output
//
//returns: output value constrained between -127 and 127
//arguments: pointer to PID structure, set point, sensor value or process variable
float
pidCalculate(PID pPID, float fSetPoint, float fProcessVariable){
	float fDeltaTime = (float)(nPgmTime - pPID->m_uliLastTime) / 1000.0;
	pPID->m_uliLastTime = nPgmTime;

	float fDeltaPV = 0;
	if(fDeltaTime > 0)
		fDeltaPV = (fProcessVariable - pPID->m_fLastValue) / fDeltaTime;
	pPID->m_fLastValue = fProcessVariable;

	float fError = fSetPoint - fProcessVariable;

	if(fabs(fError) > pPID->m_fEpsilonInner && fabs(fError) < pPID->m_fEpsilonOuter)
		pPID->m_fSigma += fError * fDeltaTime;

	float fOutput = fError * pPID->m_fKP
					+ pPID->m_fSigma * pPID->m_fKI
					- fDeltaPV * pPID->m_fKD;

	fOutput = abs(fOutput) > 127 ? 127 * fOutput/abs(fOutput) : fOutput;
	return fOutput;
}
