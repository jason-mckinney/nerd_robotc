#include "../libPID/NERD_PID.c"
#include "../libToolkit/TrueSpeed.h"

PID liftPID;

int lift [10];

float liftSetPoint;
int liftSensorPort;

bool liftHoldRunning = false;

void driveLift (int);
void liftGoTo (float, float);
void liftHold (float);
void liftHoldStop ();
void liftInit (float, float, float, float, float, int);
void liftStop ();

task taskLiftHold ();
