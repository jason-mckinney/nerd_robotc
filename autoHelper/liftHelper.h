#include "../libPID/NERD_PID.c"
#include "../libToolkit/TrueSpeed.h"

PID liftPID;

int lift [10];

float liftSetPoint;
int liftSensorPort;

bool liftHoldRunning = false;

void driveLift (int speed);
void liftGoTo (float setPoint, float range);
void liftHold (float setPoint);
void liftHoldStop ();
void liftInit (float kP, float kI, float kD, float inner, float outer, int sensorPort);
void liftStop ();

task taskLiftHold ();
