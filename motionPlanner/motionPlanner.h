#ifndef NERD_MOTIONPLANNER
#define NERD_MOTIONPLANNER

#include "../libPID/NERD_PID.c"

typedef struct {
	PID positionController;
	PID velocityController;

} motionController;

motionController* motorControllers [10];

//sensor variable 


#endif