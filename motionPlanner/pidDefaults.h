#ifndef PID_DEFAULTS
#define PID_DEFAULTS

#include "NERD_PID.h"

const struct {
	PID posNoGravity;
	PID posWithGravity;
	PID velNoGravity;
	PID velWithGravity;
} pidDefaults = {{0.3, 0.03, 0.003, 0, 0, 0, 0, 0, 0}, 
		{0.5, 0.05, 0.005, 0, 0, 0, 0, 0, 0}, 
		{0.3, 0.03, 0.003, 0, 0,, 0, 0, 0}, 
		{0.5, 0.05, 0.005, 0, 0, 0, 0, 0, 0}};

#endif