#include "../libPID/NERD_PID.c"
#include "../libToolkit/TrueSpeed.h"
#include "../libGyro/NERD_Gyro.c"

PID leftDrivePID;
PID rightDrivePID;
PID driveGyroPID;

Gyro driveGyro;

int leftDrive [10];
int rightDrive [10];

float leftDriveSetPoint;
int leftDriveSensorPort;

float rightDriveSetPoint;
int rightDriveSensorPort;

float driveGyroSetPoint;
int driveGyroSensorPort;
float driveGyroAngle;
bool driveHoldRunning = false;

void driveGoTo (float setPoint, float range);
void driveGyroHold (float setPoint);
void driveGyroInit (float kP, float kI, float kD, float inner, float outer, int sensorPort);
void driveGyroTurn (float setPoint, float range);
void driveHold (float setPoint);
void driveHold (float setPointLeft, float setPointRight);
void driveHoldStop ();
void driveInit (float kP, float kI, float kD, float inner, float outer, int sensorPortLeft, int sensorPortRight);
void driveLeftDrive (int speed);
void driveRightDrive (int speed);
void driveStop ();
void driveTurnLeft (float setPoint, float range);
void driveTurnRight (float setPoint, float range);

task taskDriveHold ();
task taskDriveGyroHold ();