# Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public void `[`pidInit`](#motion_planner_8c_1ad6e45f67f53a6a3eec3fbd3c0e03d9e7)`(PID pid,float Kp,float Ki,float Kd,float innerIntegralBand,float outerIntegralBand)`            | initialize pid structure, set parameters
`public void `[`pidInit`](#motion_planner_8c_1a3716b050fba96285a13502b79d13795c)`(PID pid,PID toCopy)`            | initialize pid structure, set parameters based on another PID structure
`public float `[`pidCalculate`](#motion_planner_8c_1a55c32f7660c964a71e288c9efe3a962a)`(PID pid,int setPoint,int processVariable)`            | calculate pid output
`public float `[`pidCalculateWithVelocitySet`](#motion_planner_8c_1aea4332a3ded7a93261d8c53e0b83d277)`(PID pid,int setPoint,int processVariable,int velocitySet)`            | calculate PID output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error
`public float `[`pidCalculateVelocity`](#motion_planner_8c_1acb752ade99450cfce2f7e7ad3f80e9c6)`(PID pid,int setPoint,int processVariable)`            | calculate PID output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.
`public int * `[`getRawSensor`](#motion_planner_8c_1ac842b4b04041e6a1ca5dfbf4e8652bea)`(int port)`            | return a pointer to a ROBOTC sensor
`public void `[`createMotionProfile`](#motion_planner_8c_1a1b9b6097dd7e4e65093c30e080a19ef1)`(int motorPort,int * sensor,int vMax,float Ka,int t1,int t2,int cycleTime,int positionCycles)`            | create a motion profile for a motor/sensor pair. PID controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/vMax
`public void `[`createMotionProfile`](#motion_planner_8c_1a31cfbe0e8576c0a0ea7d6109d092b604)`(int motorPort,int * sensor,int vMax)`            | create a motion profile for a motor/sensor pair using default timing settings and a Ka of 0. This will provide simple feedforward position/velocity control
`public void `[`setPositionController`](#motion_planner_8c_1a30ce5d795dc46d722f83c97fde8d3198)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)`            | set the position PID controller for the specified motor's motion profile
`public void `[`setVelocityController`](#motion_planner_8c_1a84c067f5b749396c97f339e18b3916dd)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)`            | set the velocity PID controller for the specified motor's motion profile
`public void `[`setMotionSlave`](#motion_planner_8c_1a98a294f0bd0e05a06e4b3ce91fd3b768)`(int motorPort,int masterPort)`            | set a motor to copy another motor's motion profile and mirror its output value
`public void `[`setPosition`](#motion_planner_8c_1a14723e38e593eefa92500a993303da0c)`(int motorPort,int position)`            | issue a move command to the specified position. This will currently do nothing if the move is considered to be a "short move", ie. the motor is unable to fully ramp up to max velocity during the move. Note that this is an absolute position command, so two consecutive moves to 4000 are not equivalent to a single move to 8000.
`public void `[`setPWMOutput`](#motion_planner_8c_1a22e3be616dbc3c49d64c1be6c3d912fa)`(int motorPort,int output)`            | set a motor's output to a value from -127 to 127
`public void `[`setVelocity`](#motion_planner_8c_1a6bfe725c4f5986245fba04d84ba9a725)`(int motorPort,int velocity)`            | set a motor's velocity to the specified value
`private void `[`updateMotors`](#motion_planner_8c_1a807a87c5df438fde21c1e8213906695b)`()`            | 
`private void `[`measureVelocity`](#motion_planner_8c_1ab2c54bc2c4636cb590b1b7675b6ddc03)`(motionProfiler * profile)`            | 
`private void `[`velocityUpdate`](#motion_planner_8c_1a00408d576818e5185973f32f718fe368)`(motionProfiler * profile)`            | 
`private void `[`positionUpdate`](#motion_planner_8c_1a18e4b5b74e187634ebc0800e119d6fcf)`(motionProfiler * profile)`            | 
`private task `[`rawSensorMonitor`](#motion_planner_8c_1a7a8acda92bd2fda1a52531d93313884c)`()`            | 
`private task `[`motionPlanner`](#motion_planner_8c_1abbcf1c2f53386d0935c1727eff22d2ae)`()`            | 

## Members

#### `public void `[`pidInit`](#motion_planner_8c_1ad6e45f67f53a6a3eec3fbd3c0e03d9e7)`(PID pid,float Kp,float Ki,float Kd,float innerIntegralBand,float outerIntegralBand)` 

initialize pid structure, set parameters

#### Parameters
* `pid` instance of PID structure 

* `Kp` PID Kp constant 

* `Ki` PID Ki constant 

* `Kd` PID Kd constant 

* `innerIntegralBand` inner bound of PID I summing cutoff 

* `outerIntegralBand` outer bound of PID I summing cutoff

#### `public void `[`pidInit`](#motion_planner_8c_1a3716b050fba96285a13502b79d13795c)`(PID pid,PID toCopy)` 

initialize pid structure, set parameters based on another PID structure

#### Parameters
* `pid` instance of PID structure 

* `toCopy` PID instance to copy settings from

#### `public float `[`pidCalculate`](#motion_planner_8c_1a55c32f7660c964a71e288c9efe3a962a)`(PID pid,int setPoint,int processVariable)` 

calculate pid output

#### Parameters
* `pid` instance of PID structure 

* `setPoint` set point of PID controller 

* `processVariable` sensor/feedback value

#### Returns
output value of the control loop

#### `public float `[`pidCalculateWithVelocitySet`](#motion_planner_8c_1aea4332a3ded7a93261d8c53e0b83d277)`(PID pid,int setPoint,int processVariable,int velocitySet)` 

calculate PID output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error

#### Parameters
* `pid` the PID controller to use for the calculation 

* `setPoint` the set point of the system 

* `processVariable` the value of the feedback sensor in the system 

* `velocitySet` the velocity set point of the system

#### Returns
the output value of the control loop

#### `public float `[`pidCalculateVelocity`](#motion_planner_8c_1acb752ade99450cfce2f7e7ad3f80e9c6)`(PID pid,int setPoint,int processVariable)` 

calculate PID output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.

#### Parameters
* `pid` the PID controller to use for the calculation 

* `setPoint` the set point of the system 

* `processVariable` the value of the feedback sensor in the system

#### Returns
the output value of the control loop

#### `public int * `[`getRawSensor`](#motion_planner_8c_1ac842b4b04041e6a1ca5dfbf4e8652bea)`(int port)` 

return a pointer to a ROBOTC sensor

#### Parameters
* `port` the sensor value to get a pointer to

#### Returns
the pointer to the sensor value

#### `public void `[`createMotionProfile`](#motion_planner_8c_1a1b9b6097dd7e4e65093c30e080a19ef1)`(int motorPort,int * sensor,int vMax,float Ka,int t1,int t2,int cycleTime,int positionCycles)` 

create a motion profile for a motor/sensor pair. PID controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/vMax

#### Parameters
* `motorPort` the motor port to create a profile for 

* `sensor` a pointer to the sensor value to monitor. This can be a pointer to any integer, or a "raw" sensor value using [getRawSensor()](#motion_planner_8c_1ac842b4b04041e6a1ca5dfbf4e8652bea). 

* `vMax` the maximum velocity to use when calculating moves 

* `Ka` acceleration constant used when ramping up/down velocity during moves 

* `t1` time to spend at peak acceleration at the beginning/end of a move. This and t2 will determine the shape of the motion curve 

* `t2` time to spend at peak jerk (time derivative of acceleration) at the beginning/end of acceleration. Time to get from 0 to max velocity (or max to 0) = t1 + 2*t2 

* `cycleTime` polling rate/sample period of system. Polling frequency = 1000/cycleTime. Note that ports 2-9 on cortex only can update at a frequency of 18.5Hz, so values less than ~20 here will offer diminishing returns. 

* `positionCycles` cycles to skip for position updates during moves. This will generally be 3-5

#### `public void `[`createMotionProfile`](#motion_planner_8c_1a31cfbe0e8576c0a0ea7d6109d092b604)`(int motorPort,int * sensor,int vMax)` 

create a motion profile for a motor/sensor pair using default timing settings and a Ka of 0. This will provide simple feedforward position/velocity control

#### Parameters
* `motorPort` the motor port to create a profile for 

* `sensor` a pointer to the sensor value to monitor. This can be a pointer to any integer, or a "raw" sensor value using [getRawSensor()](#motion_planner_8c_1ac842b4b04041e6a1ca5dfbf4e8652bea). 

* `vMax` the maximum velocity to use when calculating moves

#### `public void `[`setPositionController`](#motion_planner_8c_1a30ce5d795dc46d722f83c97fde8d3198)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)` 

set the position PID controller for the specified motor's motion profile

#### Parameters
* `motorPort` the motor to update 

* `Kp` proportional gain 

* `Ki` integral gain 

* `Kd` derivative gain 

* `innerBand` the inner integral deadBand value 

* `outerBand` the outer integral deadBand value

#### `public void `[`setVelocityController`](#motion_planner_8c_1a84c067f5b749396c97f339e18b3916dd)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)` 

set the velocity PID controller for the specified motor's motion profile

#### Parameters
* `motorPort` the motor to update 

* `Kp` proportional gain 

* `Ki` integral gain 

* `Kd` derivative gain 

* `innerBand` the inner integral deadBand value 

* `outerBand` the outer integral deadBand value

#### `public void `[`setMotionSlave`](#motion_planner_8c_1a98a294f0bd0e05a06e4b3ce91fd3b768)`(int motorPort,int masterPort)` 

set a motor to copy another motor's motion profile and mirror its output value

#### Parameters
* `motorPort` the motor to have mirror another motor 

* `masterPort` the motor to mirror

#### `public void `[`setPosition`](#motion_planner_8c_1a14723e38e593eefa92500a993303da0c)`(int motorPort,int position)` 

issue a move command to the specified position. This will currently do nothing if the move is considered to be a "short move", ie. the motor is unable to fully ramp up to max velocity during the move. Note that this is an absolute position command, so two consecutive moves to 4000 are not equivalent to a single move to 8000.

#### Parameters
* `motorPort` the motor to issue the move command to 

* `position` the position to move to

#### `public void `[`setPWMOutput`](#motion_planner_8c_1a22e3be616dbc3c49d64c1be6c3d912fa)`(int motorPort,int output)` 

set a motor's output to a value from -127 to 127

#### Parameters
* `motorPort` the motor to set the output value of 

* `output` output value to set

#### `public void `[`setVelocity`](#motion_planner_8c_1a6bfe725c4f5986245fba04d84ba9a725)`(int motorPort,int velocity)` 

set a motor's velocity to the specified value

#### Parameters
* `motorPort` the motor to set 

* `velocity` desired velocity

#### `private void `[`updateMotors`](#motion_planner_8c_1a807a87c5df438fde21c1e8213906695b)`()` 

#### `private void `[`measureVelocity`](#motion_planner_8c_1ab2c54bc2c4636cb590b1b7675b6ddc03)`(motionProfiler * profile)` 

#### `private void `[`velocityUpdate`](#motion_planner_8c_1a00408d576818e5185973f32f718fe368)`(motionProfiler * profile)` 

#### `private void `[`positionUpdate`](#motion_planner_8c_1a18e4b5b74e187634ebc0800e119d6fcf)`(motionProfiler * profile)` 

#### `private task `[`rawSensorMonitor`](#motion_planner_8c_1a7a8acda92bd2fda1a52531d93313884c)`()` 

#### `private task `[`motionPlanner`](#motion_planner_8c_1abbcf1c2f53386d0935c1727eff22d2ae)`()` 

Generated by [Moxygen](https://sourcey.com/moxygen)