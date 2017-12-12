# Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`define `[`TRUESPEED_H`](#motion_planner_8c_1a0539e40392ac567049e64b8702a1a679)            | 
`define `[`NERD_PID_h`](#motion_planner_8c_1a921a68decd8b7a0286f657e4f2839ff0)            | 
`define `[`PID_C`](#motion_planner_8c_1a56d8cc7537790863f0284e50c4c78ce6)            | 
`define `[`NERD_MOTIONPLANNER`](#motion_planner_8c_1a9a8deb415b6818184cd925709e01f3ea)            | 
`public void `[`pidInit`](#motion_planner_8c_1ad6e45f67f53a6a3eec3fbd3c0e03d9e7)`(`[`PID`](#struct_p_i_d)` pid,float Kp,float Ki,float Kd,float innerIntegralBand,float outerIntegralBand)`            | initialize pid structure, set parameters
`public void `[`pidInit`](#motion_planner_8c_1a3716b050fba96285a13502b79d13795c)`(`[`PID`](#struct_p_i_d)` pid,`[`PID`](#struct_p_i_d)` toCopy)`            | initialize pid structure, set parameters based on another [PID](#struct_p_i_d) structure
`public float `[`pidCalculate`](#motion_planner_8c_1a55c32f7660c964a71e288c9efe3a962a)`(`[`PID`](#struct_p_i_d)` pid,int setPoint,int processVariable)`            | calculate pid output
`public float `[`pidCalculateWithVelocitySet`](#motion_planner_8c_1aea4332a3ded7a93261d8c53e0b83d277)`(`[`PID`](#struct_p_i_d)` pid,int setPoint,int processVariable,int velocitySet)`            | calculate [PID](#struct_p_i_d) output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error
`public float `[`pidCalculateVelocity`](#motion_planner_8c_1acb752ade99450cfce2f7e7ad3f80e9c6)`(`[`PID`](#struct_p_i_d)` pid,int setPoint,int processVariable)`            | calculate [PID](#struct_p_i_d) output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.
`public int * `[`getRawSensor`](#motion_planner_8c_1ac842b4b04041e6a1ca5dfbf4e8652bea)`(int port)`            | return a pointer to a ROBOTC sensor
`public void `[`createMotionProfile`](#motion_planner_8c_1a1b9b6097dd7e4e65093c30e080a19ef1)`(int motorPort,int * sensor,int vMax,float Ka,int t1,int t2,int cycleTime,int positionCycles)`            | create a motion profile for a motor/sensor pair. [PID](#struct_p_i_d) controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/vMax
`public void `[`createMotionProfile`](#motion_planner_8c_1a31cfbe0e8576c0a0ea7d6109d092b604)`(int motorPort,int * sensor,int vMax)`            | create a motion profile for a motor/sensor pair using default timing settings and a Ka of 0. This will provide simple feedforward position/velocity control
`public void `[`setPositionController`](#motion_planner_8c_1a30ce5d795dc46d722f83c97fde8d3198)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)`            | set the position [PID](#struct_p_i_d) controller for the specified motor's motion profile
`public void `[`setVelocityController`](#motion_planner_8c_1a84c067f5b749396c97f339e18b3916dd)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)`            | set the velocity [PID](#struct_p_i_d) controller for the specified motor's motion profile
`public void `[`setMotionSlave`](#motion_planner_8c_1a98a294f0bd0e05a06e4b3ce91fd3b768)`(int motorPort,int masterPort)`            | set a motor to copy another motor's motion profile and mirror its output value
`public void `[`setPosition`](#motion_planner_8c_1a14723e38e593eefa92500a993303da0c)`(int motorPort,int position)`            | issue a move command to the specified position. This will currently do nothing if the move is considered to be a "short move", ie. the motor is unable to fully ramp up to max velocity during the move. Note that this is an absolute position command, so two consecutive moves to 4000 are not equivalent to a single move to 8000.
`public void `[`setPWMOutput`](#motion_planner_8c_1a22e3be616dbc3c49d64c1be6c3d912fa)`(int motorPort,int output)`            | set a motor's output to a value from -127 to 127
`public void `[`setVelocity`](#motion_planner_8c_1a6bfe725c4f5986245fba04d84ba9a725)`(int motorPort,int velocity)`            | set a motor's velocity to the specified value
`private void `[`updateMotors`](#motion_planner_8c_1a807a87c5df438fde21c1e8213906695b)`()`            | 
`private void `[`measureVelocity`](#motion_planner_8c_1ab2c54bc2c4636cb590b1b7675b6ddc03)`(`[`motionProfiler`](#structmotion_profiler)` * profile)`            | 
`private void `[`velocityUpdate`](#motion_planner_8c_1a00408d576818e5185973f32f718fe368)`(`[`motionProfiler`](#structmotion_profiler)` * profile)`            | 
`private void `[`positionUpdate`](#motion_planner_8c_1a18e4b5b74e187634ebc0800e119d6fcf)`(`[`motionProfiler`](#structmotion_profiler)` * profile)`            | 
`private task `[`rawSensorMonitor`](#motion_planner_8c_1a7a8acda92bd2fda1a52531d93313884c)`()`            | 
`private task `[`motionPlanner`](#motion_planner_8c_1abbcf1c2f53386d0935c1727eff22d2ae)`()`            | 
`struct `[`motionProfiler`](#structmotion_profiler) | 
`struct `[`PID`](#struct_p_i_d) | [PID](#struct_p_i_d) controller data structure

## Members

#### `define `[`TRUESPEED_H`](#motion_planner_8c_1a0539e40392ac567049e64b8702a1a679) 

#### `define `[`NERD_PID_h`](#motion_planner_8c_1a921a68decd8b7a0286f657e4f2839ff0) 

#### `define `[`PID_C`](#motion_planner_8c_1a56d8cc7537790863f0284e50c4c78ce6) 

#### `define `[`NERD_MOTIONPLANNER`](#motion_planner_8c_1a9a8deb415b6818184cd925709e01f3ea) 

#### `public void `[`pidInit`](#motion_planner_8c_1ad6e45f67f53a6a3eec3fbd3c0e03d9e7)`(`[`PID`](#struct_p_i_d)` pid,float Kp,float Ki,float Kd,float innerIntegralBand,float outerIntegralBand)` 

initialize pid structure, set parameters

#### Parameters
* `pid` instance of [PID](#struct_p_i_d) structure 

* `Kp` [PID](#struct_p_i_d) Kp constant 

* `Ki` [PID](#struct_p_i_d) Ki constant 

* `Kd` [PID](#struct_p_i_d) Kd constant 

* `innerIntegralBand` inner bound of [PID](#struct_p_i_d) I summing cutoff 

* `outerIntegralBand` outer bound of [PID](#struct_p_i_d) I summing cutoff

#### `public void `[`pidInit`](#motion_planner_8c_1a3716b050fba96285a13502b79d13795c)`(`[`PID`](#struct_p_i_d)` pid,`[`PID`](#struct_p_i_d)` toCopy)` 

initialize pid structure, set parameters based on another [PID](#struct_p_i_d) structure

#### Parameters
* `pid` instance of [PID](#struct_p_i_d) structure 

* `toCopy` [PID](#struct_p_i_d) instance to copy settings from

#### `public float `[`pidCalculate`](#motion_planner_8c_1a55c32f7660c964a71e288c9efe3a962a)`(`[`PID`](#struct_p_i_d)` pid,int setPoint,int processVariable)` 

calculate pid output

#### Parameters
* `pid` instance of [PID](#struct_p_i_d) structure 

* `setPoint` set point of [PID](#struct_p_i_d) controller 

* `processVariable` sensor/feedback value

#### Returns
output value of the control loop

#### `public float `[`pidCalculateWithVelocitySet`](#motion_planner_8c_1aea4332a3ded7a93261d8c53e0b83d277)`(`[`PID`](#struct_p_i_d)` pid,int setPoint,int processVariable,int velocitySet)` 

calculate [PID](#struct_p_i_d) output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error

#### Parameters
* `pid` the [PID](#struct_p_i_d) controller to use for the calculation 

* `setPoint` the set point of the system 

* `processVariable` the value of the feedback sensor in the system 

* `velocitySet` the velocity set point of the system

#### Returns
the output value of the control loop

#### `public float `[`pidCalculateVelocity`](#motion_planner_8c_1acb752ade99450cfce2f7e7ad3f80e9c6)`(`[`PID`](#struct_p_i_d)` pid,int setPoint,int processVariable)` 

calculate [PID](#struct_p_i_d) output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.

#### Parameters
* `pid` the [PID](#struct_p_i_d) controller to use for the calculation 

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

create a motion profile for a motor/sensor pair. [PID](#struct_p_i_d) controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/vMax

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

set the position [PID](#struct_p_i_d) controller for the specified motor's motion profile

#### Parameters
* `motorPort` the motor to update 

* `Kp` proportional gain 

* `Ki` integral gain 

* `Kd` derivative gain 

* `innerBand` the inner integral deadBand value 

* `outerBand` the outer integral deadBand value

#### `public void `[`setVelocityController`](#motion_planner_8c_1a84c067f5b749396c97f339e18b3916dd)`(int motorPort,float Kp,float Ki,float Kd,float innerBand,float outerBand)` 

set the velocity [PID](#struct_p_i_d) controller for the specified motor's motion profile

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

#### `private void `[`measureVelocity`](#motion_planner_8c_1ab2c54bc2c4636cb590b1b7675b6ddc03)`(`[`motionProfiler`](#structmotion_profiler)` * profile)` 

#### `private void `[`velocityUpdate`](#motion_planner_8c_1a00408d576818e5185973f32f718fe368)`(`[`motionProfiler`](#structmotion_profiler)` * profile)` 

#### `private void `[`positionUpdate`](#motion_planner_8c_1a18e4b5b74e187634ebc0800e119d6fcf)`(`[`motionProfiler`](#structmotion_profiler)` * profile)` 

#### `private task `[`rawSensorMonitor`](#motion_planner_8c_1a7a8acda92bd2fda1a52531d93313884c)`()` 

#### `private task `[`motionPlanner`](#motion_planner_8c_1abbcf1c2f53386d0935c1727eff22d2ae)`()` 

# struct `motionProfiler` 

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public `[`PID`](#struct_p_i_d)` `[`positionController`](#structmotion_profiler_1a6af58c8bbf5ff2a9657b3003a6521db0) | 
`public `[`PID`](#struct_p_i_d)` `[`velocityController`](#structmotion_profiler_1a731ffc5dde37d1e3bb373311c7f84d61) | 
`public float `[`Kv`](#structmotion_profiler_1a9dd79979f873707ff49614e3d2b05371) | 
`public float `[`Ka`](#structmotion_profiler_1af7e0be223ca11bc5e95332a223a5a410) | 
`public int * `[`sensor`](#structmotion_profiler_1ac45c58330bcdae28978c77a6cfde8f72) | 
`public int `[`velocityFilter`](#structmotion_profiler_1aa5d8685398a6a8a736ee930be4de1d64) | 
`public int `[`velocityRead`](#structmotion_profiler_1abab91ecaedaa0da8ff8efb499269ff0a) | 
`public char `[`profileSetting`](#structmotion_profiler_1af1c26d55aae953850de4497090f96033) | 
`public int `[`cycleCounter`](#structmotion_profiler_1a7568fe510550cda7e14714cb1940ca8f) | 
`public short `[`motorOutput`](#structmotion_profiler_1a596a165e0e260065d862d418d84c7f6b) | 
`public float `[`positionOut`](#structmotion_profiler_1ae3c797ea09227486617f837045db17ce) | 
`public int `[`lastSensorValue`](#structmotion_profiler_1a0f19b669e85c8c71ac80e985351be173) | 
`public unsigned int `[`lastTime`](#structmotion_profiler_1ab17786ea0ccd57d2b527d06143e22c15) | 
`public int `[`finalPosition`](#structmotion_profiler_1a49d3333b9018fe1b2454e56c30a72311) | 
`public float `[`positionSet`](#structmotion_profiler_1ae45782f37eaa20c9ccb117874abc7ff2) | 
`public float `[`velocitySet`](#structmotion_profiler_1abecf199e03d7c61b7694435888364aaf) | 
`public float `[`accelSet`](#structmotion_profiler_1ae3f53b386932f91b87cfd744c4532603) | 
`public float `[`jerk`](#structmotion_profiler_1ad12385a16c1e70621af252dc46ddd611) | 
`public char `[`planComplete`](#structmotion_profiler_1a8b7b1cd195d2ce0981b7d4e85675896a) | 
`public int `[`vMax`](#structmotion_profiler_1a939c9510d8065a680fb9390dc98c89d9) | 
`public int `[`t1`](#structmotion_profiler_1a1f006058637a82726360f71ab3396f0e) | 
`public int `[`t2`](#structmotion_profiler_1aa931d62492a0ef5a5a1ff1a2ed78d9c6) | 
`public int `[`t4`](#structmotion_profiler_1a3d25457e1add6ee8836d29f078e7cc1b) | 
`public int `[`tMax`](#structmotion_profiler_1a8b70c4e6e7bfb03961f944751725e6f2) | 
`public float `[`cycleTime`](#structmotion_profiler_1a937382b7ba919b74e49bb7686a3b86c6) | 
`public int `[`positionCycles`](#structmotion_profiler_1a519d6d21d377d231f1c947d83de92841) | 

## Members

#### `public `[`PID`](#struct_p_i_d)` `[`positionController`](#structmotion_profiler_1a6af58c8bbf5ff2a9657b3003a6521db0) 

#### `public `[`PID`](#struct_p_i_d)` `[`velocityController`](#structmotion_profiler_1a731ffc5dde37d1e3bb373311c7f84d61) 

#### `public float `[`Kv`](#structmotion_profiler_1a9dd79979f873707ff49614e3d2b05371) 

#### `public float `[`Ka`](#structmotion_profiler_1af7e0be223ca11bc5e95332a223a5a410) 

#### `public int * `[`sensor`](#structmotion_profiler_1ac45c58330bcdae28978c77a6cfde8f72) 

#### `public int `[`velocityFilter`](#structmotion_profiler_1aa5d8685398a6a8a736ee930be4de1d64) 

#### `public int `[`velocityRead`](#structmotion_profiler_1abab91ecaedaa0da8ff8efb499269ff0a) 

#### `public char `[`profileSetting`](#structmotion_profiler_1af1c26d55aae953850de4497090f96033) 

#### `public int `[`cycleCounter`](#structmotion_profiler_1a7568fe510550cda7e14714cb1940ca8f) 

#### `public short `[`motorOutput`](#structmotion_profiler_1a596a165e0e260065d862d418d84c7f6b) 

#### `public float `[`positionOut`](#structmotion_profiler_1ae3c797ea09227486617f837045db17ce) 

#### `public int `[`lastSensorValue`](#structmotion_profiler_1a0f19b669e85c8c71ac80e985351be173) 

#### `public unsigned int `[`lastTime`](#structmotion_profiler_1ab17786ea0ccd57d2b527d06143e22c15) 

#### `public int `[`finalPosition`](#structmotion_profiler_1a49d3333b9018fe1b2454e56c30a72311) 

#### `public float `[`positionSet`](#structmotion_profiler_1ae45782f37eaa20c9ccb117874abc7ff2) 

#### `public float `[`velocitySet`](#structmotion_profiler_1abecf199e03d7c61b7694435888364aaf) 

#### `public float `[`accelSet`](#structmotion_profiler_1ae3f53b386932f91b87cfd744c4532603) 

#### `public float `[`jerk`](#structmotion_profiler_1ad12385a16c1e70621af252dc46ddd611) 

#### `public char `[`planComplete`](#structmotion_profiler_1a8b7b1cd195d2ce0981b7d4e85675896a) 

#### `public int `[`vMax`](#structmotion_profiler_1a939c9510d8065a680fb9390dc98c89d9) 

#### `public int `[`t1`](#structmotion_profiler_1a1f006058637a82726360f71ab3396f0e) 

#### `public int `[`t2`](#structmotion_profiler_1aa931d62492a0ef5a5a1ff1a2ed78d9c6) 

#### `public int `[`t4`](#structmotion_profiler_1a3d25457e1add6ee8836d29f078e7cc1b) 

#### `public int `[`tMax`](#structmotion_profiler_1a8b70c4e6e7bfb03961f944751725e6f2) 

#### `public float `[`cycleTime`](#structmotion_profiler_1a937382b7ba919b74e49bb7686a3b86c6) 

#### `public int `[`positionCycles`](#structmotion_profiler_1a519d6d21d377d231f1c947d83de92841) 

# struct `PID` 

[PID](#struct_p_i_d) controller data structure

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public float `[`Kp`](#struct_p_i_d_1a09cfc766a233ad617270562cc4146d07) | 
`public float `[`Ki`](#struct_p_i_d_1a59fac16f568541187ff485c4c47b0ec5) | 
`public float `[`Kd`](#struct_p_i_d_1a98268d71502ba080d88a9b1f50fdbe80) | 
`public float `[`innerIntegralBand`](#struct_p_i_d_1a032c3f5739e042d209c972fc31e57fe3) | 
`public float `[`outerIntegralBand`](#struct_p_i_d_1a02d0ee6d23eee5716d8e390aa387f237) | 
`public float `[`sigma`](#struct_p_i_d_1ae11aeeb83e22b5b7f1e6b4347eb1daa6) | 
`public float `[`lastValue`](#struct_p_i_d_1a437b26536f3832d38c5d783076d310b1) | 
`public unsigned long `[`lastTime`](#struct_p_i_d_1a22cb446e5271d5d2c4b2e23792fb1966) | 
`public float `[`lastSetPoint`](#struct_p_i_d_1a43ff6047780f4de51e46ea718c221fca) | 

## Members

#### `public float `[`Kp`](#struct_p_i_d_1a09cfc766a233ad617270562cc4146d07) 

#### `public float `[`Ki`](#struct_p_i_d_1a59fac16f568541187ff485c4c47b0ec5) 

#### `public float `[`Kd`](#struct_p_i_d_1a98268d71502ba080d88a9b1f50fdbe80) 

#### `public float `[`innerIntegralBand`](#struct_p_i_d_1a032c3f5739e042d209c972fc31e57fe3) 

#### `public float `[`outerIntegralBand`](#struct_p_i_d_1a02d0ee6d23eee5716d8e390aa387f237) 

#### `public float `[`sigma`](#struct_p_i_d_1ae11aeeb83e22b5b7f1e6b4347eb1daa6) 

#### `public float `[`lastValue`](#struct_p_i_d_1a437b26536f3832d38c5d783076d310b1) 

#### `public unsigned long `[`lastTime`](#struct_p_i_d_1a22cb446e5271d5d2c4b2e23792fb1966) 

#### `public float `[`lastSetPoint`](#struct_p_i_d_1a43ff6047780f4de51e46ea718c221fca) 

Generated by [Moxygen](https://sourcey.com/moxygen)