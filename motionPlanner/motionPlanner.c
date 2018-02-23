/// \cond IGNORE
#ifndef TRUESPEED_H
#define TRUESPEED_H

/**
 * true_speed lookup table maps linear motor input to logarithmic motor output in order
 * to improve motor control
 */
const unsigned int true_speed[128] =
{
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 88, 89, 89, 90, 90,127,127,127
};
#endif

#ifndef NERD_PID
#define NERD_PID

/**
 * PID controller data structure
 */

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float inner_integral_band;
	float outer_integral_band;
	float sigma;
	float last_value;
	unsigned long last_time;
	float last_set_point;
} PID;

/**
 * initialize pid structure, set parameters
 *
 * @param pid instance of PID structure
 * @param Kp  PID Kp constant
 * @param Ki  PID Ki constant
 * @param Kd  PID Kd constant
 * @param inner_integral_band  inner bound of PID I summing cutoff
 * @param outer_integral_band  outer bound of PID I summing cutoff
 */
void
pid_init (PID pid, float Kp, float Ki, float Kd, float inner_integral_band, float outer_integral_band) {
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
	pid.inner_integral_band = inner_integral_band;
	pid.outer_integral_band = outer_integral_band;
	pid.sigma = 0;
	pid.last_value = 0;
	pid.last_time = nPgmTime;
}

/**
 * initialize pid structure, set parameters based on another PID structure
 *
 * @param pid  instance of PID structure
 * @param to_copy  PID instance to copy settings from
 */
void pid_init_copy (PID pid, PID to_copy) {
	pid.Kp = to_copy.Ki;
	pid.Ki = to_copy.Ki;
	pid.Kd = to_copy.Kd;
	pid.inner_integral_band = to_copy.inner_integral_band;
	pid.outer_integral_band = to_copy.outer_integral_band;
	pid.sigma = 0;
	pid.last_value = 0;
	pid.last_time = nPgmTime;
}

/**
 * calculate pid output
 *
 * @param pid  instance of PID structure
 * @param set_point  set point of PID controller
 * @param sensor_read  sensor/feedback value
 *
 * @return  output value of the control loop
 */
float
pid_calculate (PID pid, float set_point, float sensor_read) {
	float delta_time = (nPgmTime - pid.last_time)/1000.0;
	pid.last_time = nPgmTime;

	float sensor_rate = 0;
	if(delta_time > 0)
		sensor_rate = (sensor_read - pid.last_value) / delta_time;
	pid.last_value = sensor_read;

	float error = set_point - sensor_read;

	if(fabs(error) > pid.inner_integral_band && fabs(error) < pid.outer_integral_band)
		pid.sigma += error * delta_time;

	if (fabs (error) > pid.outer_integral_band)
		pid.sigma = 0;

	float output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- sensor_rate * pid.Kd;

	return output;
}

/**
 * calculate PID output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error
 *
 * @param pid  the PID controller to use for the calculation
 * @param set_point  the set point of the system
 * @param sensor_read  the value of the feedback sensor in the system
 * @param velocity_set  the velocity set point of the system
 *
 * @return  the output value of the control loop
 */
float
pid_calculate_with_velocity_set (PID pid, float set_point, float sensor_read, float velocity_set) {
	float delta_time = (nPgmTime - pid.last_time)/1000.0;
	pid.last_time = nPgmTime;

	float sensor_rate = 0;
	if(delta_time > 0)
		sensor_rate = (sensor_read - pid.last_value) / delta_time + velocity_set;
	pid.last_value = sensor_read;

	float error = set_point - sensor_read;

	if(fabs(error) > pid.inner_integral_band && fabs(error) < pid.outer_integral_band)
		pid.sigma += error * delta_time;

	if (fabs (error) > pid.outer_integral_band)
		pid.sigma = 0;

	float output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- sensor_rate * pid.Kd;

	return output;
}

/**
 * calculate PID output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.
 *
 * @param pid  the PID controller to use for the calculation
 * @param set_point  the set point of the system
 * @param sensor_read  the value of the feedback sensor in the system
 *
 * @return  the output value of the control loop
 */
float
pid_calculate_velocity (PID pid, int set_point, int sensor_read) {
	float delta_time = (nPgmTime - pid.last_time)/1000.0;
	pid.last_time = nPgmTime;

	float sensor_rate = 0;
	if(delta_time > 0)
		sensor_rate = (sensor_read - pid.last_value) / delta_time;
	pid.last_value = sensor_read;

	float error = set_point - sensor_read;

	if(fabs(error) > pid.inner_integral_band && fabs(error) < pid.outer_integral_band)
		pid.sigma += error * delta_time;

	if (fabs (error) > pid.outer_integral_band)
		pid.sigma = 0;

	float output = set_point * pid.Kp
					+ pid.sigma * pid.Ki
					- sensor_rate * pid.Kd;

	return output;
}
#endif


/// \cond IGNORE
#ifndef NERD_MOTIONPLANNER
#define NERD_MOTIONPLANNER

#define MOVE_BUFFER_SIZE 10

#define SETTING_INACTIVE 0x0
#define SETTING_ACTIVE 0x1
#define SETTING_ACTIVEPOSITION 0x2
#define SETTING_MIRROR_REVERSE 0x3

struct move {
	long start_time;
	long time_limit;
	float target_velocity;
	char move_not_executed;
};

struct motion_profiler {
	PID position_controller;
	PID velocity_controller;

	float Kv;
	float Ka;
	float jerk_limit;
	void *sensor;
	int velocity_filter [5];
	int velocity_read;

	char profile_setting;
	short motor_output;
	float position_out;
	float last_sensor_value;
	long last_measure_time;
	long last_compute_time;
	char cycle_counter;
	long move_start_time;
	float max_acceleration;
	float position_target;

	float velocity_target;

	float position_set; //position set point
	float velocity_set; //velocity set point
	float acceleration_set; //acceleration output
	float jerk; //rate of acceleration change

	float max_velocity; //max rate of system, in sensor units/second
	int acceleration_time; //time for velocity ramping
	int motor_to_follow; //motor port to mirror

	// + jerk
	long t1; //time to set jerk to 0
	// 0 jerk
	long t2; //time to set jerk to decelerate (acceleration_set -= jerk)
	// - jerk
	long t3; //time to set jerk to 0
	// 0 jerk

	move move_buffer [MOVE_BUFFER_SIZE]; //random access buffer for queueing move commands

	float cycle_time;
	int position_cycles; //amount of cycles to wait before new position update
};

void set_position_controller (int motor_port, float Kp, float Ki, float Kd, float inner_band, float outer_band);
void set_velocity_controller (int motor_port, float Kp, float Ki, float Kd, float inner_band, float outer_band);

motion_profiler profiler_pool[10]; //  because of ROBOTC not being true C we need to allocate space for profilers at compile time instead of instantiating them as we need them
motion_profiler* motor_controller [10];
motion_profiler* unique_controllers [10];

//sensor variable
int raw_sensor_value [20];
/// \endcond

float
convert_to_float (void *var) {
	int *i = var;
	float *f = var;

	if (*i == *var)
		return *var;

	return *f;
}

void
queue_move (motion_profiler *profile, long start_time, float target_velocity) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->move_buffer [i].move_not_executed == 0) {
			profile->move_buffer [i].start_time = start_time;
			profile->move_buffer [i].target_velocity = target_velocity;
			profile->move_buffer [i].move_not_executed = 1;
			profile->move_buffer [i].time_limit = profile->acceleration_time;
			return;
		}
	}
}

void
queue_move_with_time_limit (motion_profiler *profile, long start_time, float target_velocity, long time_limit) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->move_buffer [i].move_not_executed == 0) {
			profile->move_buffer [i].start_time = start_time;
			profile->move_buffer [i].target_velocity = target_velocity;
			profile->move_buffer [i].move_not_executed = 1;
			profile->move_buffer [i].time_limit = time_limit;
			return;
		}
	}
}

char
has_move_queued (motion_profiler *profile) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i)
		if (profile->move_buffer[i].move_not_executed == 1)
			return 1;

	return 0;
}

void
clear_move_queue (motion_profiler *profile) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->move_buffer [i].move_not_executed)
			profile->move_buffer [i].move_not_executed = 0;
	}
}

/**
 * return a pointer to a ROBOTC sensor
 *
 * @param port  the sensor value to get a pointer to
 *
 * @return  the pointer to the sensor value
 */
int*
get_sensor_pointer (int port) {
	if (port < 0 || port > 19)
		return NULL;
	return &raw_sensor_value [port];
}

/**
 * create a motion profile for a motor/sensor pair. PID controllers for the motion profile will be set to default with 0 feedback control and a neutral feedforward gain of 127.0/max_velocity
 *
 * @param motor_port  the motor port to create a profile for
 * @param sensor  a pointer to the sensor value to monitor. This can be a pointer to any integer, or a "raw" sensor value using get_sensor_pointer().
 * @param max_velocity  the maximum velocity to use when calculating moves
 * @param Ka  acceleration constant used when ramping up/down velocity during moves
 * @param t1  time to spend at peak acceleration at the beginning/end of a move. This and jerk_limit will determine the shape of the motion curve
 * @param jerk_limit  the amount to limit the jerk by (0.0 to 1.0). The higher this value is, the more curved the acceleration profile will be. This should usually be 0.5
 * @param cycle_time  polling rate/sample period of system. Polling frequency = 1000/cycle_time. Note that ports 2-9 on cortex only can update at a frequency of 18.5Hz, so values less than ~20 here will offer diminishing returns.
 * @param position_cycles  cycles to skip for position updates during moves. This will generally be 3-5
 */
void
set_motion_profiler_custom (int motor_port, void *sensor, float max_velocity, float Ka, int t1, float jerk_limit, int cycle_time, int position_cycles) {
	if (motor_port < 0 || motor_port > 9)
		return;

	if (motor_controller [motor_port] != NULL)
		return;

	if (jerk_limit > 1.0)
		jerk_limit = 1.0;
	if (jerk_limit < 0)
		jerk_limit = 0;

	int i;
	motion_profiler *controller;

	for (i = 0; i < 10; i++) {
		if (unique_controllers[i] != NULL)
			continue;

		unique_controllers [i] = &(profiler_pool [i]);
		controller = unique_controllers [i];
		break;
	}

	motor_controller [motor_port] = controller;
	controller->jerk_limit = jerk_limit;
	controller->sensor = sensor;
	controller->velocity_read = 0;
	controller->profile_setting = SETTING_INACTIVE;
	controller->motor_output = 0;
	controller->last_sensor_value = convert_to_float(sensor);
	controller->last_measure_time = nPgmTime;
	controller->last_compute_time = nPgmTime;
	controller->max_velocity = max_velocity;
	controller->position_out = 0;
	controller->Ka = Ka;

	controller->velocity_filter[0] = 0;
	controller->velocity_filter[1] = 0;
	controller->velocity_filter[2] = 0;
	controller->velocity_filter[3] = 0;
	controller->velocity_filter[4] = 0;

	controller->acceleration_time = t1;
	controller->t1 = 0;
	controller->t2 = 0;
	controller->t3 = 0;

	controller->cycle_time = cycle_time;
	controller->position_cycles = position_cycles;

	pid_init (controller->position_controller, 0, 0, 0, 30, 150);
	pid_init (controller->velocity_controller, 127.0/max_velocity, 0, 0, 50, 500);
}

void
set_motion_profile (int motor_port, tSensors sensor, float max_velocity, float Ka, int t1, float jerk_limit, int cycle_time, int position_cycles) {
	if (sensor < in1 || sensor > dgtl12)
		return;

	int *sensor_pointer = get_sensor_pointer(sensor);
	set_motion_profiler_custom(motor_port, sensor_pointer, max_velocity, Ka, t1, jerk_limit, cycle_time, position_cycles);
}

void
set_simple_motion_profile (int motor_port, tSensors sensor, float max_velocity) {
	if (sensor < in1 || sensor > dgtl12)
		return;

	int *sensor_pointer = get_sensor_pointer(sensor);
	set_motion_profiler_custom (motor_port, sensor_pointer, max_velocity, 0.0, 600, 0.5, 20, 4);
	set_velocity_controller (motor_port, 127.0/max_velocity, 0.0, 0.0, 50, 400);
	set_position_controller (motor_port, 5.0, 0.0, 0.0, 30, 150);
}

void
set_simple_motion_profile_custom (int motor_port, void *sensor, float max_velocity) {
	set_motion_profiler_custom (motor_port, sensor, max_velocity, 0.0, 600, 0.5, 20, 4);

	set_velocity_controller (motor_port, 127.0/max_velocity, 0.0, 0.0, 50, 400);
	set_position_controller (motor_port, 5.0, 0.0, 0.0, 30, 150);
}

void
update_motion_profile_custom (int motor_port, void *sensor, float max_velocity, float Ka, int t1, float jerk_limit, int cycle_time, int position_cycles) {
	if (motor_port < 0 || motor_port > 9)
		return;
	if (motor_controller[motor_port] == NULL)
		return;

	motion_profiler *controller = motor_controller[motor_port];

	if (jerk_limit > 1.0)
		jerk_limit = 1.0;
	if (jerk_limit < 0)
		jerk_limit = 0;

	controller->sensor = sensor;
	controller->max_velocity = max_velocity;
	controller->Ka = Ka;
	controller->acceleration_time = t1;
	controller->cycle_time = cycle_time;
	controller->jerk_limit = jerk_limit;
	controller->position_cycles = position_cycles;
}

void
update_motion_profile (int motor_port, tSensors sensor, float max_velocity, float Ka, int t1, float jerk_limit, int cycle_time, int position_cycles) {
	if (sensor < in1 || sensor > dgtl12)
		return;

	int *sensor_pointer = get_sensor_pointer(sensor);

	update_motion_profile_custom(motor_port, sensor_pointer, max_velocity, Ka, t1, jerk_limit, cycle_time, position_cycles);
}

void
profile_set_max_velocity (int motor_port, float max_velocity) {
	if (motor_controller [motor_port] == NULL)
		return;

	motor_controller [motor_port]->max_velocity = max_velocity;
	motor_controller [motor_port]->velocity_controller.Kp = 127.0 / max_velocity;
}

void
profile_set_ka (int motor_port, float Ka) {
	if (motor_controller [motor_port] == NULL)
		return;

	motor_controller [motor_port]->Ka = Ka;
}

void
profile_set_acceleration_time (int motor_port, int t1) {
	if (motor_controller [motor_port] == NULL)
		return;

	motor_controller [motor_port]->acceleration_time = t1;
}

void
profile_set_cycle_time (int motor_port, int cycle_time) {
	if (motor_controller [motor_port] == NULL)
		return;

	motor_controller [motor_port]->cycle_time = cycle_time;
}

void
profile_set_position_frequency (int motor_port, int position_cycles) {
	if (motor_controller [motor_port] == NULL)
		return;

	motor_controller [motor_port]->position_cycles = position_cycles;
}

void
profile_set_jerk_limit (int motor_port, float jerk_limit) {
	if (motor_controller [motor_port] == NULL)
		return;

	if (jerk_limit > 1.0)
		jerk_limit = 1.0;
	else if (jerk_limit < 0.0)
		jerk_limit = 0.0;

	motor_controller [motor_port]->jerk_limit = jerk_limit;
}

/**
 * set the position PID controller for the specified motor's motion profile
 *
 * @param motor_port  the motor to update
 * @param Kp  proportional gain
 * @param Ki  integral gain
 * @param Kd  derivative gain
 * @param inner_band  the inner integral deadBand value
 * @param outer_band  the outer integral deadBand value
 */
void
set_position_controller (int motor_port, float Kp, float Ki, float Kd, float inner_band, float outer_band) {
	if (motor_controller [motor_port] == NULL)
		return;

	motion_profiler *profile = motor_controller [motor_port];
	pid_init (profile->position_controller, Kp, Ki, Kd, inner_band, outer_band);
}

/**
 * set the velocity PID controller for the specified motor's motion profile
 *
 * @param motor_port  the motor to update
 * @param Kp  proportional gain
 * @param Ki  integral gain
 * @param Kd  derivative gain
 * @param inner_band  the inner integral deadBand value
 * @param outer_band  the outer integral deadBand value
 */
void
set_velocity_controller (int motor_port, float Kp, float Ki, float Kd, float inner_band, float outer_band) {
	if (motor_controller [motor_port] == NULL)
		return;

	motion_profiler *profile = motor_controller [motor_port];
	pid_init (profile->velocity_controller, Kp, Ki, Kd, inner_band, outer_band);
}


/**
 * set a motor to copy another motor's motion profile and mirror its output value
 *
 * @param motor_port  the motor to have mirror another motor
 * @param master_port  the motor to mirror
 */
void
set_motion_slave (int motor_port, int master_port) {
	if (motor_controller [master_port] == NULL)
		return;
	motor_controller [motor_port] = motor_controller [master_port];
}

void
set_motion_slave_reversed (int motor_port, int master_port) {
	if (motor_controller [master_port] == NULL)
		return;

	if (motor_controller[motor_port] == NULL)
		set_simple_motion_profile_custom (motor_port, motor_controller[master_port]->sensor, motor_controller[master_port]->max_velocity);

	motor_controller[motor_port]->motor_to_follow = master_port;
	motor_controller[motor_port]->profile_setting = SETTING_MIRROR_REVERSE;
}

/**
 * issue a move command to the specified position. This will currently do nothing if the move is considered to be a "short move", ie. the motor is unable to fully ramp up to max velocity during the move. Note that this is an absolute position command, so two consecutive moves to 4000 are not equivalent to a single move to 8000.
 *
 * @param motor_port  the motor to issue the move command to
 * @param position  the position to move to
 */
void
set_position (int motor_port, int position) {
	if (motor_controller [motor_port] == NULL)
		return;

	motion_profiler *profile = motor_controller [motor_port];

	float distance = position - convert_to_float(profile->sensor);
	float initialVelocity = profile->velocity_read;
	float velocity_error = sgn (distance) * profile->max_velocity - initialVelocity;
	float ramp_up_time = fabs((sgn(distance) * profile->max_velocity - initialVelocity)/profile->max_velocity * profile->acceleration_time);
	float rampUpDist = 0.0005 * (ramp_up_time-profile->acceleration_time) * initialVelocity + profile->acceleration_time / 2000.0 * profile->max_velocity * sgn (velocity_error);
	float rampDownDist = profile->acceleration_time * profile->max_velocity / 2000.0 * sgn (velocity_error);
	float cruiseTime = sgn (distance) * (distance - rampUpDist - rampDownDist) / profile->max_velocity * 1000.0;
	long decelTime = cruiseTime + ramp_up_time + nPgmTime;

	clear_move_queue(profile);
	queue_move_with_time_limit (profile, nPgmTime, sgn(distance) * profile->max_velocity, decelTime - nPgmTime);
	queue_move_with_time_limit (profile, decelTime, 0, decelTime - nPgmTime);
	profile->profile_setting = SETTING_ACTIVEPOSITION;
	profile->position_target = position;
}

/**
 * set a motor's output to a value from -127 to 127
 *
 * @param motor_port  the motor to set the output value of
 * @param output  output value to set
 */
void
set_pwm_output (int motor_port, int output) {
	if (motor_controller [motor_port] == NULL)
		return;

	if (output > 127)
		output = 127;
	if (output < -127)
		output = -127;

	clear_move_queue (motor_controller[motor_port]);
	motor_controller[motor_port]->profile_setting = SETTING_INACTIVE;
	motor_controller[motor_port]->motor_output = output;
	motor_controller[motor_port]->velocity_set = 0;
	motor_controller[motor_port]->position_target = 0;
	motor_controller[motor_port]->position_set = 0;
}

/**
 * set a motor's velocity to the specified value
 *
 * @param motor_port  the motor to set
 * @param velocity  desired velocity
 */
void
set_velocity (int motor_port, float velocity) {
	if (motor_controller [motor_port] == NULL)
		return;

	motion_profiler *profile = motor_controller[motor_port];
	profile->profile_setting = SETTING_ACTIVE;
	profile->position_set = convert_to_float (profile->sensor);
	profile->position_target = convert_to_float (profile->sensor);

	clear_move_queue (profile);
	queue_move (profile, nPgmTime, velocity);
}

/// @private
void
update_motors () {
	int i;

	for (i = 0; i < 10; ++i) {
		if (motor_controller [i] == NULL)
			continue;

		if (motor_controller[i]->profile_setting == SETTING_MIRROR_REVERSE)
			motor_controller[i]->motor_output = -motor_controller[motor_controller[i]->motor_to_follow]->motor_output;

		if (motor_controller [i]->motor_output > 127)
			motor_controller [i]->motor_output = 127;
		else if (motor_controller [i]->motor_output < -127)
			motor_controller [i]->motor_output = -127;

		motor[i] = sgn(motor_controller [i]->motor_output) * true_speed[(int) fabs (motor_controller [i]->motor_output)];
	}
}

/// @private
void
measure_velocity (motion_profiler *profile) {
	float deltaT = (nPgmTime - profile->last_measure_time)/1000.0;
	float sensor_value = convert_to_float (profile->sensor);

	//get sensor velocity, ticks per second
	float sensor_rate = deltaT == 0 ? 0 : (sensor_value - profile->last_sensor_value) / deltaT;

	for (int j = 4; j > 0; --j) {
		profile->velocity_filter [j] = profile->velocity_filter [j-1];
	}
	profile->velocity_filter [0] = sensor_rate;

	sensor_rate = profile->velocity_filter [0] * 0.5 + profile->velocity_filter [1] * 0.25 + profile->velocity_filter [2] * 0.125 + profile->velocity_filter [3] * 0.0625 + profile->velocity_filter [4] * 0.0625;
	profile->velocity_read = sensor_rate;
	profile->last_sensor_value = sensor_value;
}

void
start_move (motion_profiler *profile, float target_velocity, long time_limit) {
	profile->velocity_target = target_velocity;
	float velocity_error = target_velocity - profile->velocity_read;
	long ramp_up_time = fabs(velocity_error / profile->max_velocity * profile->acceleration_time);

	if (ramp_up_time > time_limit)
		ramp_up_time = time_limit;

	long jerk_time = ramp_up_time / 2.0 * profile->jerk_limit;

	if (ramp_up_time < jerk_time) {
		jerk_time = 0;
	}

	if (ramp_up_time != 0)
		profile->max_acceleration = velocity_error / (ramp_up_time - jerk_time) * 1000.0;
	else
		profile->max_acceleration = 0;

	if (jerk_time > 0)
		profile->jerk = profile->max_acceleration / jerk_time * 1000.0;
	else
		profile->jerk = 0;

	profile->t1 = jerk_time;
	profile->t2 = ramp_up_time - jerk_time;
	profile->t3 = ramp_up_time;

	profile->move_start_time = -1;
}

/// @private
void
profile_update (motion_profiler *profile) {
	if (profile->move_start_time == -1)
		profile->move_start_time = nPgmTime;

	float moveTime = nPgmTime - profile->move_start_time;
	float delta_time = nPgmTime - profile->last_compute_time;

	if (profile->last_compute_time < profile->move_start_time) {
		delta_time -= profile->move_start_time - profile->last_measure_time;
		if (delta_time < 0)
			delta_time = 0;
	}
	profile->last_compute_time = nPgmTime;

	if (nPgmTime < (unsigned long) (profile->move_start_time + profile->t1)) { // t0
		profile->acceleration_set = profile->jerk * moveTime / 1000.0;
	} else if (nPgmTime > (unsigned long) (profile->move_start_time + profile->t1) && nPgmTime < (unsigned long) (profile->move_start_time + profile->t2)) { // t1
		profile->acceleration_set = profile->max_acceleration;
	} else if (nPgmTime > (unsigned long) (profile->move_start_time + profile->t2) && nPgmTime < (unsigned long) (profile->move_start_time + profile->t3)) { // t2
		profile->acceleration_set = profile->max_acceleration - profile->jerk * (moveTime - profile->t2) / 1000.0;
	} else if (nPgmTime > (unsigned long) (profile->move_start_time + profile->t3)) { // t3
		profile->acceleration_set = 0;
		profile->velocity_set = profile->velocity_target;
	}

	profile->velocity_set += profile->acceleration_set * delta_time/1000.0;
	profile->position_set += profile->velocity_set * delta_time/1000.0;

	//do position PID if cycle includes it
	if (profile->cycle_counter % profile->position_cycles == 0) {
	 	profile->position_out = pid_calculate_with_velocity_set (profile->position_controller, profile->position_set, convert_to_float (profile->sensor), profile->velocity_set);
	}
	profile->cycle_counter++;

	//do velocity PID
	float velocity_out = pid_calculate_velocity (profile->velocity_controller, profile->position_out + profile->velocity_set, profile->velocity_read) + profile->acceleration_set * profile->Ka;
	//float velocity_out =  profile->velocity_set * profile->Kv + profile->acceleration_set * profile->Ka;//pid_calculate (profile->velocity_controller, profile->velocity_set + profile->position_out, profile->velocity_read);

	//set motor PWM output
	profile->motor_output = velocity_out;

	if (profile->motor_output > 127)
		profile->motor_output = 127;
	else if (profile->motor_output < -127)
		profile->motor_output = -127;
}

/// @private
task raw_sensor_monitor () {
	int i;

	while (1) {
		for (i = 0; i < 20; ++i) {
			raw_sensor_value [i] = SensorValue [i];
		}
	}

	abortTimeslice ();
}

/// @private
task motion_planner () {
	int i;

	startTask (raw_sensor_monitor);

	while (1) {
		for (i = 0; i < 10; ++i) {
			if (unique_controllers [i] == NULL)
				continue;

			motion_profiler *profile = unique_controllers [i];

			if (nPgmTime - profile->last_measure_time < profile->cycle_time) {
				continue;
			}

			measure_velocity (profile);
			profile->last_measure_time = nPgmTime;

			if (profile->profile_setting == SETTING_ACTIVE || profile->profile_setting == SETTING_ACTIVEPOSITION) {
				for (int j = 0; j < MOVE_BUFFER_SIZE; ++j) {
					if (profile->move_buffer[j].move_not_executed && (unsigned long) (profile->move_buffer[j].start_time) < nPgmTime) {
						start_move (profile, profile->move_buffer[j].target_velocity, profile->move_buffer[j].time_limit);
						profile->move_buffer[j].move_not_executed = 0;

						break;
					}
				}

				if (profile->profile_setting == SETTING_ACTIVEPOSITION && (unsigned long) profile->t3 < nPgmTime && profile->velocity_set == 0 && !has_move_queued(profile))
					profile->position_set = profile->position_target;

				profile_update (profile);
 			}
		}

		update_motors ();
	}

	abortTimeslice ();
}

#endif
