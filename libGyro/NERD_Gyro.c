/* -----------------------------------------------------------------------------
                                  MIT License

                        Copyright (c) 2018 Jason McKinney

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

--------------------------------------------------------------------------------
	NERD_Gyro.c

	Created:  2016-06-30
	
	Minor Revisions:
	-	v1.0.0  Initial Release

--------------------------------------------------------------------------------	
	The author asks that proper attribution be given for this software should the
	source be unavailable (for example, if compiled into a binary/used on a robot).

	The author can be contacted via email at jason_at_jmmckinney_dot_net
	or on the VEX Forum as jmmckinney.
-------------------------------------------------------------------------------- */


#ifndef NERD_GYRO
#define NERD_GYRO

struct gyro_config{
	float std_deviation;
	float avg;
	float volts_per_degree_per_second;
	char gyro_flipped;
};

typedef struct {
	struct gyro_config config;
	int port_number;
} Gyro;

//ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 5

#define GYRO_OVERSAMPLE 2

//points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 1500

float calibration_buffer [GYRO_CALIBRATION_POINTS];

float gyro_get_rate (Gyro gyro);

/**
 * generate calibration data for the gyro by collecting
 * zero movement data for reference when reading data later
 *
 * @param gyro instance of gyro structure
 */
void
gyro_calibrate (Gyro gyro){
	float raw_average = 0.0;
	float std_deviation = 0.0;

	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float raw = SensorValue (gyro.port_number);
		raw_average += raw;
		calibration_buffer [i] = raw;
		delay (1);
	}
	raw_average /= GYRO_CALIBRATION_POINTS;
	gyro.config.avg = raw_average;

	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		std_deviation += fabs (raw_average - calibration_buffer [i]);
	std_deviation /= (float) GYRO_CALIBRATION_POINTS;

	gyro.config.std_deviation = std_deviation;

	/*
	 * Datasheet from VEX indicates that the sensitivity of the gyro is 1.1mV/dps
	 * and the cortex ADC for raw analog reads ranges from 0-4095 for 0v-5v
	 * readings. The gyro is scaled from the nominal 2.7v-3.6v operating range
	 * that the actual chip has to work on the cortex's 5v logic voltage. The scale multiplier
	 * value is in the ballpark of 1.515.
	 */
	gyro.config.volts_per_degree_per_second = 0.0011 * 1.515;
}

/**
 * initialize gyro and run the calibration subroutine
 *
 * @param gyro instance of gyro structure
 * @param port_number the port number of the gyro
 */
void
gyro_init (Gyro gyro, int port_number, char gyro_flipped) {
	gyro.port_number = port_number;
	gyro.config.gyro_flipped = gyro_flipped;
	gyro_calibrate (gyro);
}

/**
 * calculate filtered gyro rate data, ignoring anything within
 * GYRO_STD_DEVS standard deviations of the average gyro
 * rate value at zero motion
 *`
 * @param gyro instance of gyro structure
 *
 * @return gyro rate, in degrees per second
 */
float
gyro_get_rate (Gyro gyro){
	float gyro_read = 0.0;

	#if defined (GYRO_OVERSAMPLE)
		if (GYRO_OVERSAMPLE > 0) {
			int sample_sum = 0;
			int n_samples = pow (4, GYRO_OVERSAMPLE);

			for (int i = 0; i < n_samples; ++i)
				sample_sum += SensorValue(gyro.port_number);
			gyro_read = (float) sample_sum / (float) n_samples;
		}
		else
			gyro_read = SensorValue (gyro.port_number);
	#else
		gyro_read = SensorValue (gyro.port_number);
	#endif

	//Difference from zero-rate value or the average calibration read
	float difference = gyro_read - gyro.config.avg;

	//Difference fro zero-rate value, in volts
	float gyro_voltage = difference * 5.0 / 4095.0;

	if (fabs (difference) > GYRO_STD_DEVS * gyro.config.std_deviation)
		if (gyro.config.gyro_flipped)
			return -1 * gyro_voltage / gyro.config.volts_per_degree_per_second;
		else
			return gyro_voltage / gyro.config.volts_per_degree_per_second;
	return 0;
}
#endif
