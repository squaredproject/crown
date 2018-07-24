// -----------------------------------------------------------------------
//
//	File: sinusoid.c
//	c file for sinusoid generation in WAVE
//
//	Written by Michael Prados, April 1, 2012
//
// -----------------------------------------------------------------------

#include <stddef.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <util/delay.h> // needs F_CPU def

#include "conductor.h"
#include "UART0.h"
#include "UART1.h"
#include "UART2.h"
#include "a2d.h"
#include "conductor.h"
#include "sinetable.h"
#include "sinusoid.h"
#include "modes.h"

#define MAXINTERVAL	32
#define MININTERVAL 2


static int32_t pacer_theta		= 0;												// paces 16 cycles
static int32_t theta[NUMBER_FINGERS][KNUCKLES_PER_FINGER] 				= { {0} }; 	// scaled such that theta = 1024 = 2 * PI rad

static int32_t pacer_period												= 0;		// samples
static uint8_t frequency[NUMBER_FINGERS][KNUCKLES_PER_FINGER] 			= { {0} }; 	// expressed relative to pacer in 1/16ths: f_knuckle = frequency / (16 * pacer_period)
static int32_t phase[NUMBER_FINGERS][KNUCKLES_PER_FINGER] 				= { {0} }; 	// for max = 512?

static int32_t position[NUMBER_FINGERS][KNUCKLES_PER_FINGER] 			= { {0} }; 	// for max = +/- 255
static int32_t amplitude[NUMBER_FINGERS][KNUCKLES_PER_FINGER] 			= { {0} }; 	// for max = +/- 255
static int32_t amplitude_des[NUMBER_FINGERS][KNUCKLES_PER_FINGER] 		= { {0} }; 	// for max = +/- 255

uint8_t sine_debug_mode												= 0;

uint8_t direct_position_mode = 0;

int32_t sine_table(int32_t phi, int32_t amplitude);

int32_t mod_table( int32_t number, int32_t table_multiplier );

void initialize_sinusoids( void )
{
	uint8_t finger;
	uint8_t knuckle;

	pacer_theta  = 0;
	pacer_period = 160;

	for(finger = 0; finger < NUMBER_FINGERS;finger++)
	{
		for(knuckle = 0; knuckle < KNUCKLES_PER_FINGER;knuckle++)
		{
			amplitude[finger][knuckle] = 0;
			frequency[finger][knuckle] = 16;
			phase[finger][knuckle] = 0;
		}
	}

}


void apply_ui_to_sinusoids( int32_t knob[8], uint8_t mode )
{
	direct_position_mode = 0;
	pacer_period = MIN_PERIOD + (knob[4] >> 1);
	apply_ui_modes(mode, pacer_theta, knob, amplitude_des, phase, frequency, &direct_position_mode);
}


void pace_sinusoids( void )
{
	uint8_t finger;
	uint8_t knuckle;

	int32_t this_pacer_increment = TABLE_LENGTH / pacer_period;

	if (this_pacer_increment < 1)
		this_pacer_increment = 1;

	pacer_theta += this_pacer_increment;

	if (pacer_theta >= (16 * TABLE_LENGTH))
	{
		pacer_theta %= (16 * TABLE_LENGTH);
	}

	for(finger = 0; finger < NUMBER_FINGERS;finger++)
	{
		for(knuckle = 0; knuckle < KNUCKLES_PER_FINGER;knuckle++)
		{
			int32_t ideal_increment = (this_pacer_increment * frequency[finger][knuckle]) >> 4;
			if (ideal_increment < 1)
				ideal_increment = 1;

			int32_t theta_des = ((frequency[finger][knuckle] * pacer_theta) >> 4) + phase[finger][knuckle];

			theta_des = mod_table(theta_des, 16);

			int32_t theta_error = mod_table((theta_des - theta[finger][knuckle] + 8 * TABLE_LENGTH), 16 ) - 8 * TABLE_LENGTH;

			int32_t increment_ceiling = (ideal_increment * 3)>>1;
			if (increment_ceiling < 1)
				increment_ceiling = 1;							// worst case = 0, change to 1

			int32_t increment_floor = ideal_increment >> 1;	// worst case = 0, acceptable

			if (theta_error > increment_ceiling)
				theta_error = increment_ceiling;
			else if (theta_error < increment_floor)
				theta_error = increment_floor;

			theta[finger][knuckle] += theta_error;

			theta[finger][knuckle] = mod_table( theta[finger][knuckle], 16 );
		}
	}


}

void generate_positions( void )
{
	uint8_t finger;
	uint8_t knuckle;

	for(finger = 0; finger < NUMBER_FINGERS;finger++)
	{
		for(knuckle = 0; knuckle < KNUCKLES_PER_FINGER; knuckle++)
		{
			int32_t amplitude_error = amplitude_des[finger][knuckle] - amplitude[finger][knuckle];

			if ((int16_t)amplitude_error > MAX_AMPLITUDE_RATE)
			{
				amplitude_error = (int32_t)MAX_AMPLITUDE_RATE;
			} else if ((int16_t)amplitude_error < -MAX_AMPLITUDE_RATE)
			{
				amplitude_error = (int32_t)(-MAX_AMPLITUDE_RATE);
			}

			amplitude[finger][knuckle] += amplitude_error;

			if (direct_position_mode == 1)
			{
				position[finger][knuckle] = amplitude[finger][knuckle] + CENTER_POSITION;
			} else
			{
				position[finger][knuckle] = sine_table(theta[finger][knuckle], amplitude[finger][knuckle]) + CENTER_POSITION;
			}
			// FIELD HACK - reduce the amplitude by half
			position[finger][knuckle] = position[finger][knuckle] / 2;
		}
	}
}

void stream_positions( void )
{
	uint8_t finger;
	uint8_t knuckle;

	for(finger = 0; finger < NUMBER_FINGERS;finger++)
	{
		for(knuckle = 0; knuckle < KNUCKLES_PER_FINGER;knuckle++)
		{
			fprintf(NETWORKFILE,"<%u%ut%04d>", finger, knuckle+1, (int)position[finger][knuckle]);
		}
	}
	fprintf(NETWORKFILE,"\r\n");

}

void stream_debug( void )
{
	uint8_t finger;
	uint8_t knuckle;

	if (sine_debug_mode |= SINE_DEBUG_NONE)
	{
		for(finger = 0; finger < NUMBER_FINGERS;finger++)
		{
			for(knuckle = 0; knuckle < KNUCKLES_PER_FINGER;knuckle++)
			{
				switch (sine_debug_mode) {

				case SINE_DEBUG_POSITIONS:
					printf("<%u%ut%04d>", finger, knuckle+1, (int)position[finger][knuckle]);
					break;

				case SINE_DEBUG_THETA:
					printf("<%u%ut%04d>", finger, knuckle+1, (int)theta[finger][knuckle]);
					break;
				}
			}
		}
		printf("\r\n");
	}
}

int32_t sine_table(int32_t phi, int32_t amplitude) {

	int8_t this_sign = 1;

	if (phi >= 1024)
		phi %= 1024;

	while(phi < 0)
	{
		phi += 1024;
	}

	if (phi >= 512)
	{
		this_sign = -1;
		phi -= 512;
	}

	int32_t sine_value = (int32_t)pgm_read_word(&stab[(uint16_t)phi]) * amplitude;

	sine_value = sine_value >> TABLE_AMPLITUDE_BITS;

	if (this_sign < 0)
		sine_value = -sine_value;

  return ((int32_t)sine_value);
}


int32_t mod_table( int32_t number, int32_t table_multiplier)
{
	int32_t divisor = table_multiplier * TABLE_LENGTH;

	if (number >= divisor)
	{
		number %= divisor;
	} else if (number < 0)
	{
		while(number < 0)
			number += divisor;
	}

	return(number);
}



