/*
 * knob.c
 *
 *  Created on: Apr 9, 2012
 *      Author: michael
 */

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
#include <string.h>
#include "conductor.h"
#include "knob.h"

int32_t knob_limit( int32_t knob_in )
{
	if (knob_in > 255)
		return(255);
	else if (knob_in < -255)
		return(-255);
	else
		return(knob_in);
}

int32_t knob_to_position( int32_t knob_in )
{
	if (knob_in < (512 - (KNOB_DEAD_ZONE >> 1)))
	{
		//return(-100);
		return((knob_in - 511 + (KNOB_DEAD_ZONE >> 1)) * 255 / (511 - (KNOB_DEAD_ZONE >> 1)));
	} else if (knob_in > (512 + (KNOB_DEAD_ZONE >> 1)))
	{
		//return(100);
		return((knob_in - 511 - (KNOB_DEAD_ZONE >> 1)) * 255 / (511 - (KNOB_DEAD_ZONE >> 1)));
	} else {
		return(0);
	}


	return(knob_in);
}

int16_t position_to_percent( int32_t position )
{
	return( (int16_t)(position * 100 / 255) );
}

int32_t knob_to_phase( int32_t knob_in )
{
	if (knob_in < (512 - (KNOB_DEAD_ZONE >> 1)))
	{
		//return(-100);
		return((knob_in - 511 + (KNOB_DEAD_ZONE >> 1)) * 256 / (511 - (KNOB_DEAD_ZONE >> 1)));
	} else if (knob_in > (511 + (KNOB_DEAD_ZONE >> 1)))
	{
		//return(100);
		return((knob_in - 511 - (KNOB_DEAD_ZONE >> 1)) * 256 / (511 - (KNOB_DEAD_ZONE >> 1)));
	} else {
		return(0);
	}
}

int16_t phase_to_deg( int32_t phase )
{
	return((int16_t)(phase * (int32_t)360/(int32_t)TABLE_LENGTH));
}

int8_t knob_to_frequency( int32_t knob_in )
{
	return((knob_in-511) * 6 / (TABLE_LENGTH-1));
}

int32_t knobs_to_amplitude( int32_t knob_in, int32_t main_amplitude_knob)
{
	return(((main_amplitude_knob >> 2) * (knob_in >> 2)) >> 7);
}

int16_t amplitude_to_percent( int32_t this_amplitude )
{
	return((int16_t)(this_amplitude * 100 / 255));
}

