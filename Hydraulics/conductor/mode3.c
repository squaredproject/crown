/*
 * mode3.c
 *
 *  Created on: Apr 5, 2012
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
#include "conductor.h"
#include "lcd.h"
#include "knob.h"



void apply_ui_mode3( int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode)
{
	*amplitude_des = knob_limit(knobs_to_amplitude( knob[knuckle + 5], knob[0]));
	*phase = (int32_t)finger * knob_to_phase(knob[3]) + (int32_t)knuckle * knob_to_phase(knob[2]);
	*frequency = 16;
	*direct_position_mode = 0;
}

void lcd_mode3(int32_t knob[8])
{
	char this_lcd_item[8][10];

	// Left Side of LCD
	sprintf(this_lcd_item[0],"FPH:%03d", 	phase_to_deg(knob_to_phase(knob[3])));
	sprintf(this_lcd_item[1],"NPH:%03d", 	phase_to_deg(knob_to_phase(knob[2])));
	sprintf(this_lcd_item[2]," ");
	sprintf(this_lcd_item[3],"AMP:%03d", 	amplitude_to_percent(knob[0]>>2));

	// Right Side of LCD
	sprintf(this_lcd_item[4],"AMPK3:%03d", 	amplitude_to_percent(knob_limit(knobs_to_amplitude( knob[7], knob[0]))));
	sprintf(this_lcd_item[5],"AMPK2:%03d", 	amplitude_to_percent(knob_limit(knobs_to_amplitude( knob[6], knob[0]))));
	sprintf(this_lcd_item[6],"AMPK1:%03d", 	amplitude_to_percent(knob_limit(knobs_to_amplitude( knob[5], knob[0]))));
	sprintf(this_lcd_item[7],"PER:%03d", 	(int16_t)((knob[4]>>1) + MIN_PERIOD)/10 );

	lcd_items(this_lcd_item, 3);
}

