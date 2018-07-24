/*
 * mode2.c
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



void apply_ui_mode2( int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode)
{
	switch(knuckle) {

	case 0:

		break;
	case 1:
		*amplitude_des = knob_to_position(knob[finger]);
		break;
	case 2:
		*amplitude_des = knob_to_position(knob[finger + 4]);
		break;
	}

	*phase = 0;
	*frequency = 16;
	*direct_position_mode = 1;
}

void lcd_mode2(int32_t knob[8])
{
	char this_lcd_item[8][10];

	// Left Side of LCD
	sprintf(this_lcd_item[0],"F3K2:%03d", 	position_to_percent(knob_to_position(knob[3])));
	sprintf(this_lcd_item[1],"F2K2:%03d", 	position_to_percent(knob_to_position(knob[2])));
	sprintf(this_lcd_item[2],"F1K2:%03d", 	position_to_percent(knob_to_position(knob[1])));
	sprintf(this_lcd_item[3],"F0K2:%03d", 	position_to_percent(knob_to_position(knob[0])));

	// Right Side of LCD
	sprintf(this_lcd_item[4],"F3K3:%03d", 	position_to_percent(knob_to_position(knob[7])));
	sprintf(this_lcd_item[5],"F2K3:%03d", 	position_to_percent(knob_to_position(knob[6])));
	sprintf(this_lcd_item[6],"F1K3:%03d", 	position_to_percent(knob_to_position(knob[5])));
	sprintf(this_lcd_item[7],"F0K3:%03d", 	position_to_percent(knob_to_position(knob[4])));

	lcd_items(this_lcd_item, 2);
}

