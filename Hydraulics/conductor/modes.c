/*
 * modes.c
 *
 *  Created on: Apr 9, 2012
 *      Author: earthmine
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

void apply_ui_mode0(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode1(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode2(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode3(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode4(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode5(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode6(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);
void apply_ui_mode7(int32_t knob[8], int32_t pacer_theta, uint8_t finger, uint8_t knuckle, int32_t *amplitude_des, int32_t *phase, uint8_t *frequency, uint8_t *direct_position_mode);

void lcd_mode0(int32_t knob[8]);
void lcd_mode1(int32_t knob[8]);
void lcd_mode2(int32_t knob[8]);
void lcd_mode3(int32_t knob[8]);
void lcd_mode4(int32_t knob[8]);
void lcd_mode5(int32_t knob[8]);
void lcd_mode6(int32_t knob[8]);
void lcd_mode7(int32_t knob[8]);

void apply_ui_modes(uint8_t mode, int32_t pacer_theta, int32_t knob[8], int32_t amplitude_des[NUMBER_FINGERS][KNUCKLES_PER_FINGER],
		int32_t phase[NUMBER_FINGERS][KNUCKLES_PER_FINGER], uint8_t frequency[NUMBER_FINGERS][KNUCKLES_PER_FINGER],
		uint8_t *direct_position_mode)
{
	uint8_t finger;
	uint8_t knuckle;
	for(finger = 0; finger < NUMBER_FINGERS;finger++)
	{
		for(knuckle = 0; knuckle < KNUCKLES_PER_FINGER;knuckle++)
		{
			switch (mode)
			{
				case 0:
					apply_ui_mode0(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;

				case 1:
					apply_ui_mode1(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;

				case 2:
					apply_ui_mode2(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;

				case 3:
					apply_ui_mode3(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;

				case 4:
					apply_ui_mode4(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;
				case 5:
					apply_ui_mode5(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;
				case 6:
					apply_ui_mode6(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;
				case 7:
					apply_ui_mode7(knob, pacer_theta, finger, knuckle, &amplitude_des[finger][knuckle], &phase[finger][knuckle], &frequency[finger][knuckle], &(*direct_position_mode));
					break;
				}
			}
		}
}

void lcd_mode(uint8_t mode, int32_t knob[8])
{
	switch (mode)
	{
		case 0:
			lcd_mode0(knob);
			break;

		case 1:
			lcd_mode1(knob);
			break;

		case 2:
			lcd_mode2(knob);
			break;

		case 3:
			lcd_mode3(knob);
			break;

		case 4:
			lcd_mode4(knob);
			break;
		case 5:
			lcd_mode5(knob);
			break;
		case 6:
			lcd_mode6(knob);
			break;
		case 7:
			lcd_mode7(knob);
			break;
		}

}
