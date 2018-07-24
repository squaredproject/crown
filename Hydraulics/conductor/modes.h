/*
 * modes.h
 *
 *  Created on: Apr 9, 2012
 *      Author: earthmine
 */

#ifndef MODES_H_
#define MODES_H_

void apply_ui_modes(uint8_t mode, int32_t pacer_theta, int32_t knob[8], int32_t amplitude_des[NUMBER_FINGERS][KNUCKLES_PER_FINGER],
		int32_t phase[NUMBER_FINGERS][KNUCKLES_PER_FINGER], uint8_t frequency[NUMBER_FINGERS][KNUCKLES_PER_FINGER],
		uint8_t *direct_position_mode);

void lcd_mode(uint8_t mode, int32_t knob[8]);


#endif /* MODES_H_ */
