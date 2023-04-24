/*
 * knob.h
 *
 *  Created on: Apr 9, 2012
 *      Author: michael
 */

#ifndef KNOB_H_
#define KNOB_H_

#include <stdint.h>

int32_t knob_limit(int32_t knob_in);
int32_t knob_to_position(int32_t knob_in);
int16_t position_to_percent(int32_t position);
int32_t knob_to_phase(int32_t knob_in);
int16_t phase_to_deg(int32_t phase);
int8_t knob_to_frequency(int32_t knob_in);
int32_t knobs_to_amplitude(int32_t knob_in, int32_t main_amplitude_knob);
int16_t amplitude_to_percent(int32_t this_amplitude);

#endif /* KNOB_H_ */
