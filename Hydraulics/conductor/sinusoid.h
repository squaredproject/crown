// -----------------------------------------------------------------------
//
//	File: sinusoid.h
//	header file for sinusoid generation in WAVE
//
//	Written by Michael Prados, April 1, 2012
//
// -----------------------------------------------------------------------

#ifndef SINUSOID_H_
#define SINUSOID_H_

void initialize_sinusoids(void);
void apply_ui_to_sinusoids( int32_t knob[8], uint8_t mode );
void pace_sinusoids(void);
void generate_positions(void);
void stream_positions(void);
void stream_debug( void );
int32_t sine_table(int32_t phi, int32_t amplitude);

extern uint8_t sine_debug_mode;
enum{ SINE_DEBUG_NONE, SINE_DEBUG_POSITIONS, SINE_DEBUG_THETA };

#endif /* SINUSOID_H_ */
