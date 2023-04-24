// -----------------------------------------------------------------------
//
//	File: conductor.h
//	main header file for the conductor in WAVE
//
//	Written by Michael Prados, April 1, 2012
//
// -----------------------------------------------------------------------

#ifndef CONDUCTOR_H_
#define CONDUCTOR_H_

#define NUMBER_FINGERS 4
#define KNUCKLES_PER_FINGER 3

#define FINGER_INDEX 0
#define KNUCKLE_INDEX 0

#define CENTER_POSITION 0 // arbitrary choice relative to sensor position
#define MAX_AMPLITUDE 256 // in sensor units
#define MAX_SPEED 20      // position units per sample, first guess

#define MAX_AMPLITUDE_RATE 16

#define DEBUGFILE &uart0file
#define NETWORKFILE &uart1file
#define LCDFILE &uart2file

#define set_bit(var, mask) ((var) |= _BV(mask))
#define clear_bit(var, mask) ((var) &= ~(_BV(mask)))

#define MIN_PERIOD 100

#define KNOB_DEAD_ZONE 64

// dual defined here and in sinetable.h . Change both if needed.
#ifndef TABLE_LENGTH
#define TABLE_LENGTH 1024
#endif /* TABLE_LENGTH */

#endif /* CONDUCTOR_H_ */
