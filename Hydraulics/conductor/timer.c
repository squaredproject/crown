/* timer.c */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>

/* global variables */

static volatile uint8_t kHz_flag;

/* set up timer0 for periodic interrupts. Each interrupt sets a flag,
   which then can be polled in the main loop for controlled timing. */
void Timer0_Init(void) {

  TCCR0A |= _BV(WGM01);           // Clear Timer on Compare Match (CTC) Mode
  TCCR0B = _BV(CS01) | _BV(CS00); // Pre-scaler=64, interrupt every 2^14 clocks
  TCNT0 = 0;                      // reset TCNT0
  OCR0A = 250;                    // set TOP
  TIMSK0 |= _BV(OCIE0A);          // enable Timer1 interrupt on TOP
  TIFR0 = _BV(OCF0A);             // clear OC0A = TOP flag
}

int Timer0_flag_is_set(void) {
  if (kHz_flag) {
    kHz_flag = 0;
    return (1);
  } else
    return (0);
}

// --------------------------------------------------------------------------

SIGNAL(TIMER0_COMPA_vect) {
  /*  1 kHz */
  // 16 000 000 / (250 * 64) = 1000
  kHz_flag = 1;
}
