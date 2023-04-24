/* timer.c */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>

/* global variables */

volatile uint8_t Timer0_100hz_Flag;
volatile uint8_t KHZ_Flag;

volatile int16_t rising1, falling1; // define times for start and end of signal
volatile int16_t rising2, falling2; // define times for start and end of signal
volatile int16_t rising3, falling3; // define times for start and end of signal

volatile int16_t counts[] = {0, 0, 0};   /* array of position counts */
volatile int16_t OFcounts[] = {0, 0, 0}; /* array of position counts */

/* TIMER ASSIGNMENTS:
Timer0: 1khz timer
Timer2: PWM timer for LED brightness
Timer3: Joint 3 encoder timer on pin PE7 (ICP3)
Timer4: Joint 1 encoder timer on pin PL0 (ICP4) (Arduino digital 48)
Timer5: Joint 2 encoder timer on pin PL1 (ICP5) (Arduino digital 49)
Joint encoders have PWM outputs: pulse length is proportional to joint angle. At
these clock and register settings: minimum pulse is a count of 8400+ maximum
pulse is a count of less than  10 encoders have been marked at a center point of
raw encoder count = 4200 timer values and overflows in counts[] and OFcounts[]
respectively CW rotation of encoder lokoing into shaft, so LEFT rotation of
(CCW) joint is negative, min is at far left
*/

/* set up timer0 for periodic interrupts. Each interrupt sets a flag,
   which then can be polled in the main loop for controlled timing. */

void Timer0_Init(void) {

  TCCR0A = 0; /* Normal op; no output pins */
  // TCCR0B = (_BV(CS02));	/* Pre-scaler=256,so interrupt every 2^16 clocks
  // */
  TCCR0B =
      _BV(CS01) | _BV(CS00); /* Pre-scaler=64, interrupt every 2^14 clocks */
  TCNT0 = 0;                 /* reset TCNT0 */
  TIMSK0 |= _BV(TOIE0);      /* enable Timer1 interrupts */
  TIFR0 = _BV(TOV0);
}

// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// BREADCRUB:
// a variable called KHZ_flag was being incremented here
// this has been moved to our captive wiring.c , where all the
// other delay stuff is happening.
// This code also seems to work only for 16 Mhz systems, which
// is grotty - consider switching to 'delay'
// SIGNAL(TIMER0_OVF_vect)
//{
/* roughly 1 kHz */
// 16 000 000 / (256 * 64) = 976.5625
//  KHZ_Flag = 1;
//}

// set up timer 2 for pwm on A and B
void PWM_Init(void) {

  DDRH = 0xFF;

  /* enable timer 3 */
  PRR0 = 0;

  /* WGM mode 5 = 0101, WGM30 & WGM32 */
  /* OC2A: not connected */
  /* PWM on OC2B (pin PH6, Arudino digital 9) */
  /* set up Timer 2B for PWM fast, 8bit, non-inverted*/
  /* WGM1 and WGM0 set */
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

  /* fast correct PWM,  32x prescale */
  TCCR2B = _BV(CS22) | _BV(CS20);

  OCR2B = 0;
}

#ifdef FOO
// set up timer 3 for pwm on A and B
void PWM3_Init(void) {

  DDRE = 0xFF;
  /* enable timer 3 */
  PRR1 = 0;

  // Enable non inverting 8Bit PWM /
  // fast PWM, non-inverting
  // 64x prescale

  /* WGM mode 5 = 0101, WGM30 & WGM32 */
  /* COM3A0 = 0 -- non inverted */
  /* COM3B0 = 1 -- inverted */
  /* set up Timer 3A and 3B for PWM fast, 8bit, inverted*/
  // TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3B0) | _BV(WGM30);
  TCCR3A = _BV(COM3A1) | _BV(COM3A0) | _BV(COM3B1) | _BV(COM3C1) | _BV(COM3C0) |
           _BV(WGM30);

  /* fast correct PWM,  64x prescale */
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

  OCR3AH = 0x00; /* only using low 8 bits */
  OCR3AL = 0xFF; /* only using low 8 bits */
  OCR3BH = 0x00; /* only using low 8 bits */
  OCR3BL = 0xFF; /* only using low 8 bits */
  OCR3CH = 0x00; /* only using low 8 bits */
  OCR3CL = 0xFF; /* only using low 8 bits */
}

#endif
/*********************************************************************/

/* set up Timer 4 for joint 1 PWM measurement*/
/* results are in counts[0], OFcounts[0] */
// set up timer 4 for pulse width measurement on PL0 ( ICP4 ) Digital pin 49

void ICP4_Init(void) {
  // PINL0 is input
  DDRL &= ~_BV(0);

  // initialize timer interrupt
  // TIMSK4 |= 0x24;
  TIMSK4 = _BV(ICIE4) | _BV(TOIE4);
  // TIMSK4 = _BV(ICIE4) ;

  /*Noise canceller, without prescaler, rising edge*/
  // TCCR4B=  0xC1;
  TCCR4B = _BV(ICNC4) | _BV(ICES4) | _BV(CS41);
  OFcounts[2] = 0;
  counts[2] = 0;
  rising1 = 0;
  falling1 = 0;
}

// overflow counter interrupts service routine
SIGNAL(TIMER4_OVF_vect) { OFcounts[2]++; }

// Timer4 capture interrupt service subroutine

SIGNAL(TIMER4_CAPT_vect) {
  /*This subroutine checks was it start of pulse (rising edge)
  or was it end (fallingedge)and performs required operations*/

  if (PINL & 0x01) { // if high level
    // save start time
    rising1 = ICR4;

    // set to trigger on falling edge
    TCCR4A = 0x00;
    TCCR4B = _BV(ICNC4) | _BV(CS41);

    // reset overflow counter
    OFcounts[2] = 0;

  } else {

    // save falling time
    falling1 = ICR4;

    // rising edge triggers next
    TCCR4B = _BV(ICNC4) | _BV(ICES4) | _BV(CS41);

    counts[2] = falling1 - rising1;

    OFcounts[2] = 0; /* got a pulse, clear overflow ctr */
  }
}

/****************************************************************************/

/* set up Timer 4 for joint 1 PWM measurement on PL1*/
/* results are in counts[0], OFcounts[0] */
// set up timer 4 for pulse width measurement on PL0 ( ICP4 ) Digital pin 49

// Timer4: Joint 1 encoder timer on pin PL0 (ICP4)
// Timer5: Joint 2 encoder timer on pin PL1 (ICP5)
// Timer3: Joint 3 encoder timer on pin PE7 (ICP3)

void ICP5_Init(void) {
  // PINL1 is input
  DDRL &= ~_BV(1);

  // initialize timer interrupt
  TIMSK5 = _BV(ICIE5) | _BV(TOIE5);

  /*Noise canceller, without prescaler, rising edge*/
  TCCR5B = _BV(ICNC5) | _BV(ICES5) | _BV(CS51);
  OFcounts[1] = 0;
  counts[1] = 0;
  rising2 = 0;
  falling2 = 0;
}

// overflow counter interrupts service routine
SIGNAL(TIMER5_OVF_vect) { OFcounts[1]++; }

SIGNAL(TIMER5_CAPT_vect) {
  /*This subroutine checks was it start of pulse (rising edge)
  or was it end (fallingedge)and performs required operations*/

  if (PINL & 0x02) { // if high level
    // save start time
    rising2 = ICR5;

    // set to trigger on falling edge
    TCCR5A = 0x00;
    TCCR5B = _BV(ICNC5) | _BV(CS51);

    // reset overflow counter
    OFcounts[1] = 0;

  } else {

    // save falling time
    falling2 = ICR5;

    // rising edge triggers next
    TCCR5B = _BV(ICNC5) | _BV(ICES5) | _BV(CS51);

    counts[1] = falling2 - rising2;
    OFcounts[1] = 0; /* got a pulse, clear overflow ctr */
  }
}

/**********************************************************************/

// Timer3: Joint 3 encoder timer on pin PE7 (ICP3)
//  results in counts[2] and OFcounts[2]
void ICP3_Init(void) {
  // PINE7 is input
  DDRE &= ~_BV(7);

  // initialize timer interrupt
  TIMSK3 = _BV(ICIE3) | _BV(TOIE3);
  TCCR3B = _BV(ICNC3) | _BV(ICES3) | _BV(CS31);
  OFcounts[0] = 0;
  counts[0] = 0;
  rising3 = 0;
  falling3 = 0;
}

// overflow counter interrupts service routine
SIGNAL(TIMER3_OVF_vect) { OFcounts[0]++; }

// Timer3 capture interrupt service subroutine
SIGNAL(TIMER3_CAPT_vect) {
  /*This subroutine checks was it start of pulse (rising edge)
  or was it end (fallingedge)and performs required operations*/

  if (PINE & _BV(7)) { // if high level
    // save start time
    rising3 = ICR3;

    TCCR3A = 0x00;
    // set to trigger on falling edge
    TCCR3B = _BV(ICNC3) | _BV(CS31);

    // reset overflow counter
    OFcounts[0] = 0;

  } else {

    falling3 = ICR3;

    // rising edge triggers next
    TCCR3B = _BV(ICNC3) | _BV(ICES3) | _BV(CS31);

    counts[0] = falling3 - rising3;
    OFcounts[0] = 0; /* got a pulse, clear overflow ctr */
  }
}
