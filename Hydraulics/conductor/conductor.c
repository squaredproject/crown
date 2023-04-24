// -----------------------------------------------------------------------
//
//	File: conductor.c
//	main file for the conductor in WAVE
//
//	Written by Michael Prados, April 1, 2012
//      adapted from code from Jon Foote
// -----------------------------------------------------------------------

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h> // needs F_CPU def

#include "UART0.h"
#include "UART1.h"
#include "UART2.h"
#include "UARTbaudrates.h" // needs F_CPU def
#include "a2d.h"
#include "conductor.h"
#include "lcd.h"
#include "modes.h"
#include "parser.h"
#include "sinusoid.h"
#include "timer.h"

#define FW_VERSION "Conductor v 0.53"

// PORT ASSIGNMENTS:

// PORTA: 	(digital 22-29)  		not used
// PORTB: 	(digital 10-13, 50-53)	PB0-PB3 = mode switch, PB7 = LED on
// digital 13 PORTC: 	(digital 30-37)			not used PORTD: (digital
// 18-21, 38)		PD2 = RXD1, PD3 = TXD1
// PORTE: 	(digital 1,2, 3, 5) 	PE0 = TXD0, PE1 = RXD0, PE4 = RS485 TX
// enable PORTF: 	(adc0-adc7)  			not used PORTG:
// (digital 4)				not used PORTH:  	(digital 9)
// PH0 = RXD2, PH1 = TXD2
// (no PORTI)						does not exist!
// PORTJ:	(digital 14,15)			future implementation of UART3: PJ0
// = RXD3, PJ1 = TXD3
// PORTK: 	(adc8-adc15) 			analog inputs for knob controls
// PORTL: 	(digital 42-49)			PL0-PL3 = mode switch

// note - Arduino digital pins are numbered opposite the port bits...

// =======================================================

#define HEARTBEAT_LED 7 /* port B bit 7, Arduino digital 13*/
#define HEARTBEAT_PORT PORTB

/* states for the state machine */
enum { NORMAL, ERROR };
uint8_t state = NORMAL; /* initial state  */

// rotary switch to set user controlled mode
uint8_t mode_switch(void);

int main(void) {

  stdout = &uart0file; // UART0 entirely devoted to status and debug

  uint8_t cData; // byte to read from UART */
  uint8_t n;

  uint16_t heartcount = 0; // timer for heartbeat led
  uint8_t toggle = 0;

  uint8_t ten_Hz_counter = 0;

  int32_t knob[8];

  // set up ports
  DDRD = 0xF0;  // 0b 1111 0000	   1 indicates Output pin
  PORTD = 0xFF; // turn on pullups

  DDRC = 0x00;  // inputs
  PORTC = 0xFF; // pullups on

  DDRF = 0x00;  // inputs
  PORTF = 0xFF; // pullups on

  DDRB = 0x00 | _BV(HEARTBEAT_LED); // LED on PB7
  PORTB = 0xFF;

  DDRL = 0x00;  // inputs
  PORTL = 0xFF; // pullups on

  DDRE = 0x1F;  //
  PORTE = 0x10; // PE4 for continuous RS485 tx

  // set up UARTS
  UART0_Init(UART_38400);  // Debug Baud Rate (to computer)
  UART1_Init(UART_115200); // Network Baud out to nodes
  //  UART1_Init(UART_76800);	// Network Baud out to nodes
  UART2_Init(UART_19200); // LCD Baud Rate

  Timer0_Init(); /* Init Timer 0 for tick interrupt */
  A2D_Init();    /* int ADC8-15 for analog input */

  initialize_sinusoids();

  // start all the interrupts running
  sei();

  // if you really need to re-set the LCD baud rate.
  // lcd_set_baud();
  //_delay_ms 	(200);

  // send some configuration bytes to the LCD
  // needs file streaming, so interrupts etc must already be set up.
  lcd_init();

  // display version for 2 seconds at startup
  fprintf(LCDFILE, FW_VERSION);
  _delay_ms(2000);

  // start main loop =======================================================
  while (1) {

    A2D_poll_adc(); // check if adc conversion is done

    // Main parser loop starts here. To save space, not modularized
    if (UART0_data_in_ring_buf()) { // check for waiting UART data

      cData = UART0_ring_buf_byte(); // get next char from ring buffer...

      if (accumulateCommandString(cData)) { // ... and add to command string
        // if we are here we have a complete command; parse it
        parseCommand(); // parse and execute commands
      }
    }

    // 1 kHz
    if (Timer0_flag_is_set()) {

      // 10 Hz control loop
      if (ten_Hz_counter >= 50) {
        for (n = 0; n < 8; n++)
          knob[n] = (int32_t)ADC_read_low_pass(n);
        uint8_t this_mode = mode_switch();

        apply_ui_to_sinusoids(knob, this_mode);
        pace_sinusoids();
        generate_positions();
        stream_positions();

        stream_debug();

        lcd_mode(this_mode, knob);

        ten_Hz_counter = 0;
      } else // end 10 Hz loop
      {
        ten_Hz_counter++;
        mode_switch();
      }

      heartcount++;

      /* blink heartbeat at 1s pattern rate */
      if (heartcount >= 500) {
        heartcount = 0;
        toggle = 1 - toggle;
        /* once per second */
        if (toggle) {
          HEARTBEAT_PORT |= _BV(HEARTBEAT_LED);
        } else {
          HEARTBEAT_PORT &= ~_BV(HEARTBEAT_LED);
        }
      } // end if heartcount

    } /* end if 1 khz */
  }   /* end while(1) */
} // end main

/* function mode_switch
 * reads the rotary switch on Arduino Mega 2560 Pins 46-53
 * The switch grounds one pin in each position. Pullups on all inputs
 * Applies 100ms debounce period
 */
uint8_t mode_switch(void) {
  uint8_t mode_byte = 0;
  uint8_t read_bit = 0;
  static uint8_t current_bit = 0;
  static uint8_t last_bit = 0;
  static uint8_t counter = 0;

  // Arduino Pins 46-53 are on PORTB and PORTL.
  mode_byte = (0x0f & ~PINB) | ((0x0f & ~PINL) << 4);

  read_bit = 8;

  while (mode_byte != 0) {
    if (mode_byte && 0x80)
      read_bit--;

    mode_byte <<= 1;
  }
  // If you don't find a grounded pin, keep the last value
  if (read_bit > 7)
    read_bit = current_bit;

  if ((read_bit == last_bit) && (read_bit != current_bit))
    counter++;
  else
    counter = 0;

  if (counter > 100) {
    current_bit = read_bit;
    counter = 0;
  }

  last_bit = read_bit;

  return (current_bit);
}
