#include <avr/io.h>


#include "WAVE.h"  // for define of F_CPU
#include "AD7376.h"
#include <util/delay.h>


// Defines for AD7376 Digital Pot port access
#define DP_PORT PORTA

#define RS_PIN 0  // Reset pin active low. High for normal operation!

#define CLKPIN  3 // A pin 25 -- clock input: chip pin  7
#define SDIPIN  5 // A pin 27 -- data input:  chip pin 11

#define CS1_PIN 1 // A pin 23 -- latch pin:   chip pin 5 
#define CS2_PIN 2  // chipselect 2
#define CS3_PIN 4 // chipselect 3

//SHDN_: shutdown pin 12 tie high


// assert reset; clear with DP_enable()
void DP_reset(void) {
  DDRA = 0xFF; // Port A is output
  /* reset pot so valve is off*/
  DP_PORT = _BV(CS1_PIN) | _BV(CS2_PIN) | _BV(CS3_PIN);
  DP_PORT &= ~ _BV(RS_PIN);
}

// Set pins to outputs and initial states
void DP_enable(void) {
  DDRA = 0xFF; // Port A is output
  /* set all chip enables high, and reset high too */
  DP_PORT = _BV(CS1_PIN) | _BV(CS2_PIN) | _BV(CS3_PIN) |  _BV(RS_PIN);
}


void DP_SendValue(uint8_t val, uint8_t chan) {
  /* val is 0-127 control for digital pot, chan is 0,1,2 for chan 1, 2, 3 */
  /* CSW - 2018 - I've reversed the mapping of channels to joints. CS1 now maps
  to joint 2, and CS3 to joint 0. Arguably, we should leave this function
  alone and add a joint-to-channel mapping in the higher level code, but <eh> */
  
  uint8_t j;

  switch(chan){
  case 0:
	DP_PORT &= ~ _BV(CS3_PIN);  // enable shift in
	break;
  case 1:
	DP_PORT &= ~ _BV(CS2_PIN);  // enable shift in
	break;
  case 2:
	DP_PORT &= ~ _BV(CS1_PIN);  // enable shift in
	break;
  }

  _delay_us(1);
  for (j = 0; j < 7; j++) {
    if ((val >> (6 - j)) & 1) { // shift out MSB first
      DP_PORT |= (1 << SDIPIN);
    } 
    else {
      DP_PORT &= ~(1 << SDIPIN);
    }
	_delay_us(1);
    DP_PORT |= (1 << CLKPIN); // rising edge of clock
	_delay_us(1);
    DP_PORT &= ~(1 << CLKPIN); 
  } 
  _delay_us(1);

  /* take all chip select pins high */
  DP_PORT = _BV(CS1_PIN) | _BV(CS2_PIN) | _BV(CS3_PIN) |  _BV(RS_PIN);
}
