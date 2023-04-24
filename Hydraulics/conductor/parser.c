// -----------------------------------------------------------------------
//
//	File: parser.c
//	parser file for WAVE
//      build code using WinAVR toolchain: see makefile
//
//	Written by Jonathan Foote (jtfoote at rotormind)
//      based on original code by Petey the Programmer
// -----------------------------------------------------------------------

// from comm.c
#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "UART0.h"
#include "conductor.h"
#include "parser.h"
#include "sinusoid.h"
#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <ctype.h>

uint8_t debug_out = 1; /* set this to output debug info */

uint8_t cmd_str[MAX_COMMAND_LENGTH];
uint8_t cmd_len = 0;

/* Protocol: <AJONNNN> where: */
/* A is board address (single-char int, 0 is broadcast), */
/* J is joint number, 1, 2, 3 (for things that care, 0 otherwise) */
/* O is opcode, single char like "I" for PID I value* */
/* NNNN   integer uint16_t parameter as decimal chars. */
/* For floats and signed things we just agree on a divisor and/or offset. */
/* *A lot of opcodes will be there for debugging/set up but not actually used in
 * practice.  */

/* NOTE: A and J are optional, assumed to be zero if not specified */
/* NOTE: NNN can be any length, no check for overflow, must fit into uint16_t!
 */

// --------------------------------------------------------------------
// parse the command string
//

void parseCommand() {
  uint16_t intData = 0; /* holds numerical value of parsed data */
  uint8_t charPos = 1;  /* start with first char past the "<" */
  uint8_t c;            /* next char to parse */

  /* is the next char an address digit? */
  c = cmd_str[charPos];

  /* if not our address then they ain't talking to us */
  /* should use parseInt to get multi-byte addr, but assume 0-9 for now */
  if (c == 'x') {
    charPos++;
  } else {
    cmd_len = 0;
    return; /* skip rest of command */
  }

  /* next char may be a joint digit spec, if so get it (0 otherwise) */
  c = cmd_str[charPos];
  if (c == 'x') {
    charPos++;
  } else {
    cmd_len = 0;
    return; /* skip rest of command */
  }

  /* OK, they are talking to us: get the rest of the command */
  /* if there's a number in the command, it follows the next (command) byte*/
  /* grab it now*/
  if (isdigit(cmd_str[charPos + 1])) // check for end of string
    intData = parseInteger(charPos + 1);
  else
    intData = 0;

  /* this is the command char byte */
  c = cmd_str[charPos];

  switch (c) {

  case 'p': /* stream position mode */
    sine_debug_mode = SINE_DEBUG_POSITIONS;
    break;

  case 't': /* API stream theta mode */
    sine_debug_mode = SINE_DEBUG_THETA;
    break;

  case 'x': /* API  stop streaming */
    sine_debug_mode = SINE_DEBUG_NONE;
    break;

  default: /* poorly formed command string, ignore */
    if (debug_out) {
      printf("\r\nP: %d", intData);
    }
    break;
  }
  /* restart parser after complete command */
  cmd_len = 0;
  return;
}

// parse the command string  at the given position
// convert Ascii signed number to short word. (16-Bit)

uint16_t parseInteger(uint8_t startChr) {
  uint16_t accum = 0; // accumulate the integer data
  // uint8_t sign = 0;
  uint8_t cPos = startChr;

  /* if you want negative values (better change var to signed too) */
  // if (cmd_str[startChr] == '-') {
  //   sign = 1;
  //   cPos++;
  // }

  /* while we see digits, convert them to integer */
  /* should probably check for integer overflow   */
  /* (string end is handled by isdigit('\0')=0) */
  while (isdigit(cmd_str[cPos])) {
    // UART0_send_byte(cmd_str[cPos]);
    accum = (accum * 10) + (cmd_str[cPos++] - '0');
  }

  // if (sign)
  //   accum = -accum;

  return accum;
}

// --------------------------------------------------------------------------
// process incoming chars - commands start with '<' and end with '>'
// return 1 if command string is complete - else return zero

uint8_t accumulateCommandString(uint8_t c) {
  /* catch beginning of this string */
  if (c ==
      '<') { // this will catch re-starts and stalls as well as valid commands.
    cmd_len = 0;
    cmd_str[cmd_len++] = c;
    return 0;
  }

  if (cmd_len != 0) { // string in progress, accumulate next char

    if (cmd_len < MAX_COMMAND_LENGTH)
      cmd_str[cmd_len++] = c;
    return (c == '>');
  }
  return 0;
}
