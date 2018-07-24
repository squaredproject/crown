// -----------------------------------------------------------------------
// 
//	File: parser.c
//	parser file for WAVE
//      build code using WinAVR toolchain: see makefile
//
//	Written by Jonathan Foote (jtfoote at rotormind)
//      based on original code by Petey the Programmer
// -----------------------------------------------------------------------

#include <avr/io.h>
#include <ctype.h> 
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "AD7376.h"	
#include "parser.h"	
#include "putstr.h"	
#include "timer.h"
#include "UARTS.h"
#include "WAVE.h"
#include "jointPID.h"

extern uint8_t addr;

extern joint_control_block *jcb[3];

uint8_t debug_out = 1;			/* set this to output debug info */

uint8_t cmd0_str[MAX_COMMAND_LENGTH];
uint8_t cmd0_len = 0;

uint8_t cmd1_str[MAX_COMMAND_LENGTH];
uint8_t cmd1_len = 0;


/* Protocol: <AJONNNN> where: */
/* A is board address (single-char int, 0 is broadcast), */
/* J is joint number, 1, 2, 3 (for things that care, 0 otherwise) */
/* O is opcode, single char like "I" for PID I value* */
/* NNNN   integer uint16_t parameter as decimal chars. */
/* For floats and signed things we just agree on a divisor and/or offset. */
/* *A lot of opcodes will be there for debugging/set up but not actually used in practice.  */

/* NOTE: A and J are optional, assumed to be zero if not specified */
/* NOTE: NNN can be any length, no check for overflow, must fit into uint16_t! */

// --------------------------------------------------------------------
// parse the command string
// 

extern volatile int16_t counts[3];  // pulse width


void Print_Counts(void) {
  putstr("\r\n COUNTS: ");
  putS16(counts[0]);
  putstr(" ");
  putS16(counts[1]);
  putstr(" ");
  putS16(counts[2]);
}


void parseCommand(uint8_t ptr){
  int16_t intData=0;    /* holds numerical value of parsed data */
  uint8_t joint_num =0; /* if there's a joint specified, this is the num */
  uint8_t charPos=1;	/* start with first char past the "<" */
  uint8_t c;		   /* next char to parse */
  uint8_t val;		  /* temp val */

  uint8_t *cmd_str;
  uint8_t cmd_len;

  if (ptr == 1){
	cmd_str = cmd1_str;
	cmd_len = cmd1_len;
  }
  else {
	cmd_str = cmd0_str;
	cmd_len = cmd0_len;

  }

  /* if command does not start with L then they ain't talking to us */
  //if(cmd_str[charPos++] != 'L') {
    //if(debug_out) putstr("\r\n no L command");
  //  return;
  //}  

  /* is the next char an address digit? */
  c = cmd_str[charPos];
  if(isdigit(c)) { 
    /* if not our address then they ain't talking to us */
    /* should use parseInt to get multi-byte addr, but assume 0-9 for now */
    if ((c - '0') != addr) {
      if(debug_out) {
		putstr("\r  ");
		putchr(c);
		putstr(" != ");
		putU8(addr);
      }
	  cmd_len = 0;
      return; 			/* skip rest of command */
    }
    // it was a digit, so move ahead
	charPos++;
  } 

  /* next char may be a joint digit spec, if so get it (0 otherwise) */
  c = cmd_str[charPos];
  joint_num = 0;
  if(isdigit(c)) { 
	joint_num = c - '0';
	if(debug_out) {
	  putstr("\n\r  ");
	  putstr("\n\r joint: ");
	  putU8(joint_num);
	}
    // it was a digit, so move ahead
	charPos++;
  }
  
  /* OK, they are talking to us: get the rest of the command */
  /* if there's a number in the command, it follows the next (command) byte*/
  /* grab it now*/
  if(isdigit(cmd_str[charPos+1]))  // check for end of string 
    intData = parseInteger(cmd_str,charPos+1);
  else if(cmd_str[charPos+1] == '-')  // check for end of string 
    intData = parseInteger(cmd_str,charPos+1);
  else 
	intData = 0;
  
  /* this is the command char byte */
  c = cmd_str[charPos];

  switch (c) {

  case 'h': /* API  homing speed 0<h<63 */
	if (joint_num) { 			/* if we specified one */
	  if (debug_out) {
		putstr("\r\nh: ");
		intData &= 0x3F;
		putint(intData);
	  }
	  jcb[joint_num - 1]->homespeed = (uint8_t)intData;
	}
	break;

  case 't': /* API  joint target value */
	if (joint_num) { 			/* if we specified one */
	  jcb[joint_num - 1]->targetPos = intData;
	}
	break;

  case 'i': /* API  immediate raw valve output, 0 < val < 127 */
	if (joint_num) { 			/* if we specified one */
	  putstr("\r\ni: ");
	  putint(intData);
	  jcb[joint_num-1]->dirve = intData;
	  DP_SendValue((uint8_t) intData,joint_num -1);	  
	}
	break;

  case 'x': /* API dead band 0 < val < 127 */
	if (joint_num) { 			/* if we specified one */
	  putstr("\r\nx: ");
	  putint(intData);
	  jcb[joint_num - 1]->dead_band = intData;	}
	break;

  case 'S': /* API print joint status value */
	if(joint_num) {
	  Print_Joint_Status(jcb[joint_num - 1]);
	  if (intData & 0x01) Dump_JCB(jcb[joint_num - 1]);
	}
	break;

  case 'o': /* API Print pOsition counts */
	Print_Counts();
	break;

  case 'H': /* home this joint number */
	if(joint_num)
	  Home_Joint(jcb[joint_num - 1]);
	break;
	
  case 'L': /* API set drive limit */
	val = (uint8_t) intData & 0x7F; /* limit value to 0-63 */
	if(joint_num) {
	  jcb[joint_num - 1]->dmin = 63 - val;
	  jcb[joint_num - 1]->dmax = 63 + val;
	}
	break;

  case 'C': /* API set joint center value BEST DONE VIA HOME COMMAND*/
	if(joint_num)
	  jcb[joint_num - 1]->center = intData;
	break;

  case 'D': /* API direction flag, set to invert sense of PID */
	if(joint_num)
	  if (debug_out) {
		putstr("\r\nD: ");
		putint(intData);
	  }
	  jcb[joint_num - 1]->direction = (uint8_t) intData;
	break;


  case 'P': /* API  joint PID P value */
	if (joint_num) { 			/* if we specified one */
	  if (debug_out) {
		putstr("\r\nP: ");
		putint(intData);
	  }
	  jcb[joint_num - 1]->Kp = intData;
	}
	break;

  case 'I': /* API  joint PID I value */
	if (joint_num) { 			/* if we specified one */
	  if (debug_out) {
		putstr("\r\nP: ");
		putint(intData);
	  }
	  jcb[joint_num - 1]->Ki = intData;
	}
	break;

  case 'T': /* API  joint PID trace value */
	if (joint_num) { 			/* if we specified one */
	  putstr("\r\nT: ");
	  putint(intData);
	  jcb[joint_num - 1]->trace = intData;
	}
	break;
	
  case 'm': /* Set joint minimum position */
  	if(joint_num)
	  jcb[joint_num - 1]->minpos = intData;
	break;
	
  case 'M': /* Set joint maximum position */
  	if(joint_num)
	  jcb[joint_num - 1]->maxpos = intData;
	break;
	
  case 'A': /* print all joint information, machine-friendly format */
    for (int i = 0; i < 3; i++) {
      char buf[128];
      sprintf(buf, "\r\n<%d%d: %d, %d, %d>", addr,j, jcb[i]->minpos, jcb[i]->maxpos, jcb[i]->currentpos);
      putstr(buf);
    }  
    break; 
  
  case 'X': /* dump everything we can */
    for (int i = 0; i < 3; i++) {
      char buf[128];
      sprintf(buf, "\r\n***Joint Control Block %d*** \n", i);
      putstr(buf);
      Dump_JCB(jcb[i]);
    }
    Dump_Status();
	break;


  default: 			/* poorly formed command string, ignore */
    if(debug_out){
      putstr("\r\n?:\n ");
      putchr(c);
    }
    break;
  }
  /* restart parser after complete command */
  //cmd_len = 0;
}


// parse the command string  at the given position
// convert Ascii signed number to short word. (16-Bit)

int16_t parseInteger(uint8_t *cmd_str, uint8_t startChr)
{
  int32_t accum = 0; // accumulate the integer data
  uint8_t sign = 0;
  uint8_t cPos = startChr;
  
  /* detect negative values */
  if (cmd_str[startChr] == '-') {
	sign = 1;
	cPos++;
  }
  
  /* while we see digits, convert them to integer */
  /* should probably check for integer overflow   */
  /* (string end is handled by isdigit('\0')=0) */
  while (isdigit(cmd_str[cPos])) {
    //UART1_send_byte(cmd_str[cPos]);
    accum = (accum * 10) + (cmd_str[cPos++] - '0');
  } 
  
  if (sign)
    return((int16_t)-accum);
  /* else */
  return (int16_t)accum;
}

// --------------------------------------------------------------------------
// process incoming chars - commands start with '<' and end with '>'
// return 1 if command string is complete - else return zero
 
uint8_t accumulateCommandString0(uint8_t c){
  /* catch beginning of this string */
  if (c == '<') { // this will catch re-starts and stalls as well as valid commands.
     cmd0_len = 0;
     cmd0_str[cmd0_len++] = c;
     return 0;
  }
  
  if (cmd0_len != 0){	 // string in progress, accumulate next char
    
    if (cmd0_len < MAX_COMMAND_LENGTH) 
      cmd0_str[cmd0_len++] = c;
    if (c == '>') {
	char buf[128];
	memcpy(buf, cmd0_str, cmd0_len);
	buf[cmd0_len] = '\0';
	putstr(buf);
	return 1;
    }
  }
  return 0;
}

uint8_t accumulateCommandString1(uint8_t c){
  /* catch beginning of this string */
  if (c == '<') { // this will catch re-starts and stalls as well as valid commands.
     cmd1_len = 0;
     cmd1_str[cmd1_len++] = c;
     return 0;
  }
  
  if (cmd1_len != 0){	 // string in progress, accumulate next char
    
    if (cmd1_len < MAX_COMMAND_LENGTH) 
      cmd1_str[cmd1_len++] = c;
    
    if (c == '>') {
	char buf[128];
	memcpy(buf, cmd1_str, cmd1_len);
	buf[cmd1_len] = '\0';
	putstr(buf);
	return 1;
    }
  }
  return 0;
}
 
