// ---------------------------------------------------------------------
//
//	File: jointPID.c
//      WAVE
//      joint control PID tasks
//
//	Refactored by Jonathan (Head Rotor at rotormind.com)
// -----------------------------------------------------------------------

#include <avr/io.h>
#include <stdlib.h>
#include "AD7376.h"
#include "UARTS.h"
#include "putstr.h"
#include "jointPID.h"
#include "WAVE.h"
#include "CAN_message.h"

// needs F_CPU def from WAVE.h
#include <util/delay.h>

extern uint8_t addr; 		/* board address */

extern volatile int16_t counts[3];  // encoder pulse widths

/* Static Vars */
 joint_control_block joint1;
 joint_control_block joint2;
 joint_control_block joint3;

/* use this to access joint control blocks */
volatile joint_control_block *jcb[3] = {&joint1, &joint2, &joint3};

static void finishHomingSetup(joint_control_block *j, uint16_t minpos_raw, uint16_t maxpos_raw, uint16_t centerPos);

void PID_Init(void)
{
  Joint_Init_JCB((joint_control_block *) jcb[0],0);
  Joint_Init_JCB((joint_control_block *) jcb[1],1);
  Joint_Init_JCB((joint_control_block *) jcb[2],2);
  
  /* start up with valves off */
  DP_SendValue(63,0);
  DP_SendValue(63,1);
  DP_SendValue(63,2);

}

/* Read the limit switches and place in the corresponding JCB */


#define JOINT0MASK  (0x07) // Mask for Joint 1 limit switches (0,1,2)
#define JOINT0PIN (PINC) // input port for joint 1 limit switches

#define JOINT1MASK  (0x38) // Mask for Joint 2 limit switches (1, 2, 3)
#define JOINT1PIN  (PINC) // input port for joint 2 limit switches

#define JOINT2MASK  (0x07) // Mask for Joint 2 limit switches (1, 2, 3)
#define JOINT2PIN  (PINF) // input port for joint 2 limit switches

// POSITIVE DRIVE IS TO THE LEFT
#define HOME_SPEED 63

#define MAX_LIMITS_IN_HOME 4
#define MAX_STALLS_IN_HOME 500
#define STALL_MARGIN 5

void Poll_Limit_Switches(void){
  /*  */
  uint8_t val; 
  
  /* read limit switches for joint 1 */
  val = PINC;
  jcb[2]->switches =  ~val & JOINT0MASK;
  jcb[1]->switches = (~val & JOINT1MASK) >> 3;
  val = PINF;
  jcb[0]->switches =  ~val & JOINT2MASK;
  //putstr("\r\n PC:");
  //putB8(val);
  //val = PINF;
  //putstr(" PF:");
  //putB8(val);

}


/* check for multple limit switches (error!) */
uint8_t Test_MLimit_Switches(joint_control_block *j) {
  /*  */
  uint8_t val = j->switches; 
  
  /* read limit switches for joint 1 */

  /* if more than one limit switch is set we have a problem */
  switch(val){
  case 0x03: 					/* 011 */
  case 0x05:					/* 101 */
  case 0x06:					/* 110 */
  case 0x07:					/* 111 */
	pause_error(SWITCH_ERROR);
  }
  return(0);
}

void Home_Joint(joint_control_block *j) {
  /* home this joint */
  uint8_t speed; 			/* creep speed and direction */
  uint8_t limit_mask;			/* mask of limit switches */
  j->homed = 0; 				/* not homed yet */
  Poll_Limit_Switches(); 	/* get limit sw status */
  uint16_t maxpos_raw;
  uint16_t minpos_raw;

  /* FIRST: turn off valves, wait for things to stop */
  putstr("\r\nHOME J");
  puthex(j->id);
  putstr(" ");
  set_LED_out(64);
  _delay_us(100);


  /* first test if we are at a limit */
  if (j->switches & LEFT_SW){  /* If we're already at the left limit, move right*/
    putstr("\r\nLEFT_SW, homing right");
    minpos_raw = counts[j->id];
    if (!j->direction)
      speed = HOME_SPEED + j->homespeed;  /* creep to the right */
    else
      speed = HOME_SPEED - j->homespeed;
    limit_mask  = RIGHT_SW;               /* ignore (pressed) left sw */
  }
  else if (j->switches & RIGHT_SW){   /* If we're already at the right limit, move left */
    /* default creep to the left */
    if (!j->direction) {
      speed = HOME_SPEED - j->homespeed;
    } else {
      speed = HOME_SPEED + j->homespeed;
    }
    putstr("\r\nRIGHT_SW, homing left");
    maxpos_raw = counts[j->id];
    limit_mask  = LEFT_SW; 			/* ignore (pressed) right sw */
  }
  else {  /* Not at any limit, move right by default */
    speed = HOME_SPEED - j->homespeed;
    putstr("\r\nNo SW: default right");
    limit_mask  = LEFT_SW | RIGHT_SW; /* change direction on hitting any limit sw */
  }

  /* look for home... We want to hit both of the limit switches and *then* the home switch... 
     Note that we're also doing a little sanity checking - if we can't find home after a few back and forths, or
     the sculpture isn't responding to our commands, break out of the loop and set an error */
  int numLimits = 0;
  int numStalls = 0;
  int lastLimit = 0;
  uint16_t lastPos = counts[j->id];
  while((limit_mask || !(j->switches & HOME_SW)) &&
      (numLimits <= MAX_LIMITS_IN_HOME) && (numStalls <= MAX_STALLS_IN_HOME)){

    if (ESTOP_PORT & _BV(ESTOP_PIN)) {
      CAN_SendHomingResult(j->id+1, 0, HOMING_RESULT_ESTOP, limit_mask);
      pause_error(ESTOP_ERROR);
      return;
    }

    // manual override to allow us to break out of the home loop
    uint8_t button_state = get_Fbuttons(0);
    if (button_state &  SPF3_SW ){ 
      CAN_SendHomingResult(j->id+1, 0, HOMING_RESULT_CLEAR, limit_mask);
      putstr("\r\nCLEAR called, break out of Home loop!");
      return;
    }

    /* not homed yet, move in the creep direction until we hit home sw */
    Poll_Limit_Switches(); 	/* get limit sw status */

    /* check if we've hit opposite switch, meaning we change direction */
    if (j->switches /*& limit_mask*/) {
      putstr("\r\nLimit found, may reverse");
      if (j->switches & LEFT_SW && (lastLimit != LEFT_SW)){
        /* found min, go right */
        putstr("\r\nLEFT_SW, homing right, limit mask ");
        minpos_raw = counts[j->id];
        if (!j->direction) {
          speed = HOME_SPEED + j->homespeed;  /* creep to the right */
        } else {
          speed = HOME_SPEED - j->homespeed;
        }
        limit_mask  &= ~LEFT_SW;      /* we've found the left switch - remove from objective */
        numLimits++;
        lastLimit = LEFT_SW;
        puthex(limit_mask);
      }
      else if (j->switches & RIGHT_SW && (lastLimit != RIGHT_SW)){
        /* found max, go to left */
        if (!j->direction) {
          speed = HOME_SPEED - j->homespeed;
        } else {
          speed = HOME_SPEED + j->homespeed;
        }
        maxpos_raw = counts[j->id];
        putstr("\r\nRIGHT_SW, homing left, limit mask ");
        limit_mask  &= ~RIGHT_SW;    /* we've found the right switch - remove from objective */
        numLimits++;
        lastLimit = RIGHT_SW;
        puthex(limit_mask);
      }
    }

    //if (j->trace){
    if (1){
      putstr("\r\nHoming J");
      puthex(j->id);
      putstr(" speed: ");
      putU8(speed);
      putstr(" ");
      putstr(speed >= HOME_SPEED ? "(RIGHT)" : "(LEFT)");
      putstr(" raw pos: ");
      //putint(counts[j->id -1]);
      putint(counts[j->id]); /* test joint hard wire to enc 3 */
      putstr(" sw: ");
      putB8(j->switches);
      putstr(" mask: ");
      puthex(limit_mask);
      putstr(" stalls: ");
      putint(numStalls);

      CAN_SendHomingStatus(j->id+1, counts[j->id], j->switches, limit_mask, numStalls);
    }	

    /* set valve command to speed*/
    set_LED_out(speed); 
    DP_SendValue(speed,j->id);
    _delay_ms(50);

    /* check for stalls */
    int16_t diff = lastPos - counts[j->id];
    if (diff < 0) diff = -diff;
    if (diff >= STALL_MARGIN) {
       lastPos = counts[j->id];
       numStalls = 0;
    } else {
       numStalls++;
       // because our limit switches suck...
       // If we're stuck going left and the left limit hasn't been met, go right
       // and arbitrarily set the left limit at 100 off the current position.
       if ((speed > HOME_SPEED) && (limit_mask & RIGHT_SW)) { // going right, and looking for right switch
            speed = HOME_SPEED - j->homespeed;
            limit_mask  &= ~RIGHT_SW;
            maxpos_raw = counts[j->id] - 100;
            numStalls = 0;
            lastLimit = RIGHT_SW;
       } else if ((speed < HOME_SPEED) && (limit_mask & LEFT_SW)){ // going left, looking for left switch
            speed = HOME_SPEED + j->homespeed;
            limit_mask  &= ~LEFT_SW;
            minpos_raw = counts[j->id] + 100;
            numStalls = 0;
            lastLimit = LEFT_SW;
       }
    }
  }

  if (numStalls > MAX_STALLS_IN_HOME) {
    /*  Hydraulics stalled out - could not get to limits! */
    putstr("\r\n HOME failed. Stall error");
    CAN_SendHomingResult(j->id+1, 0, HOMING_RESULT_STALL, limit_mask);
    pause_error(HOME_ERROR);
  } else if (numLimits > MAX_LIMITS_IN_HOME) {
    /* went back and forth a few times without triggering the center switch. Provisional pass */
    putstr("\r\n HOME failed. Center limit does not trigger. Will set to median of min/max");
    CAN_SendHomingResult(j->id+1, 1, HOMING_RESULT_CENTER_NOT_FOUND, limit_mask);
    finishHomingSetup(j, minpos_raw, maxpos_raw, (minpos_raw + maxpos_raw)/2);
  } else {
    /* if we get here, we are homed */
    CAN_SendHomingResult(j->id+1, 1, HOMING_RESULT_OK, 0);
    finishHomingSetup(j, minpos_raw, maxpos_raw, counts[j->id]);
  }
}

static void finishHomingSetup(joint_control_block *j, uint16_t minpos_raw, uint16_t maxpos_raw, uint16_t centerPos)
{
    int allFinished = TRUE;
    
     // make min and max pos relative
    j->minpos = minpos_raw - centerPos;
    j->maxpos = maxpos_raw - centerPos;
    j->center = centerPos;
 
    // turn off valves; set target position to where ever we are now
    set_LED_out(HOME_SPEED);
    DP_SendValue(HOME_SPEED,j->id);
    j->homed = TRUE;
    j->targetPos = centerPos - counts[j->id];  
    
    putstr("\r\nHomed at ");
    putint(j->center);
    putstr(" min/max:");
    putS16(j->minpos);
    putchr(' ');
    putS16(j->maxpos);
    
    for (int i=0; i<3; i++) {
        allFinished &= jcb[i]->homed;
    }
    
    if (allFinished) {
        setRunning(TRUE);
    }
}

// make sure minVal < v < maxVal (!)
int32_t clamp32(int32_t *v, int32_t minVal, int32_t maxVal)
{
	if (*v < minVal) *v = minVal;
	if (*v > maxVal) *v = maxVal;
	return *v;
}
int16_t clamp16(int16_t *v, int16_t minVal, int16_t maxVal)
{
	if (*v < minVal) *v = minVal;
	if (*v > maxVal) *v = maxVal;
	return *v;
}
uint8_t clampU8(uint8_t *v, uint8_t minVal, uint8_t maxVal)
{
	if (*v < minVal) *v = minVal;
	if (*v > maxVal) *v = maxVal;
	return *v;
}



// * initialize joint parameters
void Joint_Init_JCB(joint_control_block *j,uint8_t id)
{

  j->id = id;			/* joint number (for debugging, etc) STARTS at 0! */
  j->Kp = 25;			/* proportional PID factor (scaled by 4)*/
  j->Ki = 0;			/* integral PID factor (scaled by 32)*/
  //j->Kd = 0;			/* derivative PID term */
  j->targetPos= 0;		/* where we want to go */
  j->currentPos=0;		/* where we are now */
  j->drive=63;	        /* the current valve setting */
  j->homespeed=20;	        /* valve drive to home */
  j->dmin=0;	        /* limit drive to this min value */
  j->dmax=127;	        /* limit drive to this max value */
  j->dead_band=10;	    /* close enough to desired position if within */
  //j->lastError= 0;	    /* the error we had last iteration */
  j->integrator= 0;	    /* sum of errors */
  //  j->lastVal= 0;	    /* the power we gave it last iteration */
  j->intLimit = 500;    /* integrator limit */
  j->minpos =  0;		/* minimum encoder value at limit, may not be set */
  j->maxpos =  0;		/* maximum encoder value at limit may not be set  */
  j->trace = 0;			/* default no trace */
  j->center = 4200;		/* roughly halfway */
  j->direction = 0;		/* invert sense of drive*/ 
  j->homed = 0;		    /* not homed yet */
  j->sw_enabled = 1;       /* by default, software enabled */

  // XXX - NB - the following max and min positions are derived from manually homing 
  // the particular test rig that we have in the shop.
  // In the final version, these values should be found via the standard homing process

  switch(addr){
  case 0:
    putstr(" addr 0 ctrs\r\n");
    switch(j->id){
    case 0:
      j->center =  4715;
      break;
    case 1:
      j->center =  4114;
      break;
    case 2:
      j->center =  4355;
      break;
    }
    break; /* end addr 0 */
  case 1:
    putstr(" addr 1 ctrs\r\n");
    switch(j->id){
    case 0:
      j->center =  4049;
      break;
    case 1:
      j->center =  4065;
      break;
    case 2:
      j->center =  4179;
      break;
    }
    break; /* end addr 0 */
  case 2:
    putstr(" addr 2 ctrs\r\n");
    switch(j->id){
    case 0:
      j->center =  4302;
      break;
    case 1:
      j->center =  4356;
      break;
    case 2:
      j->center =  3910;
      break;
    }
    break; /* end addr 0 */
  case 3:
    putstr(" addr 3 ctrs\r\n");
    switch(j->id){
    case 0:
      j->center =  4384;
      break;
    case 1:
      j->center = 4588;
      break;
    case 2:
      j->center =  4677;
      break;
    }
    break; /* end addr 0 */
  }
  
  switch(j->id){
  case 0:
    j->Kp = 100; /* proportional PID factor (scaled by 4)*/
    j->maxpos = 7000 - j->center;
    j->minpos = 400 - j->center; 
    break;
  case 1:
    j->Kp = 100; /* proportional PID factor (scaled by 4)*/
    j->maxpos = 4990 - j->center;
    j->minpos = 4201 - j->center;
    break;
  case 2:
    j->Kp = 100; /* proportional PID factor (scaled by 4)*/
    j->maxpos = 4822 - j->center;
    j->minpos = 3698 - j->center;
    break;
  }
}

#if 0
  switch(j->id){
  case 0:
    j->Kp = 100; /* proportional PID factor (scaled by 4)*/
    j->maxpos = 4416 - j->center;
    j->minpos = 3707 - j->center; 
    break;
  case 1:
    j->Kp = 100; /* proportional PID factor (scaled by 4)*/
    j->maxpos = 4990 - j->center;
    j->minpos = 4201 - j->center;
    break;
  case 2:
    j->Kp = 100; /* proportional PID factor (scaled by 4)*/
    j->maxpos = 4822 - j->center;
    j->minpos = 3698 - j->center;
    break;
  }
}
#endif // 0

// return motor status
void Print_Joint_Status(joint_control_block *j )
{
  putstr("\r\nJCB: ");
  puthex(j->id); 
  putstr(" pos:");
  putS16(j->currentPos);
  putstr(" min/max:");
  putS16(j->minpos);
  putchr(' ');
  putS16(j->maxpos);
}

//#define DEBUG 1

/* calculate new drive parameter from current position */
uint8_t Joint_Servo_Task(joint_control_block *j){

  int32_t error;
  int32_t drive;
  int32_t p_term, i_term;

  j->currentPos = counts[j->id] - j->center;
  error = j->currentPos - j->targetPos;

  if (j->trace & 0x02) 
    Dump_JCB(j);

  if (j->trace & 0x01) 
    Print_all_status(j);


  // close enough to  position
  if(abs(error) <= j->dead_band){
	// zero out integrator here? 
    if (j->trace) {
      putstr("\n\rIn pos J");
	  puthex(j->id);
	}
	/* we are in position so no drive needed */
    j->drive = 63;
    return((uint8_t)j->drive);
  }



  //if here, error is significant; change drive to decrease it
  // calculate proportional term
  p_term = error * j->Kp;
  // sum to integrate error for I term
  j->integrator += error;

  //  and limit runaway
  clamp16(&(j->integrator),-(j->intLimit),j->intLimit);

  i_term = j->Ki * j->integrator;
  i_term = i_term /8; //  (divide by 4) to scale

  drive = p_term + i_term;
  drive = 63 - (drive/256); // scale and offset

  if (j->direction) {
    drive = 127 - drive;
  }
  j->drive = clamp32(&drive,j->dmin,j->dmax); // limit to between dmin, dmax
  
  
  /* XXX - doing this in main block... 
  // Sanity check. If we're at or beyond the specified limits of motion, do *not* drive any further
  // in that direction, regardless of what anyone says.
  if ((j->currentPos < j->minpos) && (j->drive > 63)) {
    j->drive = 63;
    if (j->trace ) {
      putstr("\r\nClamping motion. Attempting to drive past min pos ");
      putS16(j->currentPos);
      putS16(j->minpos);
    }
  } else if ((j->currentPos > j->maxpos) && (j->drive < 63)) {
    j->drive = 63;
    if (j->trace) {
      putstr("\r\nClamping motion. Attempting to drive pas max pos ");
    }
  }
  */

  if (j->trace & 0x04) {
    putstr(" E");
    putint(error);
    putstr(" P");
    putint(p_term);
    putstr(" I");
    putint(i_term);
  }

  if (j->trace) {
    putstr("\r\nCalculated drive is ");
    putint(j->drive);
  }
  return((uint8_t)j->drive);
}

/* print out all variables from a joint control block for debug */
void Print_all_status(joint_control_block *j){
  putstr("\r\nJ");
  puthex(j->id);
  putstr(" raw:");
  putS16((int16_t)counts[j->id]);
  putstr(" ctr:");
  putS16(j->center);
  putstr(" PID targ:");
  putS16(j->targetPos);
  putstr(" curr:");
  putS16(j->currentPos);
  putstr(" drive:");
  putU8(j->drive);
  //    putstr(" int ");
  //  putS16(j->integrator);
  putstr(" sw:");
  puthex(j->switches);
  if (j->direction) {
	putstr(" INV");
  }
}


/* print out all variables from a joint control block for debug */
void Dump_JCB(joint_control_block *j){
  putstr("\r\nJ");
  puthex(j->id);
  putstr(" ENC raw, center ");
  putS16((int16_t)counts[j->id]);
  putstr(", ");
  putS16(j->center);
  putstr("\r\nPID targ curr ");
  putS16(j->targetPos);
  putstr(" ");
  putS16(j->currentPos);
  putstr(" drive:");
  putU8(j->drive);
  putstr(" dmin:");
  putU8(j->dmin);
  putstr(" dmax:");
  putU8(j->dmax);
  putstr(" minp:");
  putS16(j->minpos);
  putstr(" maxp:");
  putS16(j->maxpos);
  putstr(" dead:");
  putU8(j->dead_band);
  //putstr(" int:");
  //putS16(j->integrator);
  putstr(" sw:");
  puthex(j->switches);
  if (j->direction) {
	putstr(" INV");
  }
}

/* Attempt to stop the joint without stopping the system. Set target to current 
   position, don't push on the valve, and kill the integration parameter */
void Neuter_Joint(joint_control_block *j){
    j->targetPos = counts[counts[j->id]] - j->center;  // set target to current position
    j->drive = HOME_SPEED;
    j->integrator = 0;
}


// TODOs -
// Find a way of reporting back limits, current position!

