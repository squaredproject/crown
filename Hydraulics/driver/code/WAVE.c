#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdio.h>
#include <avr/wdt.h>
#include "AD7376.h"
#include "UARTS.h"
#include "putstr.h"       
#include "parser.h"	   /* contains parser for input commands */
#include "timer.h"
#include "a2d.h"
#include "WAVE.h"
#include "sine.h"  
#include "jointPID.h"

// needs F_CPU def from WAVE.h
#include <util/delay.h>


/*  

Notes: 

Joint directions are denoted looking into the joint from the side of the knuckleboard. Thus "LEFT" and "RIGHT."  

Valves are connected so that 

A positive control voltage moves valves towards the "A" position and a
negative towards the "B" position. Control voltages are generated by a
digital potentiometer that varies from 0-127. A zero control voltage
is at setting 63, 1/2 way between the extremes.

Boards are connected so that a positive voltage comes from a pot control value of > 63 while negative values are less than that. 

There are three limit switches denoted LEFT, CENTER, and RIGHT in the
obvious places on each joint. Limit switches are normally closed,
thus active HIGH, but are mapped to a uint8 with positive logic such that if a switch is closed the corresponding bit is set High. Bits are mapped, starting from the LSB: LEFT CENTER RIGHT 0 0 0 0 0 and stored in the motor control struct as
"switches" variable. 

There are three joints numbered 1, 2, and 3 from the bottom up. Many variables for each joint are in a struct 

LEDs
There are three status LEDs on the board, and two 'output' LEDs. The status LEDs are
Red, Green, and Yellow, and are used to indicate standard operation (blinking green),
error condition (red) plus additional status LEDs depending on the specific error condition,
or yellow (using potentiometer as input).

The two output LEDs are used to indicate a value between 0 and 127 (typically, the signal
that we're sending to one of the hydraulic valves.) The two output LEDs are arranged in 
parallel but with opposite polarity, so that only one is lit at a time. For values between
0-63, the left (green) LED is lit, and for values over 63, the right (red) LED is lit. The 
intensity of the LED indicates the degree of deviation from the 0 point (63).

Joint Enabling
Each driver board controls three hydraulic valves which are attached to their respective
joints. Output to the valves can be enabled (or disabled) by putting a jumper to ground on
pins A8-A10. 
*/




//sineval = (uint16_t)pgm_read_byte(&stab[phase]);



// PORT ASSIGNMENTS:

// PORTA: (digital 22-29)  -SPI outputs for digital pots
//  
// PORTB  (PB7 is LED on digital 13)  Address on PB4,5,6 (D10, 11, 12)
// PORTC: (digital 30-37)  -Inputs for CH1, CH2 limit switches
// PORTE: (digital 1,2, 3, 5) -- LED outputs PE3, PE4, PE5 (PE7 is ICP3!)
// PORTF: (adc0-adc7)     -dig. inputs for CH3 limit switches, mode, ESTOP
// PORTG  (digital 4)  PG5 for LED pullup/down
// PORTH:  (digital 9) PH6 is OC2B for LED PWM  // XXX CSW - NOT USED
// PORTK:  PK0-PK3 enable jumpers (adc12-adc15)     -Analog inputs for dial controls
// PORTL: (digital 48, 49) PLO=ICP4, PL1=ICP5 -- input compare for pos sense

extern volatile int16_t counts[3];  // pulse width
extern volatile int16_t OFcounts[3]; // pulse width on timer3

// array of pointers to motor control structs
extern joint_control_block *jcb[3];


// =======================================================


#define RED_LED (_BV(3))         /*  red LED (PE3, digital 5) */
#define YEL_LED (_BV(4)) 	     /*  yel LED (PE4, digital 5) */
#define GRN_LED (_BV(5)) 	     /*  grn LED (PE5, digital 5) */

//button port definitions

//#define SPF3_SW (_BV(3))
//#define SPF4_SW (_BV(4))


#define NORMAL 0
#define ERROR  1


uint8_t state = NORMAL; 			/* address bits  */
uint8_t addr = 0; 					/* address bits  */

/* preserve this during interrupts if necessary */
volatile uint16_t pwm=0;


/* flags and counts for Timer0 */
extern volatile uint16_t Timer0_ticks;
extern volatile uint8_t KHZ_Flag;


volatile uint16_t omega0 = 0; 	 /* actual phase, 0 <= omega < TABLENGTH */

static int isRunning = FALSE;

double ftablemax = (double)(TABLEMAX/2);


uint8_t get_sine(uint16_t omega, double amp) {
  uint16_t sinval  = pgm_read_word(&stab[omega]);
  return (uint8_t) 63 + lrint(63 * amp * (( ((double) (sinval)) - ftablemax)  / ftablemax));
}


uint8_t Test_encoder_limits(uint16_t count) {
//  if ((count < 1500) || (count > 6500)) {
//    return(1);
//  }
  
  return(0);
}

uint8_t Test_limits(uint16_t count, int jointIdx) {
  uint8_t limits = 0;
  
  if (jcb[jointIdx]->currentPos <= jcb[jointIdx]->minpos) {
    limits |= LEFT_SW; 
  }

  if (jcb[jointIdx]->currentPos >= jcb[jointIdx]->maxpos) {
    limits |= RIGHT_SW; 
  }
  return limits;
}

/* return the number of bits set in this byte */
uint8_t count_bits(uint8_t byte){
  uint8_t c;
  for (c = 0; byte; byte >>= 1) {
	c += byte & 0x01;
  }
  return(c);
}

uint8_t get_Fbuttons(uint8_t debounce) {
  static uint8_t last_state = 0xFF;
  uint8_t bstate = ~PINF & (SPF3_SW | SPF4_SW); 
  if(! debounce) 		/* return raw button state (one if pressed) */
    return(bstate);
  else { 			/* only return on zero-one transition */
    debounce = bstate & ~last_state;
    last_state = bstate;
    return(debounce);
  }
}


// set the output LEDs: red for vals 64-128, green for 0-63
// This function is used for debugging, often to indicate the 
// input to one of the hydraulics valves
void set_LED_out(uint8_t val) {

  DDRG |= _BV(5); // Use digital pin PG5 (arduino digital pin 4)

  val = val & 0x7F; // restrict to 0-127
  val = 127 - val; // green is POS (rightward), RED is NEG (leftward)
  if ( val >=64) {
    TIMER2_PWM(((val-64) << 2)); // map 64-127 to 0-255
    /* make the source pin high: PE2 is digital out 4 */
    PORTG &= ~_BV(5);
  }
  else {
    /* light the green */
    TIMER2_PWM(((val) << 2)); // map 63-0 to 0-255
    /* make the source pin low to light the red */
    PORTG |= _BV(5);
  }
}


// set the output LEDs on PE 3,4,5: red, green, yel
void set_status_LEDs(uint8_t val) {
  //val &= RED_LED | GRN_LED | YEL_LED;
  PORTE |= val;
}

void clear_status_LEDs(void) {
  PORTE &= ~(RED_LED | GRN_LED | YEL_LED);
}


/* error condition, halt and blink LEDs */
void pause_error(uint8_t code) {

  state = ERROR;
  /* wait for button press to clear error again */

  DP_reset(); // turn off all valves

  switch(code) {

  case ESTOP_ERROR:
	putstr("\r\nERROR: ESTOP ");
	set_status_LEDs(RED_LED);
	break;

  case ENCODER_ERROR:
	putstr("\r\nERROR: encoder ");
	set_status_LEDs(RED_LED | YEL_LED);
	break;

  case SWITCH_ERROR:
	putstr("\r\nERROR: limits ");
	set_status_LEDs(RED_LED | GRN_LED);
	break;

  case HOME_ERROR:
	putstr("\r\nERROR: homing ");
	set_status_LEDs(RED_LED | GRN_LED | YEL_LED);
	break;
  }

}

int main( void ){
  
  uint8_t cData; 		/* byte to read from UART */
  uint8_t i;					/* loop counter */
  uint16_t heartcount = 0; 		/* counter for heartbeat led */
  uint8_t tenhz_count = 0; 		/* counter for 10hz loop */
  uint8_t toggle = 0;
  uint8_t button_state = 0;
  int16_t target = 0; 		/* dc value read from ADC */
  uint8_t enable = 0; 				/* Which joints are enabled */
  uint8_t drive = 0; 				/* temp value */
  uint8_t bAtLimit = FALSE;    /* Whether we've hit a limit switch or not*/
  /* CSB  on PD6 (output) */
  /* DRDY on PD7 (input) */

  DDRD = 0xF0;		  /* 0b 1111 1111	   1 indicates Output pin */
  PORTD = 0x0F; 				/* pullups on low portD */

  PORTD = 0xFF;	          /* turn on pullups */

  DDRK = 0x00; 				/* PORTK: low 4 bits digital in, high 4 a2d */
  PORTK = 0x0F;					/* pullups on on low 4 bits */

  DDRB = 0x80;
  PORTB = 0x70;
  
  //UART_Init(UART_9600); 
  UART0_Init(UART_115200); 
  UART1_Init(UART_115200); 
  
  Timer0_Init();		/* Init Timer 0 for tick interrupt */
  PWM_Init();

/* Timer3: Joint 3 encoder timer on pin PE7 (ICP3) */
/* Timer4: Joint 1 encoder timer on pin PL0 (ICP4) */
/* Timer5: Joint 2 encoder timer on pin PL1 (ICP5) */

  ICP3_Init();			/* init timer3 for jointx PWM measure  */
  ICP4_Init();			/* init timer4 for jointx PWM measure  */
  ICP5_Init();			/* init timer5 for jointx PWM measure  */
  A2D_Init(); 			/* int ADC8-15 for analog input */
  PID_Init();			/* init data structures for joint PID loops */

  DDRC = 0x00;			/* pins on PORTC designated for limit sw, estop */
  PORTC = 0xFF;			/* pullups on */

  DDRF = 0x00;			/* pins on PORTF designated for limit sw, estop */
  PORTF = 0xFF;			/* pullups on */


  DDRE = 0x7F; 					/* for LEDS on PE3, PE4, PE5, PE7 is ICP3 */

  sei();
  DP_enable(); 					/* set up valve outputs */


  set_status_LEDs(0); 			/* start with status LEDS off */

  
  sei();
  
  // start with no power to the drives
  DP_SendValue(63,0);
  DP_SendValue(63,1);
  DP_SendValue(63,2);

  _delay_ms(100);
  
  _delay_ms(100);
  
  /* determine address from jumpers on PB4 (10), PB5(11),PB6(12),PB7(13) */
  /* Note that this needs to go below the delay - there appears to be a race condition
     on read that affects some (but not all) boards */
  addr = (~PINB & 0x70) >> 4;


  if(1){
    putstr("WAVE v0.10_CSW\r\n");
    putstr("addr: ");
    puthex(addr);
    putstr("\r\n");
  }
  
  // start main loop =======================================================
  for(;;)    {

    A2D_poll_adc(); // check if adc conversion is done
    
    // Parser for data from the USB
    if (UART0_data_in_ring_buf()) { // check for waiting UART data from SPU
      cData = UART0_ring_buf_byte(); // get next char from ring buffer... 
      if(accumulateCommandString0(cData) ){ // ... and add to command string
        // if we are here we have a complete command; parse it
        parseCommand(0); // parse and execute commands
      }
    }

    // Parser for data on the 485 bus
    if (UART1_data_in_ring_buf()) { // check for waiting UART data from SPU
      //char buf[16];
      cData = UART1_ring_buf_byte(); // get next char from ring buffer... 
      //sprintf(buf, "485 Receive %c\n", cData);
      //putstr(buf);
	  if(accumulateCommandString1(cData) ){ // ... and add to command string
		// if we are here we have a complete command; parse it
		parseCommand(1); // parse and execute commands
      }
    }
    
    
	/* roughly 976 hz at 16Mhz clock */
    if (KHZ_Flag) {	
      KHZ_Flag = 0;
      
      Poll_Limit_Switches();

      if(tenhz_count++ > 98) {/* main loop at ~ten hz */
        tenhz_count = 0;

        if (!isRunning) {
            set_status_LEDs(RED_LED);
            continue;
        }
        
        /* first, check enable pins PORTK 0, 1, 2
         only test and control a joint if corresponing enable is set */
        enable = ~PINK;
        
        /* get current values for limit switches */

        /* Error on deadman switch not set */
        if (ESTOP_PORT & _BV(ESTOP_PIN)) {
          pause_error(ESTOP_ERROR);
        } 
        
#ifdef foo
		else if (ESTOP_PORT & _BV(5)) {  /* sinusoid with PID */
		  omega0 += (A2D_read_channel(0) >> 5) + 1;
		  if (omega0 >= TABLENGTH)
			omega0 -= TABLENGTH;
		  target =  get_sine(omega0,
				     (double) A2D_read_channel(1)/1024.0) - 63;

		  target *= 8;
		  //putstr("\r\nsine: ");
		  //putS16((int16_t)target);
		  jcb[0]->targetPos = target;

		  /* update phase */
		  drive = Joint_Servo_Task(0);
		  set_LED_out(drive);
		  //DP_SendValue(drive,0);
		}
#endif

        else if (state == NORMAL) { /* no error, operate normally */
          int bLimitReached = FALSE;
          for(i=0;i<3;i++){     /* for each joint, get inputs, set output */
            if (enable & _BV(i)){
               // Sanity test that the encoders are giving us good data
              if (Test_encoder_limits(counts[i])) { /* encoders in range ? */
                                char buf[128];
                sprintf(buf, "\r\nEncoder limit error on joint %d, value %d", i, counts[i]);
                putstr(buf);
                pause_error(ENCODER_ERROR);
              }
              // Another sanity test of encoder data. 'OF' is overflow - ie, we haven't
              // seen the expected edge on the PWM signal. Typically this error will happen
              // if the encoder is not physically connected.
              else if ((OFcounts[i] > 15) ) {
                putstr("\r\n OF: ");
                putS16(OFcounts[i]);
                pause_error(ENCODER_ERROR);
              }

              else { /* all tests OK, do the normal thing */

                if (~ESTOP_PORT & _BV(6)) {  /* gemerate target pos from pot */
                  if (jcb[i]->trace) {
                    putstr("\r\nReading target from local pot");
                  }
                  target = A2D_read_channel(7) - 512;
                  jcb[i]->targetPos = target;
                  set_status_LEDs(YEL_LED);
                }

                /* calculate drive from PID position */
                drive = Joint_Servo_Task(jcb[i]);
                
                /* now figure out whether we're over the limits */
                uint8_t softLimits = Test_limits(counts[i], i);
                uint8_t bLeftLimit  = ((softLimits & LEFT_SW)  && (jcb[i]->switches & LEFT_SW));
                uint8_t bRightLimit = ((softLimits & RIGHT_SW) && (jcb[i]->switches & RIGHT_SW));
                
                /* If we're trying to drive the system past its limits, don't */
                if ((drive < 63 && bLeftLimit) || (drive > 63 && bRightLimit)) { 
                  drive = 63;
                  jcb[i]->drive = drive;
                  bLimitReached = TRUE;
                }
                // Normal condition - set valve to derived value
                DP_SendValue((uint8_t)drive,i);
                /* set debug LEDs according to trace 
                (Shows first joint unless asked to trace a different one*/
                if (jcb[i]->trace | (0 == i)) {
                  set_LED_out((uint8_t)drive);
                }                
              } 
            } 
            else {              /* joint disbled; make sure valve is off */
              if (jcb[i]->trace) {
                putstr("\r\n Joint not enabled");
              }
              DP_SendValue(63,i);
            }
          } /* end for(i) */

          if (bLimitReached) {
            set_status_LEDs(YEL_LED | GRN_LED); 
            bAtLimit = TRUE;
          } else if (bAtLimit) {
            clear_status_LEDs();
            bAtLimit = FALSE;
          }
        }/* end normal op block */
      } /* end tenhz */
      
      /* blink hearbeat at 1s pattern rate */
      if (!bAtLimit && heartcount++ >= 488) {
        heartcount = 0;
        toggle = 1 - toggle;
        if (state == NORMAL) {
          if (toggle) {
            PORTE |= GRN_LED;
          }
          else {
            PORTE &= ~GRN_LED;
            PORTE &= ~YEL_LED;
          }
        }
      }

      button_state = get_Fbuttons(0);
      if (state == ERROR) {
        if (button_state &  SPF3_SW ){
          putstr("\r\n***Clear!***");
          clear_status_LEDs();
          state = NORMAL;
          DP_enable();
        }
      } if (button_state & SPF4_SW) {
        // reset the fucking thing. Watchdog?
        //wdt_enable(WDTO_30MS);
        //while(1) {};
      }
    }/* endif 100hz */
  } /* end while(1) */
}

void Dump_Status(void) {
  char buf[128];
  uint8_t enable = ~PINK;
  putstr("\r\n***GLOBAL STATUS***\n");
  addr = (~PINB & 0x70) >> 4; // Get addr again, just in case
  sprintf(buf, "\r\n  Addr is            : %d", addr);
  putstr(buf);
  sprintf(buf, "\r\n  Running is         : %s", isRunning ? "TRUE" : "FALSE");
  putstr(buf);
  sprintf(buf, "\r\n  Use Pot for Target : %s", (~ESTOP_PORT & _BV(6)) ? "TRUE" : "FALSE");
  putstr(buf);
  sprintf(buf, "\r\n  Estop Triggered    : %s",  ((ESTOP_PORT & _BV(ESTOP_PIN)) ? "TRUE" : "FALSE"));
  putstr(buf);
  sprintf(buf, "\r\n  Error              : %s",  (state == ERROR ? "TRUE" : "FALSE"));
  putstr(buf);
  for (int i=0; i<3; i++) {
    sprintf(buf, "\r\n  Joint %d Enabled : %s", i, ((enable & _BV(i)) ? "TRUE" : "FALSE"));
    putstr(buf);
  }  
  for (int i=0; i<3; i++) {
    sprintf(buf, "\r\n  Joint %d Homed   : %s", i, (jcb[i]->homed ? "TRUE" : "FALSE"));
    putstr(buf);
  }
}

void setRunning(uint8_t bRunning) {
    isRunning = (bRunning == TRUE);
}



