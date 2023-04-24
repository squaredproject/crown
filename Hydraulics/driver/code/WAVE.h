/* Error codes for pause_error()  */
#ifndef WAVE_H
#define WAVE_H

#include <stdint.h>

#define NORMAL 0x00        /* normal run state  */
#define ESTOP_ERROR 0x01   /* emergency stop switch is hit */
#define ENCODER_ERROR 0x02 /* no signal from encoder */
#define SWITCH_ERROR 0x03  /* more than one limit switch down */
#define HOME_ERROR 0x04    /* error on homing, can't find limit */

/* deadman switch emergency stop. If this pin is high, STOP EVERYTHING */
#define ESTOP_PIN 7     // Mask for emergency stop pin (bit 7)
#define ESTOP_PORT PINF // input port for emergency stop pin

// button port definitions
#define SPF3_SW (_BV(3)) // CLEAR switch
#define SPF4_SW (_BV(4))

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

void set_LED_out(uint8_t speed);
void set_status_LEDs(uint8_t val);
void pause_error(uint8_t code);
void Dump_Status(void);
uint8_t get_Fbuttons(uint8_t debounce);
void setRunning(uint8_t bRunning);
void clearError(void);

#endif // WAVE_H
