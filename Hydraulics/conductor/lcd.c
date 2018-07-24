/*
 * lcd.c
 *
 *  Created on: Apr 5, 2012
 *      Author: michael
 */

#include <stddef.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <util/delay.h> // needs F_CPU def

#include "UARTbaudrates.h" // needs F_CPU def
#include "UART0.h"
#include "UART1.h"
#include "UART2.h"
#include "timer.h"
#include "conductor.h"
#include "sinusoid.h"
#include "lcd.h"


// this seemed to bork the display; needs testing
void lcd_set_baud(void)
{
	  UART2_Init(UART_9600);	// LCD Baud Rate
	  _delay_ms 	(100);
	  fprintf(LCDFILE,"\xfe\x61\x06");
	  _delay_ms 	(100);
	  UART2_Init(UART_19200);	// LCD Baud Rate
	  _delay_ms 	(100);
}

void lcd_init(void)
{
    		fprintf(LCDFILE,"\xfe\x53\x08\xfe\x46\xFE\x51");
}

void lcd_default(uint16_t knob[8],uint8_t mode)
{
	lcd_line1();
	fprintf(LCDFILE, "FNGPH:%03u     R1:%03u", knob[3]>>3, knob[7]>>2 );
	lcd_line2();
	fprintf(LCDFILE, "NUKPH:%03u     R2:%03u", knob[2]>>3,  knob[6]>>2 );
	lcd_line3();
	fprintf(LCDFILE, "L3:%03u  MODE  R3:%03u", knob[1]>>2, knob[5]>>2 );
	lcd_line4();
	fprintf(LCDFILE, "AMP:%03u  %02u  PER:%03u", knob[0]>>2, mode, (knob[4]>>2)+20 );
}

void lcd_items( char lcd_item[8][10], uint8_t mode )
{
	lcd_line1();
	fprintf(LCDFILE, "%-9s  %9s", lcd_item[0],lcd_item[4] );
	lcd_line2();
	fprintf(LCDFILE, "%-9s  %9s", lcd_item[1],lcd_item[5] );
	lcd_line3();
	fprintf(LCDFILE, "%-9s  %9s", lcd_item[2],lcd_item[6] );
	lcd_line4();
	fprintf(LCDFILE, "%-9sM%1u%9s", lcd_item[3], mode, lcd_item[7] );
}

void lcd_line1(void)
{
	fprintf(LCDFILE,"\xfe\x46");
}

void lcd_line2(void)
{
	fprintf(LCDFILE,"\xfe\x45\x40");
}

void lcd_line3(void)
{
	fprintf(LCDFILE,"\xfe\x45\x14");
}

void lcd_line4(void)
{
	fprintf(LCDFILE,"\xfe\x45\x54");
}

