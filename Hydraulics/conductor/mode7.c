/*
 * mode2.c
 *
 *  Created on: Apr 5, 2012
 *      Author: michael
 */

#include "conductor.h"
#include "knob.h"
#include "lcd.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h> // needs F_CPU def

void apply_ui_mode7(int32_t knob[8], int32_t pacer_theta, uint8_t finger,
                    uint8_t knuckle, int32_t *amplitude_des, int32_t *phase,
                    uint8_t *frequency, uint8_t *direct_position_mode) {
  *amplitude_des = 0;
  *phase = 0;
  *frequency = 16;
  *direct_position_mode = 1;
}

void lcd_mode7(int32_t knob[8]) {
  char this_lcd_item[8][10];

  // Left Side of LCD
  sprintf(this_lcd_item[0], "ZERO ALL");
  sprintf(this_lcd_item[1], " ");
  sprintf(this_lcd_item[2], " ");
  sprintf(this_lcd_item[3], " ");

  // Right Side of LCD
  sprintf(this_lcd_item[4], " ");
  sprintf(this_lcd_item[5], " ");
  sprintf(this_lcd_item[6], " ");
  sprintf(this_lcd_item[7], " ");

  lcd_items(this_lcd_item, 7);
}
