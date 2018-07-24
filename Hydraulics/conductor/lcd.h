/*
 * lcd.h
 *
 *  Created on: Apr 5, 2012
 *      Author: michael
 */

#ifndef LCD_H_
#define LCD_H_

void lcd_set_baud(void);

void lcd_init(void);

void lcd_line1(void);
void lcd_line2(void);
void lcd_line3(void);
void lcd_line4(void);
void lcd_default(uint16_t knob[8],uint8_t mode);
void lcd_items( char lcd_item[8][10], uint8_t mode );

#endif /* LCD_H_ */
