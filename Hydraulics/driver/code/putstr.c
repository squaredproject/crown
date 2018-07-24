// putstr.c - Utility routines for outputing data to the serial port

#include <avr/io.h>
#include "UARTS.h"
#include "putstr.h"

// --------------------------------------------------------------------------
// Send a string of chars out the UARTn port.
// Chrs are put into circular ring buffer.
// Routine returns immediately unless no room in Ring Buf. 
// Interrupts Transmit chrs out of ring buf.

void putstr(char *str)
{
  uint8_t ch;

  while((ch=*str)!= '\0')
  {
    UART0_send_byte(ch);
    str++;
  }
}

void putchr(char c)
{
    UART0_send_byte(c);
}


// --------------------------------------------------------------------

void putU8(uint8_t number)
{
  uint8_t value[3]={0,0,0};

  //if(number == 0) {
  //  UART0_send_byte('0');
  //  return;
  //}
  while((number - 100)>=0)
  {
    number -= 100;
    value[2]++;
  }
  value[2] += '0';

  while((number - 10)>=0)
  {
    number -= 10;
    value[1]++;
  }
  value[1] += '0';

  value[0] = number + '0';

  UART0_send_byte(value[2]);
  UART0_send_byte(value[1]);
  UART0_send_byte(value[0]);
}

// --------------------------------------------------------------------

/* print out binary representation of char */
void putB8(uint8_t number)
{
  uint8_t i;
  for (i=0;i<8;i++){
    UART0_send_byte((number & 0x80) ? '1':'0');
    number = number << 1;
  }
}

// --------------------------------------------------------------------------

void puthex(uint8_t byte)
{
  uint8_t nib;
  
  nib = (byte & 0xF0) >> 4;
  if (nib > 9) 
	UART0_send_byte(nib - 10 + 'A');
  else
	UART0_send_byte(nib + '0');

  nib = byte & 0x0F;
  if (nib > 9) 
	UART0_send_byte(nib - 10 + 'A');
  else
	UART0_send_byte(nib + '0');

}



void putint(int32_t number)
{
  uint8_t value[7]={0,0,0,0,0,0,0};

  if (number < 0) {
	number = -number;
	UART0_send_byte('-');
  }

  while((number - 1000000)>=0)
  {
    number -= 1000000;
    value[6]++;
  }
  value[6] += '0';

  while((number - 100000)>=0)
  {
    number -= 100000;
    value[5]++;
  }
  value[5] += '0';

  while((number - 10000)>=0)
  {
    number -= 10000;
    value[4]++;
  }
  value[4] += '0';

  while((number - 1000)>=0)
  {
    number -= 1000;
    value[3]++;
  }
  value[3] += '0';

  while((number - 100)>=0)
  {
    number -= 100;
    value[2]++;
  }
  value[2] += '0';

  while((number - 10)>=0)
  {
    number -= 10;
    value[1]++;
  }
  value[1] += '0';

  value[0] = number + '0';

  //UART0_send_byte(32);	// space
  //

  UART0_send_byte(value[6]);
  UART0_send_byte(value[5]);
  UART0_send_byte(value[4]);
  UART0_send_byte(value[3]);
  UART0_send_byte(value[2]);
  UART0_send_byte(value[1]);
  UART0_send_byte(value[0]);
}

void putS16(int16_t number)
{
  uint8_t value[6]={0,0,0,0,0,0};

  if(number >= 0)
  {
    value[5]='+';
  }
  else
  {
    value[5]='-';
    number *= -1;
  }

  while((number - 10000)>=0)
  {
    number -= 10000;
    value[4]++;
  }
  value[4] += '0';

  while((number - 1000)>=0)
  {
    number -= 1000;
    value[3]++;
  }
  value[3] += '0';

  while((number - 100)>=0)
  {
    number -= 100;
    value[2]++;
  }
  value[2] += '0';

  while((number - 10)>=0)
  {
    number -= 10;
    value[1]++;
  }
  value[1] += '0';

  value[0] = number + '0';

  UART0_send_byte(32);	// space
  UART0_send_byte(value[5]);
  UART0_send_byte(value[4]);
  UART0_send_byte(value[3]);
  UART0_send_byte(value[2]);
  UART0_send_byte(value[1]);
  UART0_send_byte(value[0]);
}
