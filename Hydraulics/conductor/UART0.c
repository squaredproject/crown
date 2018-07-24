// -------------------------------------------------------------------------------
// UART0.c
// Routines for interrupt controlled UART
// rewritten for atmegaX8 from atmega8 by jtf July 2008
//
// rewritten for atmega2560 and file streams MAP April 2012
// -------------------------------------------------------------------------------

/* Includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include "UART0.h"


/* UART Buffer Defines */
#define UART0_RX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART0_TX_BUFFER_SIZE 128

#define UART0_RX_BUFFER_MASK ( UART0_RX_BUFFER_SIZE - 1 )
#if ( UART0_RX_BUFFER_SIZE & UART0_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif

#define UART0_TX_BUFFER_MASK ( UART0_TX_BUFFER_SIZE - 1 )
#if ( UART0_TX_BUFFER_SIZE & UART0_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif


static int uart0_put_char(char c, FILE *stream);
FILE uart0file = FDEV_SETUP_STREAM(uart0_put_char, NULL, _FDEV_SETUP_WRITE);

/* Static Variables -- Tx & Rx Ring Buffers */
static char UART0_RxBuf[UART0_RX_BUFFER_SIZE];
static volatile unsigned char UART0_RxHead;
static volatile unsigned char UART0_RxTail;
static char UART0_TxBuf[UART0_TX_BUFFER_SIZE];
static volatile unsigned char UART0_TxHead;
static volatile unsigned char UART0_TxTail;



// -------------------------------------------------------------------------------
// Init UART0 - Enable Rx Interrupts
void UART0_Init( unsigned int baud ) {
  
  UBRR0H = (unsigned char)(baud>>8);						// Set baud rate
  UBRR0L = (unsigned char)baud;								// Value depends on MPU clock speed */

  UCSR0B = _BV(RXEN0)|_BV(TXEN0)|_BV(RXCIE0)|_BV(UDRIE0);  	// Enable rx and tx, and corresponding interrupts
  UCSR0C = _BV(USBS0)|_BV(UCSZ00)|_BV(UCSZ01);				// Set frame format: 8data, 2stop bit
  
  /* Init ring buf indexes */
  UART0_RxTail = 0;
  UART0_RxHead = 0;
  UART0_TxTail = 0;
  UART0_TxHead = 0;
  
}



// ---------------------------------------------------------------------
// Check if there are any bytes waiting in the input ring buffer.
unsigned char UART0_data_in_ring_buf( void )
{
	return ( UART0_RxHead != UART0_RxTail ); /* Return 0 (FALSE) if the receive buffer is empty */
}


// -------------------------------------------------------------------------------
// Pull 1 byte from Ring Buffer of bytes received from USART
char UART0_ring_buf_byte( void )
{
	unsigned char tmptail;

	while ( UART0_RxHead == UART0_RxTail )  /* Wait for incoming data */
		;
	tmptail = ( UART0_RxTail + 1 ) & UART0_RX_BUFFER_MASK;/* Calculate buffer index */

	UART0_RxTail = tmptail;	/* Store new index */
	return UART0_RxBuf[tmptail]; /* Return data */
}


//----------------------------------------------------
// send a byte on uart0
//----------------------------------------------------
static int uart0_put_char(char c, FILE *stream)
{
	unsigned char tmphead;
	/* Calculate buffer index */
	tmphead = ( UART0_TxHead + 1 ) & UART0_TX_BUFFER_MASK; /* Wait for free space in buffer */
	while ( tmphead == UART0_TxTail );

	UART0_TxBuf[tmphead] = (unsigned char)c; /* Store data in buffer */
	UART0_TxHead = tmphead;	    /* Store new index */

	UCSR0B |= (1<<UDRIE0);	/* Enable UDRE interrupt */
    return 0;
}

// ----------------------------------------------------------------------
// Interrupt handlers - UART0 Rx vector
SIGNAL(USART0_RX_vect) {
  char data;
  unsigned char tmphead;
  
  data = UDR0;                 /* Read the received data */

  /* Calculate buffer index */
  tmphead = ( UART0_RxHead + 1 ) & UART0_RX_BUFFER_MASK;
  UART0_RxHead = tmphead;      /* Store new index */

  //  if ( tmphead == UART0_RxTail )
  //  {
      /* ERROR! Receive buffer overflow */
  //  }
	
  UART0_RxBuf[tmphead] = data; /* Store received data in buffer */
}


// ------------------------------------------------------------------------
// Interrupt handler - UART0 Tx vector for Data Register Empty - UDRE
SIGNAL(USART0_UDRE_vect)
{
  unsigned char tmptail;
  
  /* Check if all data is transmitted */
  if ( UART0_TxHead != UART0_TxTail ) {
    /* Calculate buffer index */
    tmptail = ( UART0_TxTail + 1 ) & UART0_TX_BUFFER_MASK;
    UART0_TxTail = tmptail;      /* Store new index */
    
    UDR0 = UART0_TxBuf[tmptail];  /* Start transmition */
  }
  else {
    UCSR0B &= ~(1<<UDRIE0);   /* Disable UDRE interrupt or we'll re-trigger on exit */
  }
}
