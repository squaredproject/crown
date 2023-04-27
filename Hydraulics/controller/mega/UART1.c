// -------------------------------------------------------------------------------
// UART1.c
// Routines for interrupt controlled UART
// rewritten for atmegaX8 from atmega8 by jtf July 2008
//
// rewritten for atmega2560 and file streams MAP April 2012
// -------------------------------------------------------------------------------

#include "UART1.h"

/* Includes */
#include <avr/interrupt.h>
#include <avr/io.h>

// from comm.c
#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* UART Buffer Defines */
#define UART1_RX_BUFFER_SIZE 128 /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART1_TX_BUFFER_SIZE 128

#define UART1_RX_BUFFER_MASK (UART1_RX_BUFFER_SIZE - 1)
#if (UART1_RX_BUFFER_SIZE & UART1_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2
#endif

#define UART1_TX_BUFFER_MASK (UART1_TX_BUFFER_SIZE - 1)
#if (UART1_TX_BUFFER_SIZE & UART1_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

static int uart1_put_char(char c, FILE *stream);
FILE uart1file = FDEV_SETUP_STREAM(uart1_put_char, NULL, _FDEV_SETUP_WRITE);

/* Static Variables -- Tx & Rx Ring Buffers */
static char UART1_RxBuf[UART1_RX_BUFFER_SIZE];
static volatile unsigned char UART1_RxHead;
static volatile unsigned char UART1_RxTail;
static char UART1_TxBuf[UART1_TX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;

// -------------------------------------------------------------------------------
// Init UART1 - Enable Rx Interrupts
void UART1_Init(unsigned int baud) {

  UBRR1H = (unsigned char)(baud >> 8); // Set baud rate
  UBRR1L = (unsigned char)baud;        // Value depends on MPU clock speed */

  UCSR1B = _BV(RXEN1) | _BV(TXEN1) | _BV(RXCIE1) |
           _BV(UDRIE1); // Enable rx and tx, and corresponding interrupts
  UCSR1C = _BV(USBS1) | _BV(UCSZ10) |
           _BV(UCSZ11); // Set frame format: 8data, 2stop bit

  /* Init ring buf indexes */
  UART1_RxTail = 0;
  UART1_RxHead = 0;
  UART1_TxTail = 0;
  UART1_TxHead = 0;

  // UCSR0B |= (1<<RXCIE0);						/* Enable Rx interrupt
  // */
}

// ---------------------------------------------------------------------
// Check if there are any bytes waiting in the input ring buffer.
unsigned char UART1_data_in_ring_buf(void) {
  return (UART1_RxHead !=
          UART1_RxTail); /* Return 0 (FALSE) if the receive buffer is empty */
}

// -------------------------------------------------------------------------------
// Pull 1 byte from Ring Buffer of bytes received from USART
char UART1_ring_buf_byte(void) {
  unsigned char tmptail;

  while (UART1_RxHead == UART1_RxTail) /* Wait for incoming data */
    ;
  tmptail =
      (UART1_RxTail + 1) & UART1_RX_BUFFER_MASK; /* Calculate buffer index */

  UART1_RxTail = tmptail;      /* Store new index */
  return UART1_RxBuf[tmptail]; /* Return data */
}

//----------------------------------------------------
// send a byte on uart1
//----------------------------------------------------
static int uart1_put_char(char c, FILE *stream) {
  unsigned char tmphead;
  /* Calculate buffer index */
  tmphead = (UART1_TxHead + 1) &
            UART1_TX_BUFFER_MASK; /* Wait for free space in buffer */
  while (tmphead == UART1_TxTail)
    ;

  UART1_TxBuf[tmphead] = (unsigned char)c; /* Store data in buffer */
  UART1_TxHead = tmphead;                  /* Store new index */

  UCSR1B |= (1 << UDRIE1); /* Enable UDRE interrupt */
  return 0;
}

// ----------------------------------------------------------------------
// Interrupt handlers - UART1 Rx vector
SIGNAL(USART1_RX_vect) {
  char data;
  unsigned char tmphead;

  data = UDR1; /* Read the received data */

  /* Calculate buffer index */
  tmphead = (UART1_RxHead + 1) & UART1_RX_BUFFER_MASK;
  UART1_RxHead = tmphead; /* Store new index */

  //  if ( tmphead == UART1_RxTail )
  //  {
  /* ERROR! Receive buffer overflow */
  //  }

  UART1_RxBuf[tmphead] = data; /* Store received data in buffer */
}

// ------------------------------------------------------------------------
// Interrupt handler - UART1 Tx vector for Data Register Empty - UDRE
SIGNAL(USART1_UDRE_vect) {
  unsigned char tmptail;

  /* Check if all data is transmitted */
  if (UART1_TxHead != UART1_TxTail) {
    /* Calculate buffer index */
    tmptail = (UART1_TxTail + 1) & UART1_TX_BUFFER_MASK;
    UART1_TxTail = tmptail; /* Store new index */

    UDR1 = UART1_TxBuf[tmptail]; /* Start transmition */
  } else {
    UCSR1B &=
        ~(1 << UDRIE1); /* Disable UDRE interrupt or we'll re-trigger on exit */
  }
}
