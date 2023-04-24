// -------------------------------------------------------------------------------
// UART2.c
// Routines for interrupt controlled UART
// rewritten for atmegaX8 from atmega8 by jtf July 2008
//
// rewritten for atmega2560 and file streams MAP April 2012
// -------------------------------------------------------------------------------

/* Includes */
#include <avr/interrupt.h>
#include <avr/io.h>

// from comm.c
#include <avr/pgmspace.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "UART2.h"

/* UART Buffer Defines */

#define UART2_RX_BUFFER_SIZE 128 /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART2_TX_BUFFER_SIZE 128

#define UART2_RX_BUFFER_MASK (UART2_RX_BUFFER_SIZE - 1)
#if (UART2_RX_BUFFER_SIZE & UART2_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2
#endif

#define UART2_TX_BUFFER_MASK (UART2_TX_BUFFER_SIZE - 1)
#if (UART2_TX_BUFFER_SIZE & UART2_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

static int uart2_put_char(char c, FILE *stream);
FILE uart2file = FDEV_SETUP_STREAM(uart2_put_char, NULL, _FDEV_SETUP_WRITE);

/* Static Variables -- Tx & Rx Ring Buffers */
static char UART2_RxBuf[UART2_RX_BUFFER_SIZE];
static volatile unsigned char UART2_RxHead;
static volatile unsigned char UART2_RxTail;
static char UART2_TxBuf[UART2_TX_BUFFER_SIZE];
static volatile unsigned char UART2_TxHead;
static volatile unsigned char UART2_TxTail;

// -------------------------------------------------------------------------------
// Init UART2 - Enable Rx Interrupts
void UART2_Init(unsigned int baud) {
  // unsigned char x;

  UBRR2H = (unsigned char)(baud >> 8); // Set baud rate
  UBRR2L = (unsigned char)baud;        // Value depends on MPU clock speed */

  UCSR2B = _BV(RXEN2) | _BV(TXEN2) | _BV(RXCIE2) |
           _BV(UDRIE2); // Enable rx and tx, and corresponding interrupts
  UCSR2C = _BV(USBS2) | _BV(UCSZ20) |
           _BV(UCSZ21); // Set frame format: 8data, 2stop bit

  /* Init ring buf indexes */
  UART2_RxTail = 0;
  UART2_RxHead = 0;
  UART2_TxTail = 0;
  UART2_TxHead = 0;
}

// ---------------------------------------------------------------------
// Check if there are any bytes waiting in the input ring buffer.
unsigned char UART2_data_in_ring_buf(void) {
  return (UART2_RxHead !=
          UART2_RxTail); /* Return 0 (FALSE) if the receive buffer is empty */
}

// -------------------------------------------------------------------------------
// Pull 1 byte from Ring Buffer of bytes received from USART
char UART2_ring_buf_byte(void) {
  unsigned char tmptail;

  while (UART2_RxHead == UART2_RxTail) /* Wait for incoming data */
    ;
  tmptail =
      (UART2_RxTail + 1) & UART2_RX_BUFFER_MASK; /* Calculate buffer index */

  UART2_RxTail = tmptail;      /* Store new index */
  return UART2_RxBuf[tmptail]; /* Return data */
}

//----------------------------------------------------
// send a byte on uart2
//----------------------------------------------------
static int uart2_put_char(char c, FILE *stream) {
  unsigned char tmphead;
  /* Calculate buffer index */
  tmphead = (UART2_TxHead + 1) &
            UART2_TX_BUFFER_MASK; /* Wait for free space in buffer */
  while (tmphead == UART2_TxTail)
    ;

  UART2_TxBuf[tmphead] = (unsigned char)c; /* Store data in buffer */
  UART2_TxHead = tmphead;                  /* Store new index */

  UCSR2B |= (1 << UDRIE2); /* Enable UDRE interrupt */
  return 0;
}

// ----------------------------------------------------------------------
// Interrupt handlers - UART2 Rx vector
SIGNAL(USART2_RX_vect) {
  char data;
  unsigned char tmphead;

  data = UDR2; /* Read the received data */

  /* Calculate buffer index */
  tmphead = (UART2_RxHead + 1) & UART2_RX_BUFFER_MASK;
  UART2_RxHead = tmphead; /* Store new index */

  //  if ( tmphead == UART2_RxTail )
  //  {
  /* ERROR! Receive buffer overflow */
  //  }

  UART2_RxBuf[tmphead] = data; /* Store received data in buffer */
}

// ------------------------------------------------------------------------
// Interrupt handler - UART2 Tx vector for Data Register Empty - UDRE
SIGNAL(USART2_UDRE_vect) {
  unsigned char tmptail;

  /* Check if all data is transmitted */
  if (UART2_TxHead != UART2_TxTail) {
    /* Calculate buffer index */
    tmptail = (UART2_TxTail + 1) & UART2_TX_BUFFER_MASK;
    UART2_TxTail = tmptail; /* Store new index */

    UDR2 = UART2_TxBuf[tmptail]; /* Start transmition */
  } else {
    UCSR2B &=
        ~(1 << UDRIE2); /* Disable UDRE interrupt or we'll re-trigger on exit */
  }
}
