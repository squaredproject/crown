// ----------------------------------------------------------------------------
// UARTS.c
// Routines for interrupt controlled UART0 and UART1
// rewritten for atmegaX8 from atmega8 by jtf July 2008
// rewritten for atmega2560 from atmega8/168 by jtf May 2012
// ----------------------------------------------------------------------------

/* Includes */
#include <avr/interrupt.h>
#include <avr/io.h>

#include "UARTS.h"

/* UART1 Buffer Defines */
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

/* Static Variables -- Tx & Rx Ring Buffers */
static unsigned char UART1_RxBuf[UART1_RX_BUFFER_SIZE];
static volatile unsigned char UART1_RxHead;
static volatile unsigned char UART1_RxTail;
static unsigned char UART1_TxBuf[UART1_TX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;

/* UART0 Buffer Defines */
#define UART0_RX_BUFFER_SIZE 128 /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART0_TX_BUFFER_SIZE 128

#define UART0_RX_BUFFER_MASK (UART0_RX_BUFFER_SIZE - 1)
#if (UART0_RX_BUFFER_SIZE & UART0_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2
#endif

#define UART0_TX_BUFFER_MASK (UART0_TX_BUFFER_SIZE - 1)
#if (UART0_TX_BUFFER_SIZE & UART0_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

/* Static Variables -- Tx & Rx Ring Buffers */
static unsigned char UART0_RxBuf[UART0_RX_BUFFER_SIZE];
static volatile unsigned char UART0_RxHead;
static volatile unsigned char UART0_RxTail;
static unsigned char UART0_TxBuf[UART0_TX_BUFFER_SIZE];
static volatile unsigned char UART0_TxHead;
static volatile unsigned char UART0_TxTail;

// -------------------------------------------------------------------------------
// Init UART1 - Enable Rx Interrupts

/* define UART1 to use UART1, otherwise default to UART0 */
// #define UART1 hoohah

void UART1_Init(unsigned int baud) {
  // unsigned char x;

  UBRR1H = (unsigned char)(baud >> 8); /* Set baud rate */
  UBRR1L = (unsigned char)baud;        /* Value depends on MPU clock speed */

  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << UDRIE1);
  // UCSR1B = (1<<RXEN1)|(1<<TXEN1);
  /* Enable rx and tx, and corresponding interrupts */
  UCSR1C =
      (1 << USBS1) | (3 << UCSZ10); /* Set frame format: 8data, 2stop bit */

  // x = 0;										/* Init ring buf indexes
  // */ UART1_RxTail = x; UART1_RxHead = x; UART1_TxTail = x; UART1_TxHead = x;

  // UCSR0B |= (1<<RXCIE0);						/* Enable Rx interrupt
  // */
}

// ----------------------------------------------------------------------
// Interrupt handlers - UART1 Rx vector

// SIGNAL(USART_RX_vect){
SIGNAL(USART1_RX_vect) {
  unsigned char data;
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
// SIGNAL(USART_TX_vect)

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

// ---------------------------------------------------------------------
// Check if there are any bytes waiting in the input ring buffer.

unsigned char UART1_data_in_ring_buf(void) {
  return (UART1_RxHead !=
          UART1_RxTail); /* Return 0 (FALSE) if the receive buffer is empty */
}

// -------------------------------------------------------------------------------
// Pull 1 byte from Ring Buffer of bytes received from USART

unsigned char UART1_ring_buf_byte(void) {
  unsigned char tmptail;

  while (UART1_RxHead == UART1_RxTail) /* Wait for incoming data */
    ;
  tmptail =
      (UART1_RxTail + 1) & UART1_RX_BUFFER_MASK; /* Calculate buffer index */

  UART1_RxTail = tmptail;      /* Store new index */
  return UART1_RxBuf[tmptail]; /* Return data */
}

// ----------------------------------------------------

void UART1_send_byte(unsigned char data) {
  unsigned char tmphead;
  /* Calculate buffer index */
  tmphead = (UART1_TxHead + 1) &
            UART1_TX_BUFFER_MASK; /* Wait for free space in buffer */
  while (tmphead == UART1_TxTail)
    ;

  UART1_TxBuf[tmphead] = data; /* Store data in buffer */
  UART1_TxHead = tmphead;      /* Store new index */

  UCSR1B |= (1 << UDRIE1); /* Enable UDRE interrupt */
}

void UART0_Init(unsigned int baud) {
  unsigned char x;

  UBRR0H = (unsigned char)(baud >> 8); /* Set baud rate */
  UBRR0L = (unsigned char)baud;        /* Value depends on MPU clock speed */

  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
  /* Enable rx and tx, and corresponding interrupts */
  UCSR0C =
      (1 << USBS0) | (3 << UCSZ00); /* Set frame format: 8data, 2stop bit */

  x = 0; /* Init ring buf indexes */
  UART0_RxTail = x;
  UART0_RxHead = x;
  UART0_TxTail = x;
  UART0_TxHead = x;

  // UCSR0B |= (1<<RXCIE0);						/* Enable Rx interrupt
  // */
}

// ----------------------------------------------------------------------
// Interrupt handlers - UART0 Rx vector

// SIGNAL(USART_RX_vect){
SIGNAL(USART0_RX_vect) {
  unsigned char data;
  unsigned char tmphead;

  data = UDR0; /* Read the received data */

  /* Calculate buffer index */
  tmphead = (UART0_RxHead + 1) & UART0_RX_BUFFER_MASK;
  UART0_RxHead = tmphead; /* Store new index */

  //  if ( tmphead == UART0_RxTail )
  //  {
  /* ERROR! Receive buffer overflow */
  //  }

  UART0_RxBuf[tmphead] = data; /* Store received data in buffer */
}

// ------------------------------------------------------------------------
// Interrupt handler - UART0 Tx vector for Data Register Empty - UDRE
// SIGNAL(USART_TX_vect)

SIGNAL(USART0_UDRE_vect) {
  unsigned char tmptail;

  /* Check if all data is transmitted */
  if (UART0_TxHead != UART0_TxTail) {
    /* Calculate buffer index */
    tmptail = (UART0_TxTail + 1) & UART0_TX_BUFFER_MASK;
    UART0_TxTail = tmptail; /* Store new index */

    UDR0 = UART0_TxBuf[tmptail]; /* Start transmition */
  } else {
    UCSR0B &=
        ~(1 << UDRIE0); /* Disable UDRE interrupt or we'll re-trigger on exit */
  }
}

// ---------------------------------------------------------------------
// Check if there are any bytes waiting in the input ring buffer.

unsigned char UART0_data_in_ring_buf(void) {
  return (UART0_RxHead !=
          UART0_RxTail); /* Return 0 (FALSE) if the receive buffer is empty */
}

// -------------------------------------------------------------------------------
// Pull 1 byte from Ring Buffer of bytes received from USART

unsigned char UART0_ring_buf_byte(void) {
  unsigned char tmptail;

  while (UART0_RxHead == UART0_RxTail) /* Wait for incoming data */
    ;
  tmptail =
      (UART0_RxTail + 1) & UART0_RX_BUFFER_MASK; /* Calculate buffer index */

  UART0_RxTail = tmptail;      /* Store new index */
  return UART0_RxBuf[tmptail]; /* Return data */
}

// ----------------------------------------------------

void UART0_send_byte(unsigned char data) {
  unsigned char tmphead;
  /* Calculate buffer index */
  tmphead = (UART0_TxHead + 1) &
            UART0_TX_BUFFER_MASK; /* Wait for free space in buffer */
  while (tmphead == UART0_TxTail)
    ;

  UART0_TxBuf[tmphead] = data; /* Store data in buffer */
  UART0_TxHead = tmphead;      /* Store new index */

  UCSR0B |= (1 << UDRIE0); /* Enable UDRE interrupt */
}
