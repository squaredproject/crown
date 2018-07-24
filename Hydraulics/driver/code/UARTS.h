// ---------------------------------------------------------------------------
// UART1.h
// Routines for interrupt controlled USART for RS-232 communication.
// Last modified: 9-Apr-07
// UART baud rate defs added by Jon
// ---------------------------------------------------------------------------

// uart_Init() speed defs from Atmel ATmega88 manual, pp. 197--199

// assumes u2X = 1

#define F_CPU 16000000               		// 16MHz processor


#if  F_CPU == 20000000               		// 20 MHz processor
#warning "UART1: using 20 mHz clock"
#define UART_9600    129
#define UART_14400   86
#define UART_19200   64
#define UART_28800   42
#define UART_38400   32
#define UART_57600   21
#define UART_76800   15
#define UART_115200  10
#endif

#if  F_CPU == 18432000               		// 18.4MHz processor
#warning "UART1: using 18.4 mHz clock"
#define UART_9600    119
#define UART_14400   79
#define UART_19200   59
#define UART_28800   39
#define UART_38400   29
#define UART_57600   19
#define UART_76800   14
#define UART_115200  9
#endif


#if  F_CPU == 16000000               		// 16MHz processor
#warning "UART1: using 16 mHz clock"
#define UART_9600    103
#define UART_14400   68
#define UART_19200   51
#define UART_28800   34
#define UART_38400   25
#define UART_57600   16
#define UART_76800   12
#define UART_115200  8
#endif



#if  F_CPU == 14745000               		// 14.745MHz processor
#warning "UART1: using 14.754 mHz clock"
#define UART_9600    95
#define UART_14400   63
#define UART_19200   47
#define UART_28800   31
#define UART_38400   23
#define UART_57600   15
#define UART_76800   11
#define UART_115200  7
#endif


#if  F_CPU == 8000000               		// 8MHz processor
#warning "UART1: using 8 mHz clock"
#define UART_9600    51
#define UART_14400   34
#define UART_19200   25
#define UART_28800   16
#define UART_38400   12
#define UART_57600   8
#define UART_76800   6
#define UART_115200  3
#endif


#if  F_CPU == 7372800               		// 7.37MHz processor
#warning "UART1: using 7.37 mHz clock"
#define UART_9600    47
#define UART_14400   31
#define UART_19200   23
#define UART_28800   15
#define UART_38400   11
#define UART_57600   7
#define UART_76800   5
#define UART_115200  3
#endif


#if F_CPU == 3686400               		// 3.69MHz processor
#warning "UART1: using 3.69 mHz clock"
#define UART_9600    23
#define UART_14400   15
#define UART_19200   11
#define UART_28800   7
#define UART_38400   5

#endif

void UART0_Init( unsigned int baud );
void UART0_send_byte( unsigned char data );

unsigned char UART0_ring_buf_byte( void );
unsigned char UART0_data_in_ring_buf( void );


void UART1_Init( unsigned int baud );
void UART1_send_byte( unsigned char data );

unsigned char UART1_ring_buf_byte( void );
unsigned char UART1_data_in_ring_buf( void );

