// -------------------------------------------------------------------------------
// UART0.h
// Routines for interrupt controlled UART
// rewritten for atmegaX8 from atmega8 by jtf July 2008
//
// rewritten for atmega2560 and file streams MAP April 2012
// -------------------------------------------------------------------------------

void UART0_Init( unsigned int baud );
void UART0_send_byte( unsigned char data );
char UART0_ring_buf_byte( void );
unsigned char UART0_data_in_ring_buf( void );
extern FILE uart0file;





