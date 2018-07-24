// -------------------------------------------------------------------------------
// UART2.h
// Routines for interrupt controlled UART
// rewritten for atmegaX8 from atmega8 by jtf July 2008
//
// rewritten for atmega2560 and file streams MAP April 2012
// -------------------------------------------------------------------------------


void UART2_Init( unsigned int baud );
void UART2_send_byte( unsigned char data );
char UART2_ring_buf_byte( void );
unsigned char UART2_data_in_ring_buf( void );
extern FILE uart2file;



