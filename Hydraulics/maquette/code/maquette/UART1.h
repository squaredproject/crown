// -------------------------------------------------------------------------------
// UART1.h
// Routines for interrupt controlled UART
// rewritten for atmegaX8 from atmega8 by jtf July 2008
//
// rewritten for atmega2560 and file streams MAP April 2012
// -------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

void UART1_Init( unsigned int baud );
void UART1_send_byte( unsigned char data );
char UART1_ring_buf_byte( void );
unsigned char UART1_data_in_ring_buf( void );
extern FILE uart1file;

#ifdef __cplusplus
}
#endif




