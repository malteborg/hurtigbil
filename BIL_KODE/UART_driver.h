//********************** UART_driver.h *******************************
// Inspiration til_hvordan en UART driver kan_bygges_op
//
#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#define F_CPU 2000000UL     // MUST BE ADJUSTED TO FIT THE KIT

#include <avr/io.h>
#include <avr/interrupt.h>

#define U2x 2               // U2x must be 1 or 2
#define BAUD_PRESCALE ((((F_CPU*U2x)/(USART_BAUDRATE * 16UL)))-1)

#define CR 13
#define LF 10
#define ESCAPE 27

char str[100];   // Tekststreng_som kan_bruges_til sprintf( str, ..

char CRflag;      //CRflag=1 => der er tastet Return
char inbuf[9];    //FIFO in buffer med plads_til 8 ch + zero
int  incnt;       //In buffer counter = antal_ch i FIFO
char command;     // Kommandodelen_af input f.eks A
int  indtal;      // Taldelen_af en_kommando - f.eks. 123

// MAX value for can be: MAXinBUF>255 (depends on RAM aviable)
#define MAXoutBUF 150
struct FIFO_out_buffer
{	unsigned int count;      // number of chars in buffer
	unsigned int inx, outx;  // pointers (indexes) for buffer
	char buffer[MAXoutBUF];  // the FIFO buffer
} FIFO_out;

void  Init_Serial_Port_and_FIFO(long);
char  read();
int   readint();
void  write( char);
void  writestr( char[], char);
void  writeCRLF( char);

char *bin( unsigned char);  // Convert a byte to a string of '1' and '0'
char *floatstr(float tal);

#endif /* UART_DRIVER_H_ */