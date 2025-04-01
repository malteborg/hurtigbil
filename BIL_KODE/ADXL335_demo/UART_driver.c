//**************************  UART_driver.c  ************************************
// Inspiration til_hvordan en UART driver kan_bygges_op
#include "UART_driver.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//== ISR USART_RXC_vect ========================================================
// Read a ch from the UART - interrupt
ISR(USART_RXC_vect)
{	for (int i=0;i<8;i++) inbuf[i]=inbuf[i+1];  // move the FIFO que forward
	inbuf[7] = UDR;                             // Place the UART ch in que
	if (inbuf[7]==CR) CRflag=1;   // if its CR set the flag
    if (incnt<8) incnt++;         // If FIFO que not full - then increase counter
}
//--------- r e a d ( ) ----------------------------------------------------------
// Read a ch from FIFO buffer
// in-counter = incnt holds the number of chars in the FIFO que
// If the que is empty - 0 zero will be returned
char read()
{	if (incnt)                       // If the que holds characters
		return inbuf[ 8-incnt--];    // Return the first char in the que
	else                             // NOTE how the counter will be decreased
		return 0;  // An empty que => 0 zero will be returned
}

//------- r e a d i n t ( ) -------------------------------------------------------
// Read integer + command ... should be called when "CRflag" is set
// The purpose of the function is to receive commands like: A123<CR>
// A is the command and will be placed in "command"
// The number 123 will be stored (binary) in "indtal" and returned
// "CRflag" will be reset  => command been read
// "incnt"  will be clear  => FIFO que is empty 
int readint()
{   int ix=7;
	// Search the FIFO que backwards for the Command
    while (( ((inbuf[ix]>='0')&&(inbuf[ix]<='9'))
	        || inbuf[ix]=='-' || inbuf[ix]==CR  )&&(ix>0)) ix--;
    command = inbuf[ix];           // Command found
	indtal  = atoi(&inbuf[ix+1]);  // Convert ASCII string to integer
	CRflag = 0;
	incnt  = 0;
	return indtal;
}

//==== ISR USART_UDRE_vect ========================================================
// FIFO out read (interrupt routine)
// This interrupt routine reads data from the FIFO out buffer and write the char
// to the UART Data Reg.
// when the buffer is empty must UDRE interrupt be disabled (why?)
ISR(USART_UDRE_vect)
{	if (FIFO_out.count > 0)   // if there are chars in the buffer for transmitting
	{   FIFO_out.count--;        // decrement count
		UDR = FIFO_out.buffer[ FIFO_out.outx++]; // Put a char in transmit reg.
		                     //FIFO_out.outx++ increment the index pointer
		if (FIFO_out.outx >= MAXoutBUF)          // if index at the end of buffer
		    FIFO_out.outx=0;		             // set index back to 0
	}
	if( FIFO_out.count==0)
	{	UCSRB &= ~(1<<UDRIE);  // Empty buffer => disable UDRE interrupt
	}
}

//------- w r i t e ( ch ) ------------------------------------------------------
// FIFO out write  .. subroutine
// This subroutine reads a char from the FIFO in buffer. (count=0 => ch=255)
// The values of count and out-index = outx are adjusted.
// The UDRE interrupt will be enabled ..
void write( char ch)
{	while (FIFO_out.count==MAXoutBUF);    // Wait doing nothing (bad)
    /////////////// Now theres space for at least one char //////////////
	UCSRB &= ~(1<<UDRIE);                // Disable Transmitter Interrupt
	FIFO_out.count++;                    // Increment without interrupt
	FIFO_out.buffer[ FIFO_out.inx++]=ch; // White char to buffer
	              // FIFO_out.inx++ Increment in-index
	if (FIFO_out.inx >= MAXoutBUF)
		FIFO_out.inx=0;                  // Start from zero again
	UCSRB |= (1<<UDRIE);         // Enable Transmitter Interrupt
}

//-------- w r i t e CR LF ( ch) -----------------------------------------------
// write char + CR + LF  .. subroutine
void writeCRLF( char ch)
{	if (ch) write( ch); //Note- you cant sent a binary zero - use write(ch) instead
	write( CR);
	write( LF);
}

//---------w r i t e s t r ( str, CR+LF )----------------------------------------
// FIFO out write  .. subroutine
void writestr( char str[], char CRLF)
{   unsigned char index=0;
	while( str[index])        // 0=zero must end a string
	{	write(str[index++]);
	}
	if (CRLF==CR)          // If only CR
		write(CR);
	else if (CRLF==LF)     // if only LF
		write(LF);
	else if (CRLF==CR+LF)  // if both CR and LF
	{   write(CR);
		write(LF);
	}
}

//-------------------------------------------------------------------------------------
// convert a byte to a text string - binary value (note! four return strings)
char *bin( unsigned char byte)
{	static int x=0;
	static char str[4][9];  // Static - important in order to keep the string
	x=(x+1)&3;              // change the string
	str[x][8]=0;                         // The String must end with 0
	for (char i=0; i<8; i++)
	   str[x][7-i] = (byte&(1<<i)?'1':'0'); // Convert byte to '1' and '0'
	return str[x];
}
//-------------------------------------------------------------------------------------
// convert a byte to a text string - binary value (note! two return strings)
char *floatstr(float tal)
{	static int x=0;
	static char str[2][12];  // Static - important in order to keep the string
	x=(x+1)&1;               // change the string
	//width = (width>24)?24:width;
	if( tal>0)
		sprintf(str[x], " %d.%04d",(int)tal,(int)((tal-(int)tal)*10000));
	else
	{   tal = -tal;
		sprintf(str[x], "-%d.%04d",(int)tal,(int)((tal-(int)tal)*10000));
	}
	return str[x];
}
//-----------------------------------------------------------------------------
// This subroutine will initialize the FIFO buffers and the UART registers
void Init_Serial_Port_and_FIFO(long USART_BAUDRATE)
{   incnt  =0;
	CRflag =0;
	inbuf[8]=0;
	FIFO_out.count = 0;
	FIFO_out.inx  = 0;
	FIFO_out.outx = 0;
		
	if (U2x==2)	UCSRA  = 0x02;    // U2x=2 => Double speed
	else        UCSRA  = 0x00;    // U2x=1 => Single speed
	UCSRB  = 0x18;                // async_mode, 8 data, 1 stop, no parity
	// Enable Transmitter with TXEN=1, but no need for interrupt yet
	UCSRB |= (1<<TXEN); // |(1<<UDRIE)
	// Enable Receiver with RXEN=1 and the interrupt with RXCIE=1
	UCSRB |= (1<<RXEN)|(1<<RXCIE);
	// URSEL=1 to be able to write UCSRC
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); // 8 data bit
	UBRRL = BAUD_PRESCALE;			   /* Load lower 8-bits of the baud rate */
	UBRRH = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
}