/****************** ADXL335_demo ************************************************
 * Created: 11-02-2025 14:24:22 - Author: JJ
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "UART_driver.h"
#include "DriveTheCarFunctions.h"



void Port_init();            // Prototype til_funktionen ...
void Init_ADCxyz( uint8_t);  // Prototype til_funktionen ...
void ADXL335_logger();       // Prototype til_funktionen ...
void PWM_init();



int  X,Y,Z;                 // The X,Y;Z values from the Acc-meter
char CR_LF = 1;             // True => CR + LF,  False => CR only

/////////////////////// m a i n ( )  /////////////////////////////////////////
// Note the program uses interrupt and state machines - hence will it never
// stop at wait for something to happen.
int main(void)
{   char ch;
	char EchoVar[8];
	uint8_t MainState =1;
	uint8_t setSpeed = 0;			//set-point for desired speed, starts at 0
	uint8_t CMD_Latch;
	uint8_t ReadDone;
	


	

    PWM_init();
	Port_init();
	Init_Serial_Port_and_FIFO( 9600);  //9600, 14400, 19200, 38400, 57600, 115200 standards
	sei();	                           // Enable Global Interrupt
	writestr( "Tryk shift + Z for at indtast commando", CR+LF);
	writestr( "", CR+LF);	
	//======================================================================


			/*if( (ch=read() )){          // If a character received from the UART
			if (ch==CR)            // If its a CR
			   CR_LF = !CR_LF;     // Toggle the CR_LF flag
		}
		*/
	
	while (1){

		//ADXL335_logger();	
			//set up something to read commands.
			//set diodes up so that each states turn on specific diode.

		//PORTB =PINC;
		
		writestr("test", CR+LF);
		switch (MainState){
			case 1: if(UDR == 'Z'){
					CMD_Latch = 1; 
					writestr("Format ...A123...", CR+LF);
					}	
					
					while(CMD_Latch == 1){
					PORTB = 0xff;
					setSpeed = readint();
					char output = bin(setSpeed);
						if(setSpeed > 1){
							writestr( "write done", CR+LF);
							CMD_Latch = 0;
							writestr( output, CR+LF);
							PORTB = 0x00;
							
						}
					}
				

					MainState = 2;
					break;
				
			case 2:
					OCR2 = (255/10)*setSpeed; // calculates duty cycle from percentile set point
					//writestr(char output, CR+LF);
					MainState = 3;
					break;
			
			case 3: if(UDR == 'D') CMD_Latch = 1;
					
							
					if(CMD_Latch == 1){
						writestr("ADC maaling start", CR+LF);
						writestr("X= Y= Z= ", CR+LF);
						while(1){
							ADXL335_logger();
						}	
						if(FIFO_out.count <= 2){
							writestr("Buffer underflow - data tabt", CR+LF);
					}
					
					}
				MainState = 1;
				break;
				
			}
		
	}
    
}

//------- A D X L 3 3 5 _ l o g g e r () -------------------------------------------------
// Background: PA1=Z PA2=Y and PA3=X
// This function get the Z,Y,X value from the acc_meter - by using a "state-machine"
// state holds the "State" and when state=4 will the values be display via an UART connection
// Press <CR> to toggle the CR_LF flag
void ADXL335_logger()
{	static int state=0;  // Note! Static variables keep the value even after return
	int Zpos;
	
	switch (state)
	{	case  0: Init_ADCxyz(0); // Set Mux to PA1 and start ADC
		         state=1; 
				 break;
		case  1: if (ADCSRA&0x10)    // if ADC done
				 {   Z=ADC;          // Read a 10bit value
					 Init_ADCxyz(2); // Set Mux to PA2 and start ADC
					 state=2;
				 } break;
		case  2: if (ADCSRA&0x10)     // if ADC done
		         {   Y=ADC;           // Read a 10bit value
					 Init_ADCxyz(3);  // Set Mux to PA3 and start ADC
					 state=3;
				 }   break;
		case  3: if (ADCSRA&0x10)    // if ADC done
				 {   X=ADC;          // Read a 10bit value   
					 Init_ADCxyz(0); // Set Mux to PA1 and start ADC        
					 state=4;       // Next state writes values on screen
				  }  break;
		case  4:  if( FIFO_out.count==0) // "Wait" if the FIFO buffer not empty
		          // Division by 4 reduce the 10-bit value to a 8-bit
				  // The Acc-meter uses 3,3V reference and ATmega uses 5,0V
				  // Hence will the values be between 0 and 168 with 84 as average 
				  
				  {	 sprintf(str, "%3d %3d %3d",X/4, Y/4 ,Z/4,"\0");
					/*
					  str[(84-15)*4/7] = ':'; 
					  str[ 84*4/7]     = '|'; 
					  str[(84+18)*4/7] = ':';
					

			
					  
				
			
				  // The next statements place x,y,z due to the values they holds
					 str[ X/7<80 ? X/7: 79] = 'x';  //if X/7 less then 80 use this else use 79
					 str[ Y/7<80 ? Y/7: 79] = 'y';
					 
				   // Alternative way to handle the z value (just for demo)
					 if (Z/7<80) Zpos=Z/7; else Zpos=79;
					 str[ Zpos] = 'z';
				 */
					 if (CR_LF) writestr( str,CR+LF);
					 else       writestr( str,CR);
					 state=1;
				} break;
	}
}

//------------- I n i t _A D C x y z () ------------------------------------------------
void Init_ADCxyz( uint8_t channel)
{   //  Right adjust result and use AREF as reference
	ADMUX   = (channel & 0x07);   // ADCx and x = channel 0..7 Creates mask for ADC ports
	//ADCSRA  = 0b11010100;		  // Start ADC,  prescaler = 16 
	ADCSRA  = 0b11010100;
};	// Page 216 in ATmega32_doc2503.pdf documentation



//----------------- P o r t _ i n i t () -----------------------------------------------
void Port_init()
{	DDRD  = DDRD & ~(0x44);	 // setup PORTD, bit6 and bit2 as input
	DDRC  = 0;          // PORTC as input
	PORTC = 0xFF;       // With internal pull-up resistors
	DDRB  = 0xFF;		// setup PORTB as output
	PORTB = 0xFF;		// and turn LEDs off
	DDRA  = 0x00;		// setup PORTA as input
	
	//------------------ Default setup for ADC - overruled with Init_ADCxyz() ------
	
	ADMUX = 0x20;		// Select ADC0, Left adjust result and use AREF as reference
	SFIOR &= 0b00011111;// Select Free Running mode by clearing bit [7-5] in SFIOR register
	ADCSRA = 0xE4;		// Start ADC, Auto trigger enabled, pre_scale = 16
}


