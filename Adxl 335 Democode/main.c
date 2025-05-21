/****************** ADXL335_demo ************************************************
 * Created: 11-02-2025 14:24:22 - Author: JJ
 */ 
#include "UART_driver.h"
#include "UART_driver.c"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <util/delay.h>
#include "GForce.h"


#define baudRate 9600
// Define constants for SwingState values
#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2
void Port_init();            // Prototype til funktionen ...
void Init_ADCxyz(uint8_t);   // Prototype til funktionen ...
void ADXL335_logger();       // Prototype til funktionen ...
void PWM_init();
void Timer1_init();

void INITuart(void);
void INITadc(void);
int maaleBaneFunction(void);
void INITSwingDetector(void);
void INITexternalInterrupt(void);
int SwingDetector(int y);

int  X,Y,Z;                  // The X,Y,Z values from the Acc-meter
char CR_LF = 1;              // True => CR + LF,  False => CR only

/////////////////////// m a i n ( )  /////////////////////////////////////////
// Note the program uses interrupt and state machines - hence will it never
// stop at wait for something to happen.
int main(void)
{
    char ch;
    char EchoVar[8];
    uint8_t MainState = 0;
    uint8_t setSpeed = 0;         // set-point for desired speed, starts at 0
    uint8_t CMD_Latch;
    uint8_t ReadDone;
	int SwingState = STRAIGHT;			//State of swing, LEFT, RIGHT, STRAIGHT
	int Old_state = 0;			// logically OLD state
	


    PWM_init();
	Timer1_init();
    Port_init();
    Init_Serial_Port_and_FIFO(9600);  // 9600, 14400, 19200, 38400, 57600, 115200 standards
    INITSwingDetector();
    //INITadc();
    //INITexternalInterrupt();
	
	sei();                            // Enable Global Interrupt
    writestr("Tryk z for speed control.", CR+LF);
	writestr("Tryk d for accelerometer logger.", CR+LF);
	writestr("Tryk b for brems.", CR+LF);
    writestr("", CR+LF);
    //======================================================================
	


    
    while (1) {
		
		

		
		// Læs et tegn fra UART	
		if (ch = read()){
			
	    // Tjek for gyldige kommandoer, ellers spring iterationen over
			if (ch==CR) {// If its a CR
				CR_LF = !CR_LF;     // Toggle the CR_LF flag
			} else if (ch == 'z') {
				MainState = 1;
			} else if (ch == 'd') {
				MainState = 2;
			} else if (ch == 'b') {
				MainState = 3;
			} else if (ch == 'a') {
				MainState = 4;
			} else if (ch == 'm') {
			MainState = 5;
		}
		
		
		}
	    // Switch-case, der udfører handling baseret på den modtagne kommando
	    switch (MainState) {
			 // SET SPEED
			    case 1:{
				    CMD_Latch = 1;
				    writestr("Input hastighed i 10%:", CR+LF);

			    while(CMD_Latch == 1) {
				    int temp = readint();
					setSpeed = temp;
					if(readint()){
						writestr("Speed set", CR+LF);
						int temp2 =(255/10)*setSpeed;
						writestr(temp2, CR+LF);
					CMD_Latch = 0;
					}
				}
				OCR2 = (255 / 10) * setSpeed; 
				
				writestr("");
			    MainState = 0;
				break;
				}
		    case 2: {
			    writestr("ADC maaling start", CR+LF);
			    writestr("X= Y= Z= ", CR+LF);
			    while (1) {
				    ADXL335_logger();
					
/*
				    if (FIFO_out.count <= 2) {
					    writestr("Buffer underflow - data tabt", CR+LF);
				    }*/
					
				    // Afslut måling, hvis der modtages 'd' igen
				    if (read() == 'd') {
					    MainState = 0;
						break;
				    }
			    }
				MainState = 0;
			    break;
		    }
		    
		    case 3: {
			    writestr("Brems", CR+LF);
			    PORTC = 0b00000010;
			    _delay_ms(1000);
			    PORTC = 0b00000000;
			    writestr("Koer", CR+LF);
			    MainState = 0;
				break;
		    }
			
			case 4: {
				writestr("NITRO", CR+LF);
				PORTB = 0b00000010;
				_delay_ms(100);
				PORTB = 0b00000000;
				writestr("nitro done", CR+LF);
				MainState = 0;
				break;
				}
			case 5: {
				while(1){
					writestr("Maaling af bane", CR+LF);
					UDR = SwingState;
					UDR = ADCH;
					
					//SwingState = SwingDetector(ADCH);
					if (Old_state != SwingState){
						Old_state =	SwingState;
						writestr(SwingState,CR+LF);
					}
				if (read() == 'm') {
					MainState = 0;
					writestr("Out of loop", CR+LF);
					break;
				}
				writestr("Out of loop", CR+LF);
				break;
				MainState = 0;
				break;
			}
				
			} // end switch
    } // end while

} // end main

//------- A D X L 3 3 5 _ l o g g e r () -------------------------------------------------
// Background: PA1=Z PA2=Y and PA3=X
// This function gets the Z, Y, X values from the acc_meter - by using a "state-machine"
// state holds the "State" and when state=4 the values will be displayed via a UART connection
// Press <CR> to toggle the CR_LF flag
void ADXL335_logger()
{
    static int state = 0;  // Note! Static variables keep the value even after return
    int Zpos;
	
    switch (state)
    {   
        case 0: {
            Init_ADCxyz(0); // Set Mux to PA1 and start ADC
            state = 1; 
            break;
        }
        case 1: {
            if (ADCSRA & 0x10)    // if ADC done
            {   
                Z = ADC;          // Read a 10bit value
                Init_ADCxyz(2);   // Set Mux to PA2 and start ADC
                state = 2;
            } 
            break;
        }
        case 2: {
            if (ADCSRA & 0x10)     // if ADC done
            {   
                Y = ADC;           // Read a 10bit value
                Init_ADCxyz(3);    // Set Mux to PA3 and start ADC
                state = 3;
            }   
            break;
        }
        case 3: {
            if (ADCSRA & 0x10)    // if ADC done
            {   
                X = ADC;          // Read a 10bit value   
                Init_ADCxyz(1);   // Set Mux to PA1 and start ADC        
                state = 4;        // Next state writes values on screen
            }  
            break;
        }
        case 4: {
            if (FIFO_out.count == 0) // "Wait" if the FIFO buffer is not empty
            {    
                sprintf(str, "X: %3d Y: %3d Z: %3d", X/4, Y/4, Z/4);
                /*
                str[(84-15)*4/7] = ':'; 
                str[ 84*4/7]     = '|'; 
                str[(84+18)*4/7] = ':';
                
                // The next statements place x,y,z due to the values they hold
                str[ X/7 < 80 ? X/7 : 79] = 'x';  // if X/7 is less than 80 use this else use 79
                str[ Y/7 < 80 ? Y/7 : 79] = 'y';
                
                // Alternative way to handle the z value (just for demo)
                if (Z/7 < 80)
                    Zpos = Z/7;
                else
                    Zpos = 79;
                str[ Zpos] = 'z';
                */
                if (CR_LF)
                    writestr(str, CR+LF);
                else
                    writestr(str, CR);
                state = 1;
            }
            break;
        }
    } // end switch
} // end ADXL335_logger

//------------- I n i t _A D C x y z () ------------------------------------------------
void Init_ADCxyz(uint8_t channel)
{   
    // Right adjust result and use AREF as reference
    ADMUX  = (channel & 0x07);  // ADCx and x = channel 0..7, Creates mask for ADC ports
    ADCSRA  = 0b11010100;    // Start ADC, prescaler = 16 
    
} // end Init_ADCxyz

//----------------- P o r t _ i n i t () -----------------------------------------------
void Port_init()
{
    DDRD  = DDRD & ~(0x44);   // setup PORTD, bit6 and bit2 as input
    DDRC  = 0xFF;                // PORTC as output
    PORTC = 0x00;             // With internal pull-up resistors
    DDRB  = 0xFF;             // setup PORTB as output
    PORTB = 0xFF;             // and turn LEDs off
    DDRA  = 0x00;             // setup PORTA as input
    
    //------------------ Default setup for ADC - overruled with Init_ADCxyz() ------
    ADMUX = 0x20;             // Select ADC0, Left adjust result and use AREF as reference
    SFIOR &= 0b00011111;      // Select Free Running mode by clearing bit [7-5] in SFIOR register
    //ADCSRA = 0xE4;            // Start ADC, Auto trigger enabled, pre_scale = 16
	ADCSRA = (1<<ADIE)|(1<<ADEN)|(1<<ADSC)|(1<<ADPS2); //ADC enable, ADC Start conversion, ADC Interrupt enable, Prescale = 4.
	writestr("Ports initialzed", CR+LF);
} // end Port_init

void PWM_init()
{
	// Set PD7 (OC2) as output
	DDRD |= (1 << PD7);

	// Configure Timer/Counter2 for Fast PWM mode
	TCCR2 |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
	TCCR2 |= (1 << COM21);                // Clear OC2 on compare match, set at BOTTOM
	TCCR2 |= (1 << CS21);                 // Set prescaler to 8 (start the timer)
	writestr("PWM initialzed", CR+LF);
	// Set initial duty cycle (e.g., 50%)
	//OCR2 = 128; // 50% duty cycle (128/255)
}

// Initialize Timer1
void Timer1_init() {
	TCCR1A = 0x00; // Normal mode
	TCCR1B = (1 << CS11); // Prescaler = 8
	TCNT1 = 0; // Initialize counter to 0
}

void INITuart(void){
	DDRB = 0xFF; //set PORTB as output.
	UBRRH = 0;
	UBRRL = F_CPU/16*(baudRate)-1;		//Set baudrate to 9600
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); // RX Complete interrupt enable, Reciever enable, Transmitter enable.
	
}
void INITadc(void){
	DDRA = 0; //PORT A as input
	PORTA = 0xFF; //internal resistor ON.
	ADMUX = (1<<ADLAR); //AREF internal Vref turned off, Left adj, channel 0.
	ADCSRA = (1<<ADIE)|(1<<ADEN)|(1<<ADSC)|(1<<ADPS1); //ADC enable, ADC Start conversion, ADC Interrupt enable, Prescale = 4.
}

void INITexternalInterrupt(void){
	MCUCR = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00); //Rising edge interrupt detection on EXTERNAL INTERRUPT 1 and 0
	GICR = (1<<INT0)|(1<<INT1);
}


int maaleBaneFunction(void){
	int SwingState;
	int bane[50];
	int baneIndex = 0;
	int Old_state;
	int Afstand = 0;
	
	bane[baneIndex++] = Afstand;
	
	Old_state = SwingState;
	
	while (1){
		if (Old_state != SwingState){
			Old_state =	SwingState;
			bane[baneIndex++] = Afstand;
		}
		
	}
	
	return bane[50];
}

void INITSwingDetector(void){
	static char SwingState = STRAIGHT;
	static int ACCerror = 10;
	static int SetpointLEFTturn = 220;
	static int SetpointRIGHTturn = 100;
	static int SetpointSTRAIGHT = 130;
	
}

int SwingDetector(int y){
	static int SwingState;
	static int ACCerror = 0x0A;			//dec = 10
	static int SetpointRIGHTturn = 100; //dec = 70
	static int SetpointLEFTturn = 155; //dec = 240
	static int SetpointSTRAIGHT = 0x7F; //dec = 127
	
	switch (SwingState){
		case STRAIGHT:
		if (y > (SetpointLEFTturn)){
			SwingState = LEFT;
			;
		}
		if (y < (SetpointSTRAIGHT - SetpointRIGHTturn)){
			SwingState = RIGHT;
			
		}
		break;
		
		
		case LEFT:
		if (y < (SetpointLEFTturn-ACCerror)){
			SwingState = STRAIGHT;
		}
		break;
		
		case RIGHT:
		if (y > (SetpointSTRAIGHT-SetpointRIGHTturn+ACCerror)){
			SwingState = STRAIGHT;
		}
		break;
		
	}
	return SwingState;
}

ISR(ADC_vect){
	
	volatile int ACCvalue = ADCH;
	writestr(ADCH, CR+LF);
	
	ADCSRA |= (1<<ADSC); // Start conversion.
}