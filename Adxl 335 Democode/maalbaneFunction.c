/*
 * maaleBaneFunction.c
 *
 * Created: 30/04/2025 20:46:05
 * Author : gylfi
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 8000000UL
#define baudRate 9600
// Define constants for SwingState values
#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2

void INITuart(void);
void INITadc(void);
int maaleBaneFunction(void);
void INITSwingDetector(void);
void INITexternalInterrupt(void);
int SwingDetector(int y);


int main(void)
{

	volatile int ACCvalue = 0;
	volatile int SwingState = STRAIGHT;
	int bane[50];
	
	
	sei();
	INITuart();
	INITSwingDetector();
	INITadc();
	INITexternalInterrupt();
	
	
	bane[50] = maaleBaneFunction();
	
    while (1) 
    {
		
	
		SwingState = SwingDetector(ADCH);
		UDR = SwingState;
		PORTB = 0xFF;
		_delay_ms(2000);
	
		PORTB = 0;
		_delay_ms(2000);
    }
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
	static char SwingState;
	static int ACCerror = 10;
	static int SetpointLEFTturn = 220;
	static int SetpointRIGHTturn = 100;
	static int SetpointSTRAIGHT = 130; 
	
}

int SwingDetector(int y){
	static int SwingState;
	static int ACCerror = 0x0A;
	static int SetpointRIGHTturn = 0x46; //dec = 70
	static int SetpointLEFTturn = 0xF0; //dec = 240
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
	
	ADCSRA |= (1<<ADSC); // Start conversion.
}

ISR(INT0_vect){
		writestr("Brems", CR+LF);
		PORTC = 0b00000010;
		_delay_ms(1000);
		PORTC = 0b00000000;
		writestr("Koer", CR+LF);
		MainState = 0;
	
}