
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 8000000UL
#define baudRate 9600
// Define constants for SwingState values
#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2

void portsetup(void);
void INITuart(void);
void INITadc(void);
int maaleBaneFunction(void);
void INITexternalInterrupt(void);
int SwingDetector(int y);
void PWM_init();
void brems(void);
void sendString(const char*);
void uarttest(void);
int ReadFromUART(char charRead);
int readint();
void setpointsViaUART(void);

volatile int ACCvalue = 0;
volatile int SwingState = STRAIGHT;
volatile int setpointPWM = 0;
volatile int readIntCMD;

volatile int SetpointRIGHTturn = 5; //dec = 70
volatile int SetpointLEFTturn = 195; //dec = 240
volatile int SetpointSTRAIGHT = 100; //dec = 127


void portsetup()
{
	
	DDRA = 0; //PORT A as input
	PORTA = 0x00; //internal resistor off. because of ADC
	
	DDRB = 0xFF; //set PORTB as output.
	
	DDRC = 0xff;   // Set portC to output mode
	
	DDRD = 0; // set port D to input
	PORTD = 0xff; // Set internal resistor to ON.
	
	
	
}

int main(void)
{
	
	sei();
	portsetup();
	INITuart();
	INITadc();
	INITexternalInterrupt();
	PWM_init();
	
	//==============
	_delay_ms(3000);
	setpointPWM = 0;
	OCR2 = setpointPWM;
	PORTB = 0xFF;
	
	
	
	
    while (1) 
    {
		
		OCR2 = (255/100)*setpointPWM;

		int Old_state = SwingState;		
			
		SwingState = SwingDetector(ACCvalue);
		
	
		if (Old_state != SwingState){
			Old_state =	SwingState;
			brems();
			

			if(SwingState == STRAIGHT){
				sendString("Straight");
				
				
			}
								
			else if(SwingState == RIGHT){
				sendString("Right");
				//PORTB = 0x0F;
			}
								
								
			else if(SwingState == LEFT){
				sendString("Left");
				//PORTB = 0xF0;
			}
		}
			
		

		
    }
}


void INITuart(void){
	int cpuFselect = 16; //selects what the baudrate register should be set as
	UBRRH = 0;
	//UBRRL = (F_CPU/(16*(baudRate))-1);		//Set baudrate to 9600
	if (F_CPU == 16000000UL){
		UBRRL = 104; //16MHz
	}
	else if (F_CPU == 8000000UL){
			UBRRL = 52; //8MH
	}

	
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); // RX Complete interrupt enable, Reciever enable, Transmitter enable.
	
}
void INITadc(void){
	DDRA = 0; //PORT A as input
	PORTA = 0x00; //internal resistor off.
	ADMUX = (1<<ADLAR);//|(1<<MUX0) //AREF internal Vref turned off, Left adj, channel 0.
	ADCSRA = (1<<ADIE)|(1<<ADEN)|(1<<ADSC)|(1<<ADPS2); //ADC enable, ADC Start conversion, ADC Interrupt enable, Prescale = 4.
}

void INITexternalInterrupt(void){
	MCUCR = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(0<<ISC00); //Rising edge interrupt detection on EXTERNAL INTERRUPT 1 and 0
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


int SwingDetector(int y){
	
	static int ACCerror = 0x00;
	
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

void setpointsViaUART(){
	while(1){
		int TemporarySetpoint = readint();
		
		if (readIntCMD == 'R'){
			SetpointRIGHTturn = TemporarySetpoint;
			break;
		}
		if (readIntCMD == 'L'){
			SetpointRIGHTturn = TemporarySetpoint;
			break;
		}
		if (readIntCMD == 'S'){
			SetpointRIGHTturn = TemporarySetpoint;
			break;
		}
	}
}



void PWM_init()
{
	// Set PD7 (OC2) as output
	DDRD |= (1 << PD7);

	// Configure Timer/Counter2 for Fast PWM mode
	TCCR2 |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
	TCCR2 |= (1 << COM21);                // Clear OC2 on compare match, set at BOTTOM
	TCCR2 |= (1 << CS21);                 // Set prescaler to 8 (start the timer)
	
	// Set initial duty cycle (e.g., 50%)
	//OCR2 = 128; // 50% duty cycle (128/255)
}

void brems(){
	sendString("BREEEEEMS");
	PORTC = 0b00000010;
	PORTB = 0b00000010;
	_delay_ms(1000);
	PORTC = 0b00000000;
	PORTB = 0b00000000;
}

void sendString(const char* inputString) {
	uint8_t i = 0;

	// Send the input string
	for (; inputString[i] != '\0'; i++) {
		while (!(UCSRA & (1 << UDRE))); // Wait if UART Output Data Register is not empty
		UDR = inputString[i]; // Send data out
	}

	// Append a newline character
	while (!(UCSRA & (1 << UDRE))); // Wait if UART Output Data Register is not empty
	UDR = '\n'; // Send newline character
}

void uarttest(){
		uint8_t name[] = "Gylfi Karlsson \n";
		for(uint8_t i = 0; i < sizeof(name); i++)
		{
			while (!(UCSRA & (1<<UDRE)) );  //wait if UART Output Data Register is not empt
			
			UDR = name[i]; //Send data out.
		}
}

int readint(){   
	int inbuf[9];
	int ix=7;
	// Search the FIFO que backwards for the Command
	while (( ((inbuf[ix]>='0')&&(inbuf[ix]<='9'))
	|| inbuf[ix]=='-' || inbuf[ix]==13  )&&(ix>0)) ix--;
	readIntCMD = inbuf[ix];           // Command found
	int indtal  = atoi(&inbuf[ix+1]);  // Convert ASCII string to integer
	return indtal;
}

ISR(ADC_vect){
	
	ACCvalue = ADCH;
	
	ADCSRA |= (1<<ADSC); // Start conversion.
}

ISR(USART_RXC_vect){
	
	setpointPWM = UDR;
	sendString("recieved PWM");
	

}

ISR(INT0_vect){
	sendString("maalstreg");
	brems();
}

