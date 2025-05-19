


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define baudRate 9600
// Define constants for SwingState values
#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2
//========= Moving average filter========
#define ADC_BUFFER_SIZE 6 // Number of samples for the moving average

volatile uint16_t adcBuffer[ADC_BUFFER_SIZE]; // Buffer to store ADC values
volatile uint8_t adcBufferIndex = 0;          // Index to track the current position in the buffer
volatile uint16_t adcSum = 0;                 // Sum of all values in the buffer
volatile uint16_t filteredADCValue = 0;       // Filtered ADC value (moving average)
//========= function declerations======
void portsetup(void);
void INITuart(void);
void INITadc(void);
int maaleBaneFunction(void);
void INITexternalInterrupt(void);
int SwingDetector(int y);
void PWM_init();
void brems(void);
void sendString(const char*);
void sendStringNoNewLine(const char* inputString);
void uarttest(void);
int ReadFromUART(char charRead);
int containsChar(volatile char* buffer, char target, int bufferSize);
void sendInt(int number);
int stringToInt(const char* str);
void updateMovingAverage(uint16_t newValue);
void INITtimer0(void);
void accelerometer();
void maaleBaneFunctionMedTid(int antalSwing);


volatile int ACCvalue = 0;
volatile int SwingState = STRAIGHT;
volatile int setpointPWM = 0;
volatile int readIntCMD;
volatile int SetpointRIGHTturn = 70; //dec = 70
volatile int SetpointLEFTturn = 95; //dec = 240
volatile int SetpointSTRAIGHT = 90; //dec = 127
volatile char inputBuffer[8]; // circular buffer
volatile int UARTbufferIndex = 0; //circular buffer index
volatile int UARTrecievedLatch = 0; //Sets the latch high when the uart recieve interrupt is triggered so that the code can react.
volatile int receivedByte; // byte from the UART UDR register.
volatile char commandchar; // char received from the UART UDR.
volatile uint16_t msec = 0;	//millisecond counter, incremented every millisecond.
static int sampleTime = 50; // how many times the sampleTimeCounter needs to increment for the to be taken another accelerometer measurement.
static int sampleTimeCounter;
volatile int afstand = 0;
volatile int maalstregscounter; // counts how many times the car has driven over the finish line.
volatile int safetyFactor; // controls the regulation of speed each lap.




int main(void)
{
	
	sei();
	portsetup();
	INITuart();
	INITadc();
	INITexternalInterrupt();
	PWM_init();
	INITtimer0();
	
	//==============
	_delay_ms(3000);
	setpointPWM =  0;
	OCR2 = (255/100)*setpointPWM;
	PORTB = 0xFF;
	int mainstate = 1;
	int Old_state;
	
	
	
	sendString("initDone");
	
	
	
    while (1) 
    {
	

			
		switch(mainstate){
			case 1:{
				if (Old_state != SwingState)
					Old_state =	SwingState;	
					
				mainstate = 2;
				break;
			}
			case 2: if (containsChar(inputBuffer, 'm', 8)){
				sendString("maal bane function.");
				maaleBaneFunctionMedTid(4);
				mainstate = 3; 
				
				setpointPWM = 0;
				break;
			}
			case 3: if (containsChar(inputBuffer, 's',8)){
				sendString("started:");
				setpointPWM = 60;
				mainstate = 4;
				break;
			}

			case 4: if (containsChar(inputBuffer, 'k',8)){
				setpointPWM = 100;
				mainstate = 4;
				break;
			}
						
			case 5: if (containsChar(inputBuffer, 'b',8)){
				brems();
				mainstate = 1;
				break;
			}
				
		}
	}
}

void portsetup(){
	
	DDRA = 0; //PORT A as input
	PORTA = 0x00; //internal resistor off. because of ADC
	
	DDRB = 0xFF; //set PORTB as output.
	
	DDRC = 0xff;   // Set portC to output mode
	
	DDRD = 0; // set port D to input
	PORTD = 0xff; // Set internal resistor to ON.
	
	
	
}

void INITuart(void){
	int cpuFselect = 16; //selects what the baudrate register should be set as
	UBRRH = 0;
	//UBRRL = (F_CPU/(16*(baudRate))-1);		//Set baudrate to 9600
	#if (F_CPU == 16000000UL)
		UBRRL = 104; //16MHz
	
	#elif F_CPU == 8000000UL
			UBRRL = 52; //8MH
	
	#endif
	//UBRRL = 104; //temporary to fix UART problems.
	
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); // RX Complete interrupt enable, Reciever enable, Transmitter enable.
	
}

void INITadc(void){
	DDRA = 0; //PORT A as input
	PORTA = 0x00; //internal resistor off.
	ADMUX = (1<<ADLAR);//|(1<<MUX0) //AREF internal Vref turned off, Left adj, channel 0.
	//ADCSRA = (1<<ADIE)|(1<<ADEN)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //ADC enable, ADC Start conversion, ADC Interrupt enable, Prescale = 128.
	ADCSRA = (1<<ADATE)|(1<<ADEN)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //ADC enable, ADC Start conversion, ADC Auto Trigger enable, Prescale = 128.
	SFIOR = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0); //ADC set to "Free Running Mode"
	}

void INITexternalInterrupt(void){
	MCUCR = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00); //rising edge interrupt detection on EXTERNAL INTERRUPT 1 and 0
	GICR = (1<<INT0)|(1<<INT1);
}

void INITtimer0(){
	#if F_CPU == 16000000UL // init Timer 0 to count in 1 millisecond intervals @ 16MHz
		TCCR0 = (1<<CS01)|(1<<CS00);
		
		OCR0 = 249;
	#else					// init Timer 0 to count in 1 millisecond intervals @ 8MHz
		TCCR0 = (1<<CS01)|(1<<CS00);
		OCR0 = 124;
	#endif
	
	TIFR |= 1 << OCF0; //clear Timer 0 output compare flag
	TIMSK |= 1 << OCIE0; // enable Timer 0 output compare interupt.
}

int maaleBaneFunction(void){
	
	int bane[50];
	int baneIndex = 0;
	int Old_state;
	int setpointPWM = 50;
	
	_delay_ms(2000);
	maalstregscounter = 0;
	safetyFactor = 8;
	do {
	OCR2 = (255/100)*setpointPWM;
	
	} while(maalstregscounter == 0);
	
	afstand = 0;
	
	
	bane[baneIndex++] = afstand;
	
	
	
	do{
		if (maalstregscounter == 2)
			break;
		if (Old_state != SwingState){
			Old_state =	SwingState;
			bane[baneIndex++] = afstand;
		}
		
	}while(baneIndex < 50);
	
	sendString("Locations of swings:");
	for (int i = 1 ; i < 5; i++){
		sendStringNoNewLine("Turn:");
		sendInt(i);
		sendStringNoNewLine("Time:");
		sendInt(bane[i]);		
	}

	return bane[50];
}

void maaleBaneFunctionMedTid(int antalSwing){
	int baneIndex = 0;
	int bremseVar = 1000;
	int bane[2*antalSwing];
	int Old_state;
	

	
	maalstregscounter = 0;
	safetyFactor = 4;
	msec = 0;
	OCR2 = (255/100)*50;
	while(1){
	if (Old_state != SwingState){
		Old_state =	SwingState;
		baneIndex++;
		sendStringNoNewLine("bane index");
		sendInt(baneIndex - 1);
		sendStringNoNewLine("time:");
		sendInt(msec);
	}
	
	if (baneIndex == antalSwing){
		maalstregscounter++;
		baneIndex = 0;
		safetyFactor = safetyFactor/2;
		msec = 0;
		sendString("fake interrupt");
		sendString("");
		break;
	}
	}
	sendString("maalstregscounter:");
	sendInt(maalstregscounter);
	
	while (maalstregscounter == 1){
		if (Old_state != SwingState){
			Old_state =	SwingState;
			bane[baneIndex++] = msec;
			sendString("swing");
			sendStringNoNewLine("baneindex");
			sendInt(baneIndex-1);
		}
		if (baneIndex == antalSwing){
			maalstregscounter++;
			baneIndex = 0;
			safetyFactor = safetyFactor/2;
			msec = 0;
			sendString("fake interrupt");
			break;
		}
	}
	
	//===================

	
	sendString("sensing done:");
	
	sendString("Locations of swings:");
	for (int i = 1 ; i <= antalSwing; i++){
		sendStringNoNewLine("Turn:");
		sendInt(i);
		sendStringNoNewLine("Time:");
		sendInt(bane[i]);
	}
	OCR2 = 0;
	
	
	while(1){
		switch (SwingState){
			case STRAIGHT:{
				OCR2 = (255/100)*60; 
				if((msec+bremseVar) >= bane[baneIndex+1]){
					sendString("");
					sendStringNoNewLine("Time:");
					sendInt(bane[baneIndex]);
					brems();
					OCR2 = 100;
					baneIndex++;
				}
				break;
			}
			case LEFT: case RIGHT:{
				OCR2 = 100;
				break;
			}
		
		if (baneIndex == antalSwing){
			maalstregscounter++;
			baneIndex = 0;
			safetyFactor = safetyFactor/2;
			msec = 0;
			sendString("fake interrupt");
			break;
		}
		
		if (containsChar(inputBuffer, 's', 8))
			break;
	}
	}
 }

int SwingDetector(int y){
	
	static int ACCerror = 0x05;
	
	switch (SwingState){
		case STRAIGHT:
		if (y > (SetpointLEFTturn)){
			SwingState = LEFT;
/*
			sendString("");
			sendString("Left");
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("Time in ms:");
			sendInt(msec);*/
			
		}
		else if (y < (SetpointRIGHTturn)){
			SwingState = RIGHT;
/*
			sendString("");
			sendString("Right");
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("Time in ms:");
			sendInt(msec);*/
		}
		break;
		
		
		case LEFT:
		if (y < (SetpointLEFTturn-ACCerror)){
			SwingState = STRAIGHT;
/*
			sendString("");
			sendString("Straight");
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("Time in ms:");
			sendInt(msec);*/
			
		}
		break;
		
		case RIGHT:
		if (y > (SetpointRIGHTturn+ACCerror)){
			SwingState = STRAIGHT;
/*
			sendString("");
			sendString("Straight");
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("Time in ms:");
			sendInt(msec);*/
		}
		break;
		
	}
	return SwingState;
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

void sendStringNoNewLine(const char* inputString) {
	uint8_t i = 0;

	// Send the input string
	for (; inputString[i] != '\0'; i++) {
		while (!(UCSRA & (1 << UDRE))); // Wait if UART Output Data Register is not empty
		UDR = inputString[i]; // Send data out
	}
}

void uarttest(){
		uint8_t name[] = "Gylfi Karlsson \n";
		for(uint8_t i = 0; i < sizeof(name); i++)
		{
			while (!(UCSRA & (1<<UDRE)) );  //wait if UART Output Data Register is not empt
			
			UDR = name[i]; //Send data out.
		}
}

void sendInt(int number) {
	char buffer[10]; // Buffer to hold the string representation of the number
	itoa(number, buffer, 10); // Convert integer to string (base 10)
	
	// Send the string over UART
	for (uint8_t i = 0; buffer[i] != '\0'; i++) {
		while (!(UCSRA & (1 << UDRE))); // Wait if UART Output Data Register is not empty
		UDR = buffer[i]; // Send data out
	}

	// Append a newline character
	while (!(UCSRA & (1 << UDRE))); // Wait if UART Output Data Register is not empty
	UDR = '\n'; // Send newline character
}

int stringToInt(const char* str) {
	int result = 0;
	
	while (*str >= '0' && *str <= '9') {
		result = result * 10 + (*str - '0'); // Convert char to int and accumulate
		str++;
	}

	return result;
	}
	
//checks for char in the input buffer
int containsChar(volatile char* buffer, char target, int bufferSize) {
	for (int i = 0; i < bufferSize; i++) {
		if (buffer[i] == target) {
			return 1; // Character found
		}
	}
	return 0; // Character not found
}
	
// Function to update the moving average filter
void updateMovingAverage(uint16_t newValue) {
	// Subtract the oldest value from the sum
	adcSum -= adcBuffer[adcBufferIndex];

	// Add the new value to the sum
	adcBuffer[adcBufferIndex] = newValue;
	adcSum += newValue;

	// Update the buffer index (wrap around if necessary)
	adcBufferIndex = (adcBufferIndex + 1) % ADC_BUFFER_SIZE;

	// Calculate the moving average
	filteredADCValue = adcSum / ADC_BUFFER_SIZE;
}

void accelerometer(){
	
	if (--sampleTimeCounter > 0) return;
	sampleTimeCounter = sampleTime; 
	int newACCvalue = ADCH;
	updateMovingAverage(newACCvalue);
	ACCvalue = filteredADCValue;
	SwingState = SwingDetector(ACCvalue);
	
}

ISR(ADC_vect){
	
	//ADCSRA |= (1<<ADSC); // Start conversion.
}

ISR(USART_RXC_vect){
	
	receivedByte = UDR;
	inputBuffer[UARTbufferIndex] = receivedByte;
	UARTbufferIndex = (UARTbufferIndex + 1) % sizeof(inputBuffer);
	UARTrecievedLatch = 1;
	
	

}

ISR(INT0_vect){ //external interrupt PD2. maalstreg schmidtt 

	afstand++;
	sendInt(afstand);
}

ISR(INT1_vect){
	brems();
	
}

ISR(TIMER0_COMP_vect){
	
	msec++; 
	accelerometer();
	
	
	
}
