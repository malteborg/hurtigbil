
//--------
//Den her kode blev skrevet af gruppe 4,
//hvor en af medlemerne er en Islænding som ikke kan bestemme sig om han skal tænke på Islandsk, Dansk eller Engelsk.
// Derfor kan kommentar og variabler findes bægge på Engelsk og Dansk.

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>


#define baudRate 9600
// Define constants for SwingState values
#define STRAIGHT 0							// The state values that the car uses to sort sections of the track into LEFT, RIGHT, and STRAIGHT.
#define LEFT 1								//
#define RIGHT 2								//

#define SWING 4								// Is used when driving after the measured track to categorize which state the car is in at a given moment.
#define NOTSWING 5							//
#define afterBRAKE 6						//

//========= Moving average filter======
#define ADC_BUFFER_SIZE 8					// Number of samples for the moving average

volatile uint16_t adcBuffer[ADC_BUFFER_SIZE]; // Buffer to store ADC values
volatile uint8_t adcBufferIndex = 0;          // Index to track the current position in the buffer
volatile uint16_t adcSum = 0;                 // Sum of all values in the buffer
volatile uint16_t filteredADCValue = 0;       // Filtered ADC value (moving average)

//========= function declerations======

//--setup--
void portsetup(void);						// Sets up the I/O ports of the ATmega to the desired states of Input and Output.
void INITuart(void);						// Initializes and sets up the Registers associated with the UART Transmit and Recieve.
void INITadc(void);							// Initializes and sets up the Registers associated with the Analog Digital Conversion module.
void INITexternalInterrupt(void);			// Initializes and sets up the Registers associated with the External Interrupt pins.
void PWM_init();							// Initializes and sets up the Registers associated with Timer2 and the waveform generation for the PWM signal. 
void INITtimer0(void);						// Initializes and sets up the Registers associated with Timer0 and the 1mS "heartbeat" that is created by it.
void INITtimer1();							// Initializes and sets up the Registers associated with Timer1 and the "omdrejningstæller."

//--UART Communication-- 
void sendString(const char*);				// Serially sends out a string through the UART.
void sendStringNoNewLine(const char* inputString); //  Serially sends out a string through the UART with no newline char.
void sendInt(int number);					// Converts the integer into a char/string and sends it serially through the UART.			
void uarttest(void);						// Serially sends out an array of chars, to test if the UART is set up propperly.
int containsChar(volatile char* buffer, char target, int bufferSize); //checks the input buffer for a certain varible/char.
int stringToInt(const char* str);			// Converts a string to an integer, for more usability from the UART.
void clearUARTinputBuffer();					// Clears the input buffer.

//--Data Behandling--
void updateMovingAverage(uint16_t newValue);// Updates the moving average filter with a new value.
float calculateDistance(int sampleCount);	// Calculates the sample count to millimeters.
void calculateSpeedFunction(int dist2, int tid2);// Calculate the speed in m/S.
void calculateMovingAVGSpeedFunction(uint8_t newSpeedValue);// moving average filter for Speed.
void accelerometer();						// Takes a value from the accelerometer and ADC and puts its value into the moving average filter.
int SwingDetector(int y);					// Takes the value from the moving average filter and catagorises if it is a STRAIGHT, RIGHT or LEFT.

//-- Main functions--
void maaleBaneFunction(void);				// Maps the track, taking notes of the swings and maps them into the bane[50] array.
void DriveSuperFastFunction();				// Makes the car drive after the values mapped in the "maaleBaneFunction"

//-- Others --
void countersReset();						// Resets the counters used to keep track of data each lap.
void brems(void);							// Activates the relevant pins in PORTB to engage the brake logic. 

//=====================================


volatile int ACCvalue = 0;					// Value taken from the ADCH register and temporarily stored before being put into the moving average filter.
volatile int SwingState = STRAIGHT;			// Initializes that the car starts at a stand still, i.e on a straight, 
volatile int Old_state = STRAIGHT;			// Initializes that the car starts at a stand still, i.e on a straight, 
volatile int setpointPWM = 0;				// Initialization to not have a "floating value".
volatile int SetpointRIGHTturn = 81;		// Threshold used in the swing detector function to recognize Right from a straight.
volatile int SetpointLEFTturn  = 89;		// Threshold used in the swing detector function to recognize Left from a straight.
volatile int SetpointSTRAIGHT  = 85;		// The value that the car has when sitting still, (is not used in this code, only for reference)
volatile char UARTinputBuffer[8];			// Circular buffer for the input from the UART
volatile int UARTbufferIndex = 0;			// Circular buffer index
volatile int receivedByte;					// byte from the UART UDR register.
volatile uint16_t msec = 0;					// millisecond counter from timer0, incremented every millisecond.
static int sampleTime = 50;					// how many times the sampleTimeCounter needs to increment before another accelerometer measurement is taken.
static int sampleTimeCounter;				//
volatile uint16_t sampleCount = 0;			// Initialization to not have a "floating value".
volatile int bane[50] = {0};				// Array of the track that holds "samples" from the startline.
volatile uint16_t afstand = 0;				// calculated Distance, i.e samples that have been run through the "calculateDistance function"
volatile int maalstregscounter;				// counts how many times the car has driven over the finish line.
volatile int baneIndex = 1;					// notes in what swing on the track the car is in.
volatile int amountOfTurns = 0;				// saves the amount of turns of the track from the baneIndex variable, so that the baneIndex can be reset.
volatile int Timer1overflow = 0;			// Timer 1 overflow count
volatile int speed = 0;						//speed of the car
volatile int speedsum = 0;					//the sum of all the speed values in the speed buffer.
volatile int speedBuffer[8];				// "running" buffer for calculating the speed of the car.
volatile int speedBufferIndex = 0;			//index for the speed calculation buffer.
volatile int laptime = 0;					// Initialization to not have a "floating value".
volatile int udAfSvingDistance = 1;			// Controls how far away from the measured turn point the car begins to accelerate.






int main(void)
{
	
	sei();
	portsetup();
	INITuart();
	INITadc();
	INITexternalInterrupt();
	PWM_init();
	INITtimer0();
	INITtimer1();
	sendString("initDone");
	//==============
	_delay_ms(3000);
	
	setpointPWM =  0;
	OCR2 = (setpointPWM * 255)/100;
	
    while (1){
		sendString("maal bane function.");
		maaleBaneFunction();
		DriveSuperFastFunction();

	}
}

void portsetup(){
	
	DDRA = 0; //PORT A as input
	PORTA = 0x00; //internal resistor off. because of ADC
	
	DDRB = 0xFF; //set PORTB as output.
	DDRB &= ~(1<<DDB1); //set PB1 (timer1) as input.
	
	DDRC = 0xff;   // Set portC to output mode
	
	
	DDRD = 0; // set port D to input
	DDRD = (0<<DDD2)|(0<<DDD3); //PD2 and PD3 as input
	DDRD |= (1 << DDD7);	// Set PD7 (OC2) as output
	PORTD = 0x00; // Set internal resistor to OFF.

}

void INITuart(void){
	
	UBRRH = 0;
	//UBRRL = (F_CPU/(16*(baudRate))-1);		//Set baudrate to 9600
	#if (F_CPU == 16000000UL)
		UBRRL = 104; //16MHz
	
	#elif F_CPU == 8000000UL
			UBRRL = 52; //8MH
	
	#endif
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); // RX Complete interrupt enable, Reciever enable, Transmitter enable.
	
}

void INITadc(void){
	DDRA = 0; //PORT A as input
	PORTA = 0x00; //internal resistor off.
	ADMUX = (1<<ADLAR);//|(1<<MUX0) //AREF internal Vref turned off, Left adj, channel 0.
	ADCSRA = (1<<ADATE)|(1<<ADEN)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //ADC enable, ADC Start conversion, ADC Auto Trigger enable, Prescale = 128.
	SFIOR = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0); //ADC set to "Free Running Mode"
	}

void INITexternalInterrupt(void){
	MCUCR = (1<<ISC01)|(1<<ISC00) //rising edge interrupt detection on EXTERNAL INTERRUPT 0 (Goal sensor)
			|(1<<ISC11)|(1<<ISC10); //rising edge interrupt detection on EXTERNAL INTERRUPT 1 (Distance sensor)
	GICR = (1<<INT0)|(1<<INT1); //enables interrupt vector for EXT interrupt 1 and 0
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

void INITtimer1(){
	TCCR1A = 0x00; //Normal port operation, OC1A/OC1B disconnected, Timer/counter mode = Normal.
	TCCR1B = (1<<CS12)|(1<<CS11)|(0<<CS10); //external clock source on T1 pin. clock on falling edge.
	TIMSK |= (1<<TOIE1); //Timer1 overflow interrupt enable.

}

void PWM_init(){
	// Configure Timer/Counter2 for Fast PWM mode
	TCCR2 |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
	TCCR2 |= (1 << COM21);                // Clear OC2 on compare match, set at BOTTOM (non-inverting mode)
	//TCCR2 |= (1 << COM21)|(1<<COM20);     // Set OC2 on compare match, clear OC2 at Bottom (inverting mode)
	TCCR2 |= (1 << CS21);                 // Set prescaler to 8 (start the timer)
	
	// Set initial duty cycle (e.g., 50%)
	//OCR2 = 128; // 50% duty cycle (128/255)
}

void maaleBaneFunction(void){
	Old_state = STRAIGHT;
	int setpointPWM = 45;
	int distance = 0;
	
	_delay_ms(2000);
	maalstregscounter = 0;
	do {									// Drives until it finds the finish line.
	OCR2 = (setpointPWM * 255)/100;
	
	} while(maalstregscounter == 0);
	
	sendString("sensing....");
	countersReset(); //resets the Timer 1 counter
	
	do{										// Checks if if there is a change in drive state, if there is, logs it down into the bane[50] array
		if (Old_state != SwingState){
			Old_state =	SwingState;
			bane[baneIndex] = sampleCount;
			sendInt(baneIndex);
			sendString("");
			baneIndex++;	
			amountOfTurns = baneIndex;
		}
		
	}while(baneIndex < 50 && maalstregscounter < 2);// stops mapping when it has gone around the track once. 
	
	
	sendStringNoNewLine("bane Index: ");
	sendInt(amountOfTurns);

	for (int i = 1 ; i < amountOfTurns; i++){ // Prints the Location of swings and their identifier
		sendStringNoNewLine("Turn:");
		sendInt(i);
		sendStringNoNewLine("Distance from start: ");
		float distance = calculateDistance(bane[i]);
		sendInt((int)distance);	
		sendString("---------------------");
	}

}

void DriveSuperFastFunction(){
	int BreakDistance = 150;
	int BreakDistanceSF = 1;
	int distance;
	baneIndex = 1;
	int DriveState = NOTSWING;
	float INCfactor = 1; 
	udAfSvingDistance = 50;
	
	sendString("");
	sendString("Drive SUPER FAST Function....");
	sendString(" ");
	
	while(maalstregscounter < 2 ){ 
		OCR2 = (45 * 255)/100;
	}
	
	maalstregscounter = 1;
	setpointPWM = 60;
	
	while (1){

		INCfactor = 100*(1-exp(-0.3*maalstregscounter)); //Exponential function of how the car plans to increase the speed and brake distance.
		BreakDistanceSF = (INCfactor/100)+1;
		
		sendStringNoNewLine("inc factor:");
		sendInt(INCfactor);
		setpointPWM = INCfactor;
			if (setpointPWM<50){
				setpointPWM = 50;
			}
		sendStringNoNewLine("PWM:");
		sendInt(setpointPWM);
		
		switch (DriveState){
			case NOTSWING:{
				sendString("----");
				sendString("next swing");
				OCR2 = (setpointPWM * 255)/100;	
				
				sendStringNoNewLine("BaneIndex: ");
				sendInt(baneIndex);			
				sendStringNoNewLine("Location: ");
				sendInt(bane[baneIndex]);
				sendStringNoNewLine("BDSF: ");
				sendInt(BreakDistanceSF);
				sendString("--------");
				if (baneIndex > amountOfTurns)
					baneIndex = 1;
				while(1){
					if (sampleCount >= (bane[baneIndex]-BreakDistance*BreakDistanceSF)){
						brems();
						sendStringNoNewLine("braked at distance: ");
						sendInt(bane[baneIndex]-BreakDistance*BreakDistanceSF);
						DriveState = afterBRAKE;
						break;
					}
				}
				
			}
			
			case afterBRAKE:{
				sendString("afterbrake");
				sendString(" ");
				sendStringNoNewLine("bane index: ");
				sendInt(baneIndex);
				
				do{
					OCR2 = (45 * 255)/100;
					if(sampleCount >= bane[baneIndex]){
						baneIndex++;
						DriveState = SWING;
						break;
					}
				}while(baneIndex<50);
				
			}
			
			case SWING: {
				sendString("swing");
				while(1){
					if (sampleCount + udAfSvingDistance >= bane[baneIndex]){
						OCR2 = (45 * 255)/100;
						sendString("out of swing");
						if (baneIndex >= amountOfTurns)
							baneIndex = 1;
						else baneIndex++;
						
						DriveState = NOTSWING;
						break;
					}
				}
			}
			
		}
	}
}

int SwingDetector(int y){
	
	static int ACCerror = 0x05;
	
	switch (SwingState){
		case STRAIGHT:
		if (y > (SetpointLEFTturn)){
			SwingState = LEFT;
		}
		else if (y < (SetpointRIGHTturn)){
			SwingState = RIGHT;
		break;
		}
		
		case LEFT:
		if (y < (SetpointLEFTturn-ACCerror)){
			SwingState = STRAIGHT;	
		}
		break;
		
		case RIGHT:
		if (y > (SetpointRIGHTturn+ACCerror)){
			SwingState = STRAIGHT;
		}
		break;
		
	}
	return SwingState;
}

void brems(){
	sendString("BREEEEEMS");
	PORTC = 0b00000010;
	_delay_ms(150);
	PORTC = 0b00000000;
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
		uint8_t name[] = "UART TEST \n";
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

void clearUARTinputBuffer() {
	for (int i = 0; i < sizeof(UARTinputBuffer); i++) {
		UARTinputBuffer[i] = 0;
	}
	UARTbufferIndex = 0; // Optionally reset the index as well
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

float calculateDistance(int sampleCount){
	afstand = 5.1*sampleCount; //udregnet forhold i mellem "samples" og strækning.
	return afstand;
}

void calculateSpeedFunction(int dist2, int tid2){
    static int tid1 = 0, dist1 = 0;
    int deltaTid, deltaDist;

    deltaTid = tid2 - tid1;
    deltaDist = dist2 - dist1;

 
    if (deltaTid != 0) {   // Avoid division by zero
	    speed = deltaDist / deltaTid;
	    } 
		else {
	    speed = 0;
    }

    tid1 = tid2;
    dist1 = dist2;

}

//resets all "running" counters.
void countersReset(){ 
	sampleCount = 0;
	msec = 0;
	TCNT1 = 0;
	baneIndex = 1;
	
}

void calculateMovingAVGSpeedFunction(uint8_t newSpeedValue){
	
	speedsum -= speedBuffer[speedBufferIndex];
	
	speedBuffer[speedBufferIndex] = newSpeedValue;
	
	speedsum += newSpeedValue;
	
	speedBufferIndex = (adcBufferIndex + 1) % ADC_BUFFER_SIZE;
	
	speed = speedsum / ADC_BUFFER_SIZE;
}

void accelerometer(){
	
	if (--sampleTimeCounter > 0) return;
	sampleTimeCounter = sampleTime; 
	int newACCvalue = ADCH;
	updateMovingAverage(newACCvalue);
	ACCvalue = filteredADCValue;
	SwingState = SwingDetector(ACCvalue);
	
}

//ADC is in "free Running" mode and therefor the ISR is not needed.
ISR(ADC_vect){
	
	
}

ISR(USART_RXC_vect){
	
	receivedByte = UDR;
	UARTinputBuffer[UARTbufferIndex] = receivedByte;
	UARTbufferIndex = (UARTbufferIndex + 1) % sizeof(UARTinputBuffer);
}

ISR(INT0_vect){ //external interrupt PD2. maalstreg schmidtt 
    static uint16_t lastMaalstregTime = 0;
    uint16_t now = msec;
	sendStringNoNewLine("LT:");
	sendInt(msec);
	
    if ((uint16_t)(now - lastMaalstregTime) > 50) { // 50 ms debounce
	    lastMaalstregTime = now;
	    maalstregscounter++;
	    countersReset();
	    sendStringNoNewLine("Lap: ");
	    sendInt(maalstregscounter);
    }
    
}

ISR(INT1_vect){ // external interrupt PD3. distance
		sampleCount = TCNT1;
	
}

ISR(TIMER0_COMP_vect){ //every ms this interrupt triggers
	msec++;
	accelerometer();
}
ISR(TIMER1_OVF_vect){
	sendString("Timer 1 Overflow");
	Timer1overflow++;
	
}
