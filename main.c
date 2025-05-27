


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>


#define baudRate 9600
// Define constants for SwingState values
#define STRAIGHT 0
#define LEFT 1
#define RIGHT 2
#define SWING 4
#define NOTSWING 5
#define afterBRAKE 6
//========= Moving average filter========
#define ADC_BUFFER_SIZE 25 // Number of samples for the moving average

volatile uint16_t adcBuffer[ADC_BUFFER_SIZE]; // Buffer to store ADC values
volatile uint8_t adcBufferIndex = 0;          // Index to track the current position in the buffer
volatile uint16_t adcSum = 0;                 // Sum of all values in the buffer
volatile uint16_t filteredADCValue = 0;       // Filtered ADC value (moving average)
//========= function declerations======
void portsetup(void);
void INITuart(void);
void INITadc(void);
void maaleBaneFunction(void);
void INITexternalInterrupt(void);
int SwingDetector(int y);
void PWM_init();
void brems(void);
void sendString(const char*);
void sendStringNoNewLine(const char* inputString);
void uarttest(void);
int ReadFromUART(char charRead);
int containsChar(volatile char* buffer, char target, int bufferSize);
void clearInputBuffer();
void sendInt(int number);
int stringToInt(const char* str);
void updateMovingAverage(uint16_t newValue);
void INITtimer0(void);
void accelerometer();
void maaleBaneFunctionMedTid(int antalSwing);
float calculateDistance(int sampleCount);
float calculateSpeedFunction(int dist2, int tid2);
void calculateMovingAVGSpeedFunction(uint8_t newSpeedValue);
void countersReset();
void DriveSuperFastFunction();
void INITtimer1();


volatile int ACCvalue = 0;
volatile int SwingState = STRAIGHT;
volatile int setpointPWM = 0;
volatile int readIntCMD;
volatile int SetpointRIGHTturn = 78; //dec = 70
volatile int SetpointLEFTturn = 92; //dec = 240
volatile int SetpointSTRAIGHT = 85; //dec = 127
volatile char inputBuffer[8]; // circular buffer
volatile int UARTbufferIndex = 0; //circular buffer index
volatile int UARTrecievedLatch = 0; //Sets the latch high when the uart recieve interrupt is triggered so that the code can react.
volatile int receivedByte; // byte from the UART UDR register.
volatile char commandchar; // char received from the UART UDR.
volatile uint16_t msec = 0;	//millisecond counter, incremented every millisecond.
static int sampleTime = 5; // how many times the sampleTimeCounter needs to increment for the to be taken another accelerometer measurement.
static int sampleTimeCounter;
volatile uint16_t sampleCount = 0;
volatile int bane[50] = {0}; // array of the track.
volatile uint16_t afstand = 0;
volatile int maalstregscounter; // counts how many times the car has driven over the finish line.
volatile int safetyFactor; // controls the regulation of speed each lap.
volatile int baneIndex = 1; //notes in what swing on the track the car is in.
volatile int amountOfTurns = 0; //saves the amount of turns of the track from the baneIndex variable, so that baneIndex can be reset.
volatile int Timer1overflow = 0;
volatile float speed = 0;		//speed of the car
volatile int speedsum = 0;  //the sum of all the speed values in the speed buffer.
volatile int speedBuffer[8];	//buffer for calculating the speed of the car.
volatile int speedBufferIndex = 0; //index for the speed calculation buffer.
volatile int udAfSvingDistance = 0;
volatile int DriveState = NOTSWING;
volatile int bremseFaktor = 5;
volatile int tid1 = 1, dist1 = 1;
volatile int bremseGraense = 35;
volatile int baneLaengde;







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
	//==============
	_delay_ms(3000);
	setpointPWM =  0;
	OCR2 = (setpointPWM * 255)/100;
	int mainstate = 1;
	int Old_state;
	
	
	

	sendString("initDone");
	
	sendString("maal bane function.");
	maaleBaneFunction();
	DriveSuperFastFunction();
	
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
	UBRRL = 104; //temporary to fix UART problems.
	
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

void maaleBaneFunction(void){
	

	
	int Old_state = STRAIGHT;
	int setpointPWM = 45;
	int distance = 0;
	
	_delay_ms(2000);
	maalstregscounter = 0;
	safetyFactor = 8;
	do {
	OCR2 = (setpointPWM * 255)/100;
	
	} while(maalstregscounter == 0);
	
	
	sendString("sensing....");
	countersReset();
	
	do{
		if (Old_state != SwingState){
			Old_state =	SwingState;
			bane[baneIndex] = sampleCount;
			sendInt(baneIndex);
			sendString("");
			amountOfTurns = baneIndex;
			baneIndex++;	
		}
		
	}while(baneIndex < 50 && maalstregscounter < 2);
	
	
	sendStringNoNewLine("bane Index: ");
	sendInt(amountOfTurns);
// 	 _delay_ms(900); // make the car drive a little longer because i?m lazy......
// 	OCR2 = 0;

/*
	for (int i = 1 ; i < amountOfTurns; i++){
		sendStringNoNewLine("Turn:");
		sendInt(i);
		sendStringNoNewLine("Distance from start: ");
		float distance = calculateDistance(bane[i]);
		sendInt((int)distance);	
		sendString("---------------------");
	}*/

}

void DriveSuperFastFunction(){
	int BreakDistance = 100;
	int BreakDistanceSF = 1;
	int distance;
	safetyFactor = 3;
	baneIndex = 1;
	DriveState = NOTSWING;
	float INCfactor = 1; 
	sendString("");
	sendString("Drive SUPER FAST Function....");
	sendString(" ");
	
	while(maalstregscounter < 2 ){ //wait for the 
		OCR2 = (45 * 255)/100;
	}
	
	maalstregscounter = 1;
	setpointPWM = 70;
	
	while (1){
		
/*		speed = calculateSpeedFunction(sampleCount, msec);*/
		
		INCfactor = 100*(1-exp(-0.3*maalstregscounter));
		/*BreakDistanceSF = (INCfactor/100)+1;*/
		
		sendStringNoNewLine("inc factor:");
		sendInt(INCfactor);
		setpointPWM = INCfactor;
			if (setpointPWM<50){
				setpointPWM = 70;
			}
/*
		sendStringNoNewLine("PWM:");
		sendInt(setpointPWM);*/
			
		sendString("-----");
		sendInt(maalstregscounter);
		
		switch (DriveState){
			case NOTSWING:{
				sendString("----");
				sendString("next swing");
				OCR2 = (setpointPWM * 255)/100;	
				
				sendStringNoNewLine("BaneIndex: ");
				sendInt(baneIndex);			
				sendStringNoNewLine("Location: ");
				sendInt(bane[baneIndex]);

				

				sendString("--------");
				if (baneIndex > amountOfTurns)
					baneIndex = 1;
				
/*
				speed = calculateSpeedFunction(sampleCount, msec);
				
				BreakDistance = speed * bremseFaktor;
				
				sendString("Speed: ");
				sendInt((int)speed);*/
				int afstandMellemSving;
				if(baneIndex == 1){
					afstandMellemSving = baneLaengde - bane[amountOfTurns] + bane[1];
					sendStringNoNewLine("Afstand maalstreg: ");
					sendInt(afstandMellemSving);
				}
				else{
					afstandMellemSving = bane[baneIndex+1] - bane[baneIndex];
					sendStringNoNewLine("Afstand mellem sving: ");
					sendInt(afstandMellemSving);
					}
				
				BreakDistance = afstandMellemSving * 0.3;

				
				while(1){
					if (sampleCount >= (bane[baneIndex]-BreakDistance*BreakDistanceSF)){
						/*if(speed>=bremseGraense) brems();*/
						brems();
						sendStringNoNewLine("braked at distance: ");
						sendInt(bane[baneIndex]-BreakDistance*BreakDistanceSF);
						DriveState = afterBRAKE;
						break;
					}
				}
				break;
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
						if (baneIndex >= amountOfTurns){
							baneIndex = 1;
						}
						baneIndex++;
						sendString("Sving increment");
						
						DriveState = NOTSWING;
						break;
					}
				}
			}
			
		}
	}
}
void maaleBaneFunctionMedTid(int antalSwing){
	int baneIndex = 0;
	int bremseVar = 1000;
	int bane[2*antalSwing];
	int Old_state;
	

	
	maalstregscounter = 0;
	safetyFactor = 4;
	msec = 0;
	OCR2 = (50 * 255)/100;
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
				OCR2 = (60 * 255)/100; 
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
	
	static int ACCerror = 0;
	
	switch (SwingState){
		case STRAIGHT:
		if (y > (SetpointLEFTturn)){
			SwingState = LEFT;
/*
			sendString("");
			sendString("Left");*/
/*
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("sampleCount fra start:");
			sendInt(sampleCount);
			*/
		}
		else if (y < (SetpointRIGHTturn)){
			SwingState = RIGHT;
/*
			sendString("");
			sendString("Right");*/
/*
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("sampleCount fra start:");
			sendInt(sampleCount);*/
		}
		break;
		
		
		case LEFT:
		if (y < (SetpointLEFTturn-ACCerror)){
			SwingState = STRAIGHT;
/*
			sendString("");
			sendString("Straight");*/
/*
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("sampleCount fra start:");
			sendInt(sampleCount);*/
			
		}
		break;
		
		case RIGHT:
		if (y > (SetpointRIGHTturn+ACCerror)){
			SwingState = STRAIGHT;
/*
			sendString("");
			sendString("Straight");*/
/*
			//sendInt(ACCvalue);
			
			sendStringNoNewLine("sampleCount fra start:");
			sendInt(sampleCount);*/
		}
		break;
		
	}
	return SwingState;
}

void PWM_init()
{
	// Configure Timer/Counter2 for Fast PWM mode
	TCCR2 |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
	TCCR2 |= (1 << COM21);                // Clear OC2 on compare match, set at BOTTOM (non-inverting mode)
	//TCCR2 |= (1 << COM21)|(1<<COM20);     // Set OC2 on compare match, clear OC2 at Bottom (inverting mode)
	TCCR2 |= (1 << CS21);                 // Set prescaler to 8 (start the timer)
	
	// Set initial duty cycle (e.g., 50%)
	//OCR2 = 128; // 50% duty cycle (128/255)
}

void brems(){
	sendString("BREEEEEMS");
	PORTC = 0b00000010;
	_delay_ms(100);
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

void clearInputBuffer() {
	for (int i = 0; i < sizeof(inputBuffer); i++) {
		inputBuffer[i] = 0;
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
	afstand = 5.1*sampleCount;
	return afstand;
}

float calculateSpeedFunction(int dist2, int tid2){
    int deltaTid, deltaDist;

    deltaTid = tid2 - tid1;
    deltaDist = dist2 - dist1;

 
	speed = deltaDist*100 / deltaTid;

    tid1 = tid2;
    dist1 = dist2;
	
	return speed;
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
    baneLaengde = sampleCount;
	
	sendStringNoNewLine("LT: ");
	sendInt(msec);
	
	static uint16_t lastMaalstregTime = 0;
    uint16_t now = msec;
    if ((uint16_t)(now - lastMaalstregTime) > 50) { // 50 ms debounce
	    lastMaalstregTime = now;
	    maalstregscounter++;
	    countersReset();
	    //safetyFactor = safetyFactor/2;
		DriveState = NOTSWING;
	    sendStringNoNewLine("Lap nr.: ");
	    sendInt(maalstregscounter);
    }
    
}

ISR(INT1_vect){ // external interrupt PD3. distance
		sampleCount = TCNT1;
	
}

ISR(TIMER0_COMP_vect){ //every ms this interrupt triggers
	msec++;
	accelerometer();
	//afstand = calculateDistance(sampleCount);
	//calculateSpeedFunction(afstand, msec);
	
	
}
ISR(TIMER1_OVF_vect){
	sendString("Timer 1 Overflow");
	Timer1overflow++;
	
}