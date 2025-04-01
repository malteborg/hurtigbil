#include "DriveTheCarFunctions.h"
#include <avr/io.h>
#include <time.h>
#include <stdint.h>



//------------Input read------------
//Formats the input to listen after "Enter" and "backspace"
void USART_ReadUntilCR(char* buffer, uint8_t bufferSize)
{
	uint8_t index = 0;
	char EchoVar[8];
	
	
	while (!(UCSRA & (1 << RXC)))// Wait until data is received
	{
		char receivedChar = UDR; // Read the received character
		//UDR = receivedChar;
		
		if (receivedChar == 'BS') // search for Backspace
		return;
		
		if (receivedChar == '\r') { // Check for 'CR' (carriage return);
			return;
		}
		if (index < bufferSize - 1) { // Ensure we don't overflow the buffer
			buffer[index++] = receivedChar;
		}
		


		buffer[index] = '\0'; // Null-terminate the string
	}
}


//--------------- PWM ------------------------
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

//---------------DATA LOGGER-----------------



// Initialize Timer1
void Timer1_init() {
	TCCR1A = 0x00; // Normal mode
	TCCR1B = (1 << CS11); // Prescaler = 8
	TCNT1 = 0; // Initialize counter to 0
}



