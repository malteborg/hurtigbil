#ifndef DRIVETHECARFUNCTIONS_H_
#define DRIVETHECARFUNCTIONS_H_

#include <stdint.h>



void USART_ReadUntilCR(char* buffer, uint8_t bufferSize);
void PWM_init();


#endif /* DRIVETHECARFUNCTIONS_H_ */