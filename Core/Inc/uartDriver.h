#ifndef INC_UARTDRIVER_H_
#define INC_UARTDRIVER_H_

#include "stm32f4xx_hal.h"

#define BUFFER_SIZE 50

typedef struct uartBufferType
{
	uint32_t buffer[BUFFER_SIZE];
	uint32_t headPointer;
	uint32_t tailPointer;
} uartBufferT;

volatile uartBufferT uartBuffTX;
volatile uartBufferT uartBuffRX;

void USART_SendByte(uint8_t data);
uint8_t USART_IsBuffEmpty(volatile uartBufferT* buffer);
int32_t USART_ReadByte();
uint32_t USART_BytesToRead();
void USART_SendByteArray(uint8_t* buffer, uint32_t size);

#endif
