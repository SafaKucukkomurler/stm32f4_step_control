#include "uartDriver.h"

void USART1_IRQHandler(void){

	uint32_t usart1_isrflags = USART1->SR;
	uint32_t usart1_control_reg = USART1->CR1;

	if (((usart1_isrflags & USART_SR_RXNE) != RESET) && ((usart1_control_reg & USART_CR1_RXNEIE) != RESET))
	{
		uartBuffRX.buffer[uartBuffRX.headPointer++] = USART1->DR;

		if(uartBuffRX.headPointer == BUFFER_SIZE)
		{
			uartBuffRX.headPointer = 0;
		}

		return;
	}

	if (((usart1_isrflags & USART_SR_TXE) != RESET) && ((usart1_control_reg & USART_CR1_TXEIE) != RESET))
	{
		if(uartBuffTX.headPointer != uartBuffTX.tailPointer)
		{
			USART1->DR = uartBuffTX.buffer[uartBuffTX.tailPointer++];

			if(uartBuffTX.tailPointer == BUFFER_SIZE)
			{
				uartBuffTX.tailPointer = 0;
			}
		}
		else
		{
			CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE);
		}

		return;
	}


}

void USART_SendByte(uint8_t data){

	uartBuffTX.buffer[uartBuffTX.headPointer++] = data;

	if(uartBuffTX.headPointer == BUFFER_SIZE)
	{
		uartBuffTX.headPointer = 0;
	}

	SET_BIT(USART1->CR1, USART_CR1_TXEIE);
}

uint8_t USART_IsBuffEmpty(volatile uartBufferT* buffer){

	if(buffer->headPointer == buffer->tailPointer)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int32_t USART_ReadByte(){

	int32_t receivedByte = 0;
	if(USART_IsBuffEmpty(&uartBuffRX) == 1)
	{
		receivedByte = -1;
	}
	else
	{
		receivedByte = uartBuffRX.buffer[uartBuffRX.tailPointer++];

		if(uartBuffRX.tailPointer==BUFFER_SIZE)
		{
			uartBuffRX.tailPointer = 0;
		}
	}

	return receivedByte;
}

uint32_t USART_BytesToRead(){

	if(uartBuffRX.headPointer >= uartBuffRX.tailPointer)
	{
		return (uartBuffRX.headPointer - uartBuffRX.tailPointer);
	}
	else
	{
		return(BUFFER_SIZE + (uartBuffRX.headPointer - uartBuffRX.tailPointer));
	}
}

void USART_SendByteArray(uint8_t* buffer, uint32_t size){

	uint32_t i;
	for(i=0; i<size; i++)
	{
		USART_SendByte(buffer[i]);
	}
}

