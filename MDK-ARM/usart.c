
#include "usart.h"

int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0); 
	USART1->DR = (uint8_t) ch;      
	return ch;
//	uint8_t c[1] = {ch};
//	HAL_UART_Transmit(&huart1,c,1,0xFFF);
//	return ch;
}