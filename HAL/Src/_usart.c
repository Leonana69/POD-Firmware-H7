#include "_usart.h"

void debugUartPutChar(int c) {
	HAL_UART_Transmit(&huart4, (uint8_t*) &c, 1, 100);
}

void UART_Init() {
	
}