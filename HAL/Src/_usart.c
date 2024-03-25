#include "_usart.h"
#include "_config.h"
#include "link.h"

#include "debug.h"

USART_DMA_WRITE_FUNC_DEF(esp, ESP_UART_HANDLE);

void debugUartPutChar(int c) {
	HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*) &c, 1, 100);
}

void ESP_UART_HANDLE_IRQHandler() {
	linkBufferPutChar(ESP_UART_HANDLE.Instance->RDR);
	__HAL_UART_ENABLE_IT(&ESP_UART_HANDLE, UART_IT_RXNE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == ESP_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(esp);
	}
}

void _UART_Init() {
	UART_DMA_WRITE_SEM_INIT(esp);
	__HAL_UART_ENABLE_IT(&ESP_UART_HANDLE, UART_IT_RXNE);
}