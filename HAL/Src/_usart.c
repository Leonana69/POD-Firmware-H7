#include "_usart.h"
#include "_config.h"
#include "link.h"
#include "debug.h"

USART_DMA_READ_WRITE_FUNC_DEF(esp, ESP_UART_HANDLE);
USART_DMA_READ_WRITE_FUNC_DEF(debug, DEBUG_UART_HANDLE);
#ifdef GEAR
USART_DMA_READ_WRITE_FUNC_DEF(gear, GEAR_UART_HANDLE);
#endif

void debugUartTransmit(const uint8_t *data, uint16_t len) {
	// HAL_UART_Transmit(&DEBUG_UART_HANDLE, data, len, 100);
	debug_write_dma(data, len);
}

#define ESP_RX_BUFFER_SIZE 6
static uint8_t esp_rx_buffer_index = 0;
static uint8_t esp_rx_buffer[ESP_RX_BUFFER_SIZE * 2];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == ESP_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(esp);
	}

	if (huart->Instance == DEBUG_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(debug);
	}

#ifdef GEAR
	if (huart->Instance == GEAR_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(gear);
	}
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == ESP_UART_HANDLE.Instance) {
		uint8_t index = esp_rx_buffer_index;
		esp_rx_buffer_index = 1 - esp_rx_buffer_index;
		HAL_UART_Receive_DMA(&ESP_UART_HANDLE, esp_rx_buffer + ESP_RX_BUFFER_SIZE * esp_rx_buffer_index, ESP_RX_BUFFER_SIZE);
		for (int i = 0; i < ESP_RX_BUFFER_SIZE; i++) {
			linkBufferPutChar(esp_rx_buffer[i + ESP_RX_BUFFER_SIZE * index]);
		}
	}

#ifdef GEAR
	if (huart->Instance == GEAR_UART_HANDLE.Instance) {
		UART_DMA_RX_COMPLETE_CALLBACK(gear);
	}
#endif
}

void _UART_Init() {
	UART_DMA_READ_WRITE_SEM_INIT(esp);
	UART_DMA_READ_WRITE_SEM_INIT(debug);
	HAL_UART_Receive_DMA(&ESP_UART_HANDLE, esp_rx_buffer, ESP_RX_BUFFER_SIZE);

#ifdef GEAR
	UART_DMA_READ_WRITE_SEM_INIT(gear);
#endif
}